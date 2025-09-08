import argparse
import asyncio
import sys
import mujoco
import numpy as np

from openarm.bus import Bus
from openarm.damiao import ControlMode, MitControlParams, Motor
from openarm.damiao.can_buses import create_can_bus
from openarm.damiao.config import MOTOR_CONFIGS
from openarm.simulation.models import OPENARM_MODEL_PATH

class MuJoCoKDL:
    """A simple class for computing inverse dynamics using MuJoCo."""

    def __init__(self):
        self.model = mujoco.MjModel.from_xml_path(str(OPENARM_MODEL_PATH))
        self.model.opt.gravity = np.array([0, 0, -9.81])

        self.data = mujoco.MjData(self.model)

        # Disable all collisions
        self.model.geom_contype[:] = 0
        self.model.geom_conaffinity[:] = 0

        # Disable all joint limit
        self.model.jnt_limited[:] = 0

    def compute_inverse_dynamics(self, q: np.ndarray, qdot: np.ndarray, qdotdot: np.ndarray) -> np.ndarray:
        assert len(q) == len(qdot) == len(qdotdot)
        length = len(q)
        self.data.qpos[:length] = q
        self.data.qvel[:length] = qdot
        self.data.qacc[:length] = qdotdot
        mujoco.mj_inverse(self.model, self.data)
        return self.data.qfrc_inverse[:length]


class GravityCompensator:
    """Gravity compensation calculator with persistent MuJoCo model."""
    
    def __init__(self):
        self.kdl = MuJoCoKDL()
        self.tuning_factors = [0.8, 0.8, 1.0, 1.0, 1.0, 1.0, 1.0, 0.0]
    
    def compute(self, angles: list[float], position: str = "left") -> list[float]:
        """Compute gravity compensation torques for given joint angles.
        
        Args:
            angles: List of joint angles in radians
            position: "left" or "right" - determines if mirror motors should be negated
            
        Returns:
            List of gravity compensation torques for each joint
        """
        q = np.array(angles)
        
        # Apply negation for right arm on mirror motors
        if position == "right":
            for i in range(min(len(q), len(MOTOR_CONFIGS))):
                if MOTOR_CONFIGS[i].mirror:
                    q[i] = -q[i]
        
        gravity_torques = self.kdl.compute_inverse_dynamics(
            q, 
            np.zeros(q.shape), 
            np.zeros(q.shape)
        )
        
        return [
            torque * factor 
            for torque, factor in zip(gravity_torques, self.tuning_factors)
        ]


def parse_arguments() -> argparse.Namespace:
    """Parse command-line arguments."""
    parser = argparse.ArgumentParser(
        description="Gravity compensation for Damiao motors"
    )
    
    parser.add_argument(
        "--port",
        action="append",
        required=True,
        help="CAN ports with position to use (e.g., --port can0:left --port can1:right)",
    )
    
    return parser.parse_args()

async def main(args: argparse.Namespace) -> None:
    """Main entry point with proper shutdown handling."""
    # Create all CAN buses
    all_can_buses = create_can_bus()
    
    if not all_can_buses:
        print("Error: No CAN buses detected.")
        return
    
    print(f"\nDetected {len(all_can_buses)} CAN bus(es) total")
    
    # Parse port:position pairs
    port_configs = []  # List of (port_name, position)
    for port_spec in args.port:
        try:
            parts = port_spec.split(':')
            if len(parts) != 2:
                raise ValueError(f"Invalid format: {port_spec}")
            port_name, position = parts
            if position not in ["left", "right"]:
                raise ValueError(f"Invalid position: {position}")
            port_configs.append((port_name, position))
        except ValueError as e:
            print(f"Error: Invalid port format '{port_spec}'. Use PORT:POSITION where POSITION is 'left' or 'right'")
            for bus in all_can_buses:
                bus.shutdown()
            return
    
    # Filter buses based on specified ports and attach position
    selected_buses = []  # List of (bus, position)
    for bus in all_can_buses:
        bus_channel = str(bus.channel_info) if hasattr(bus, 'channel_info') else str(bus.channel)
        for port_name, position in port_configs:
            if port_name in bus_channel:
                selected_buses.append((bus, position))
                break
    
    if not selected_buses:
        print(f"Error: None of the specified ports were found.")
        for bus in all_can_buses:
            bus.shutdown()
        return
    
    print(f"Using {len(selected_buses)} selected bus(es):")
    for bus, position in selected_buses:
        bus_channel = str(bus.channel_info) if hasattr(bus, 'channel_info') else str(bus.channel)
        # Extract just the channel name for cleaner display
        if "channel" in bus_channel:
            import re
            match = re.search(r"channel ['\"]?(\w+)", bus_channel)
            if match:
                bus_name = match.group(1)
            else:
                bus_name = bus_channel
        else:
            bus_name = bus_channel.split()[-1] if bus_channel else "unknown"
        print(f"  {bus_name}: {position} arm")
    
    try:
        # Run gravity compensation for all selected buses together
        await _main(args, selected_buses)
    finally:
        # Proper shutdown of ALL CAN buses (not just the selected ones)
        for bus in all_can_buses:
            bus.shutdown()


async def _main(args: argparse.Namespace, selected_buses: list) -> None:
    """Main gravity compensation loop for all selected buses with their positions."""
    print("Setting up gravity compensation...")
    
    # Initialize gravity compensator
    gravity_comp = GravityCompensator()
    
    # Setup motors on all selected buses
    print("Testing motor connectivity on all selected buses...")
    
    all_motors = []  # List of motor lists for each bus
    all_positions = []  # List of position lists for each bus
    all_arm_positions = []  # List of "left" or "right" for each bus
    
    for bus_idx, (can_bus, arm_position) in enumerate(selected_buses):
        print(f"\nBus {bus_idx + 1}/{len(selected_buses)} ({arm_position} arm):")
        motors = []
        current_positions = []
        all_arm_positions.append(arm_position)
        
        for config in MOTOR_CONFIGS:
            print(f"  Testing motor {config.name} (ID 0x{config.slave_id:02X})...")
            bus = Bus(can_bus)
            motor = Motor(
                bus,
                slave_id=config.slave_id,
                master_id=config.master_id,
                motor_type=config.type,
            )
            
            try:
                # Enable and get initial state from enable response
                state = await asyncio.wait_for(motor.enable(), timeout=1.0)
                await asyncio.wait_for(motor.set_control_mode(ControlMode.MIT), timeout=1.0)
                
                if state:
                    motors.append(motor)
                    current_positions.append(state.position)  # Use position from enable response
                    print(f"    Motor {config.name} active - Initial position: {state.position:.3f} rad")
                else:
                    print(f"    Motor {config.name} inactive - No state received")
                    motors.append(None)
                    current_positions.append(0.0)
                    
            except asyncio.TimeoutError:
                print(f"    Motor {config.name} inactive - Timeout")
                motors.append(None)
                current_positions.append(0.0)
            except Exception as e:
                print(f"    Motor {config.name} inactive - Error: {e}")
                motors.append(None)
                current_positions.append(0.0)
        
        all_motors.append(motors)
        all_positions.append(current_positions)
    
    # Count total active motors across all buses
    total_active_motors = sum(1 for bus_motors in all_motors for m in bus_motors if m is not None)
    
    if total_active_motors == 0:
        print("\nError: No active motors found on any bus.")
        return
    
    # Report active motors per bus
    for bus_idx, motors in enumerate(all_motors):
        active_count = sum(1 for m in motors if m is not None)
        print(f"Bus {bus_idx + 1}: {active_count} active motors")
    
    print(f"\nStarting gravity compensation loop with {total_active_motors} total motors...")
    print("Press Ctrl+C to stop")
    
    try:
        while True:
            # Process each bus
            for bus_idx, (motors, current_positions, arm_position) in enumerate(
                zip(all_motors, all_positions, all_arm_positions)
            ):
                # Get current joint positions (only for active motors)
                active_positions = []
                active_indices = []
                for i, (motor, pos) in enumerate(zip(motors, current_positions)):
                    if motor is not None:
                        active_positions.append(pos)
                        active_indices.append(i)

                if not active_positions:
                    continue  # Skip this bus if no active motors

                # Compute gravity compensation torques with position parameter
                tuned_torques = gravity_comp.compute(active_positions, position=arm_position)

                # Prepare MIT control commands for all motors
                control_tasks = []
                motor_indices = []

                for motor_idx, motor in enumerate(motors):
                    if motor is not None:
                        try:
                            # Find the torque for this motor
                            active_idx = active_indices.index(motor_idx)
                            torque = tuned_torques[active_idx]

                            # MIT control with torque only (position, velocity, gains set to 0)
                            params = MitControlParams(
                                q=0,        # No position control
                                dq=0,       # No velocity control
                                kp=0,       # No position gain
                                kd=0,       # No damping gain
                                tau=torque  # Pure torque control
                            )

                            # Create control task
                            task = motor.control_mit(params)
                            control_tasks.append(task)
                            motor_indices.append(motor_idx)

                        except Exception as e:
                            print(f"Error preparing control for motor {motor_idx} on bus {bus_idx + 1}: {e}")

                # Send all MIT control commands at once and await all responses
                if control_tasks:
                    try:
                        states = await asyncio.gather(*control_tasks, return_exceptions=True)

                        # Update positions from all motor responses
                        for i, state in enumerate(states):
                            motor_idx = motor_indices[i]
                            if isinstance(state, Exception):
                                print(f"Error controlling motor {motor_idx} on bus {bus_idx + 1}: {state}")
                            elif state:
                                all_positions[bus_idx][motor_idx] = state.position

                    except Exception as e:
                        print(f"Error in batch control on bus {bus_idx + 1}: {e}")

            # Small delay
            await asyncio.sleep(0.01)

    except KeyboardInterrupt:
        print("\nStopping gravity compensation...")

        # Disable all motors on all buses
        for bus_idx, motors in enumerate(all_motors):
            for motor in motors:
                if motor is not None:
                    try:
                        await motor.disable()
                    except Exception:
                        pass


def run() -> None:
    """Entry point for the gravity compensation script."""
    args = parse_arguments()
    
    try:
        asyncio.run(main(args))
    except KeyboardInterrupt:
        print("\nInterrupted by user.")
        sys.exit(0)
    except Exception as e:
        print(f"Error: {e}")
        sys.exit(1)


if __name__ == "__main__":
    run()