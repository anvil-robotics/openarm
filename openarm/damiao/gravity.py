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
    
    def compute(self, angles: list[float]) -> list[float]:
        """Compute gravity compensation torques for given joint angles.
        
        Args:
            angles: List of joint angles in radians
            
        Returns:
            List of gravity compensation torques for each joint
        """
        q = np.array(angles)
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
        help="CAN ports to use (e.g., --port can0 --port can1). If not specified, auto-detects all ports.",
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
    
    # Filter buses based on args if specified
    if args.port:
        # Use only specified ports
        can_buses = []
        for bus in all_can_buses:
            # Check if this bus matches any requested port
            bus_channel = str(bus.channel_info) if hasattr(bus, 'channel_info') else str(bus.channel)
            for port in args.port:
                if port in bus_channel:
                    can_buses.append(bus)
                    break
        
        if not can_buses:
            print(f"Error: None of the specified ports {args.port} were found.")
            # Shutdown all buses before returning
            for bus in all_can_buses:
                bus.shutdown()
            return
        
        print(f"Using {len(can_buses)} selected bus(es): {args.port}")
    else:
        # Use all detected buses
        can_buses = all_can_buses
        print(f"Using all {len(can_buses)} bus(es)")
    
    try:
        # Run gravity compensation for each bus
        for bus_idx, can_bus in enumerate(can_buses):
            if len(can_buses) > 1:
                print(f"\nProcessing bus {bus_idx + 1} of {len(can_buses)}")
            await _main(args, can_bus)
    finally:
        # Proper shutdown of ALL CAN buses (not just the selected ones)
        for bus in all_can_buses:
            bus.shutdown()


async def _main(args: argparse.Namespace, can_bus) -> None:
    """Main gravity compensation loop for a single bus."""
    print("Setting up gravity compensation...")
    
    # Initialize gravity compensator
    gravity_comp = GravityCompensator()
    
    # Setup motors inline
    print("Testing motor connectivity...")
    
    motors = []
    current_positions = []
    all_motors_active = True
    
    for config in MOTOR_CONFIGS:
        print(f"Testing motor {config.name} (ID 0x{config.slave_id:02X})...")
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
                print(f"  Motor {config.name} active - Initial position: {state.position:.3f} rad")
            else:
                print(f"  Motor {config.name} inactive - No state received")
                all_motors_active = False
                motors.append(None)
                current_positions.append(0.0)
                
        except asyncio.TimeoutError:
            print(f"  Motor {config.name} inactive - Timeout")
            all_motors_active = False
            motors.append(None)
            current_positions.append(0.0)
        except Exception as e:
            print(f"  Motor {config.name} inactive - Error: {e}")
            all_motors_active = False
            motors.append(None)
            current_positions.append(0.0)
    
    # If any motor is inactive, disable all motors and return
    if not all_motors_active:
        print("\nError: Not all motors are active. Disabling all motors...")
        for motor in motors:
            if motor is not None:
                try:
                    await motor.disable()
                except Exception:
                    pass
        return
    
    print(f"\nStarting gravity compensation loop with {sum(1 for m in motors if m)} motors...")
    print("Press Ctrl+C to stop")
    
    try:
        while True:
            # Get current joint positions (only for active motors)
            active_positions = []
            active_indices = []
            for i, (motor, pos) in enumerate(zip(motors, current_positions)):
                if motor is not None:
                    active_positions.append(pos)
                    active_indices.append(i)

            if not active_positions:
                print("No active motors")
                break

            # Compute gravity compensation torques using the new function
            tuned_torques = gravity_comp.compute(active_positions)

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
                        print(f"Error preparing control for motor {motor_idx}: {e}")

            # Send all MIT control commands at once and await all responses
            if control_tasks:
                try:
                    states = await asyncio.gather(*control_tasks, return_exceptions=True)

                    # Update positions from all motor responses
                    for i, state in enumerate(states):
                        motor_idx = motor_indices[i]
                        if isinstance(state, Exception):
                            print(f"Error controlling motor {motor_idx}: {state}")
                        elif state:
                            current_positions[motor_idx] = state.position

                except Exception as e:
                    print(f"Error in batch control: {e}")

            # Small delay
            await asyncio.sleep(0.01)

    except KeyboardInterrupt:
        print("\nStopping gravity compensation...")

        # Disable all motors
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