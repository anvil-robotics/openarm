import argparse
import asyncio
import sys
from dataclasses import dataclass, field
from typing import Optional
import mujoco
import numpy as np

# Platform-specific imports for keyboard input
try:
    import select
    import termios
    import tty
    HAS_TERMIOS = True
except ImportError:
    HAS_TERMIOS = False
    
try:
    import msvcrt
    HAS_MSVCRT = True
except ImportError:
    HAS_MSVCRT = False

import can
from openarm.bus import Bus
from openarm.damiao import ControlMode, MitControlParams, Motor
from openarm.damiao.can_buses import create_can_bus
from openarm.damiao.config import MOTOR_CONFIGS
from openarm.simulation.models import OPENARM_MODEL_PATH


@dataclass
class Arm:
    """Represents a single robotic arm with its motors and configuration."""
    position: str  # "left" or "right"
    can_bus: can.BusABC  # The CAN bus for this arm
    motors: list[Optional[Motor]] = field(default_factory=list)  # Motors (None for inactive)
    positions: list[float] = field(default_factory=list)  # Current positions for each motor
    
    @property
    def active_motors(self) -> list[Motor]:
        """Get list of active (non-None) motors."""
        return [m for m in self.motors if m is not None]
    
    @property
    def active_count(self) -> int:
        """Count of active motors."""
        return len(self.active_motors)
    
    def get_active_positions(self) -> tuple[list[float], list[int]]:
        """Get positions and indices of active motors.
        Returns: (positions, indices) where indices map to original motor array"""
        positions = []
        indices = []
        for i, (motor, pos) in enumerate(zip(self.motors, self.positions)):
            if motor is not None:
                positions.append(pos)
                indices.append(i)
        return positions, indices
    
    async def disable_all_motors(self) -> None:
        """Safely disable all active motors."""
        for motor in self.motors:
            if motor is not None:
                try:
                    await motor.disable()
                except Exception:
                    pass

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

    def compute_inverse_dynamics(self, q: np.ndarray, qdot: np.ndarray, qdotdot: np.ndarray, side: str = 'left') -> np.ndarray:
        assert len(q) == len(qdot) == len(qdotdot)
        assert side in ['left', 'right'], "side must be 'left' or 'right'"
        
        length = len(q)
        
        if side == 'left':
            # Left joints: indices 0-7 (8 motors)
            joint_indices = slice(0, length)
        else:  # right
            # Right joints: indices 9-16 (8 motors), but input q is still 0-7
            joint_indices = slice(9, 9 + length)
        
        # Clear all joint states first
        self.data.qpos[:] = 0
        self.data.qvel[:] = 0
        self.data.qacc[:] = 0
        
        # Set joint states for the specified side
        self.data.qpos[joint_indices] = q
        self.data.qvel[joint_indices] = qdot
        self.data.qacc[joint_indices] = qdotdot
        
        mujoco.mj_inverse(self.model, self.data)
        return self.data.qfrc_inverse[joint_indices]


class GravityCompensator:
    """Gravity compensation calculator with persistent MuJoCo model."""
    
    def __init__(self):
        self.kdl = MuJoCoKDL()
        self.tuning_factors = [0.8, 0.8, 1.0, 1.0, 1.0, 1.0, 1.0, 0.0]
    
    def compute(self, angles: list[float], position: str = "left") -> list[float]:
        """Compute gravity compensation torques for given joint angles.
        
        Args:
            angles: List of joint angles in radians
            position: "left" or "right" - determines if mirror motors should have negated torques
            
        Returns:
            List of gravity compensation torques for each joint
        """
        q = np.array(angles)
        
        gravity_torques = self.kdl.compute_inverse_dynamics(
            q, 
            np.zeros(q.shape), 
            np.zeros(q.shape),
            side=position
        )
        
        # Apply tuning factors
        tuned_torques = [
            torque * factor 
            for torque, factor in zip(gravity_torques, self.tuning_factors)
        ]
        
        return tuned_torques


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
    
    # Store arms for cleanup
    arms: list[Arm] = []
    
    try:
        # Run gravity compensation for all selected buses together
        arms = await _main(args, selected_buses)
    finally:
        # SAFETY: Disable all motors first to avoid unwanted movements
        if arms:
            print("\nDisabling all motors for safety...")
            for arm in arms:
                await arm.disable_all_motors()
        
        # Then shutdown all CAN buses
        for bus in all_can_buses:
            bus.shutdown()


def check_keyboard_input():
    """Check if a key has been pressed (non-blocking)."""
    if HAS_MSVCRT:
        # Windows
        if msvcrt.kbhit():
            return msvcrt.getch().decode('utf-8', errors='ignore').lower()
    elif HAS_TERMIOS:
        # Unix/Linux/Mac
        if select.select([sys.stdin], [], [], 0)[0]:
            return sys.stdin.read(1).lower()
    return None


async def _main(args: argparse.Namespace, selected_buses: list) -> list[Arm]:
    """Main gravity compensation loop for all selected buses with their positions.
    
    Returns:
        List of Arm objects for cleanup in main()
    """
    print("Setting up gravity compensation...")
    
    # Set terminal to raw mode for immediate key detection (Unix/Linux/Mac)
    old_settings = None
    raw_mode = False
    if HAS_TERMIOS:
        try:
            old_settings = termios.tcgetattr(sys.stdin)
            tty.setraw(sys.stdin.fileno())
            raw_mode = True
        except:
            # Might fail in some environments (e.g., when piped)
            pass
    
    # Helper function for printing in raw mode
    def raw_print(msg: str = "") -> None:
        """Print with proper line endings in raw mode."""
        if raw_mode:
            print(msg.replace('\n', '\r\n'), end='')
            sys.stdout.flush()
        else:
            print(msg)
    
    # Initialize gravity compensator
    gravity_comp = GravityCompensator()
    
    # Setup motors on all selected buses
    raw_print("Testing motor connectivity on all selected buses...")
    
    # Create Arm objects for each bus
    arms: list[Arm] = []
    
    for bus_idx, (can_bus, arm_position) in enumerate(selected_buses):
        raw_print(f"\nBus {bus_idx + 1}/{len(selected_buses)} ({arm_position} arm):")
        
        # Create new Arm object
        arm = Arm(position=arm_position, can_bus=can_bus)
        
        for config in MOTOR_CONFIGS:
            raw_print(f"  Testing motor {config.name} (ID 0x{config.slave_id:02X})...")
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
                    arm.motors.append(motor)
                    arm.positions.append(state.position)  # Use position from enable response
                    raw_print(f"    Motor {config.name} active - Initial position: {state.position:.3f} rad")
                else:
                    raw_print(f"    Motor {config.name} inactive - No state received")
                    arm.motors.append(None)
                    arm.positions.append(0.0)
                    
            except asyncio.TimeoutError:
                raw_print(f"    Motor {config.name} inactive - Timeout")
                arm.motors.append(None)
                arm.positions.append(0.0)
            except Exception as e:
                raw_print(f"    Motor {config.name} inactive - Error: {e}")
                arm.motors.append(None)
                arm.positions.append(0.0)
        
        arms.append(arm)
    
    # Count total active motors across all arms
    total_active_motors = sum(arm.active_count for arm in arms)
    
    if total_active_motors == 0:
        raw_print("\nError: No active motors found on any bus.")
        return []
    
    # Report active motors per arm
    for arm_idx, arm in enumerate(arms):
        raw_print(f"Arm {arm_idx + 1} ({arm.position}): {arm.active_count} active motors")
    
    raw_print(f"\nStarting gravity compensation loop with {total_active_motors} total motors...")
    raw_print("Press 'Q' to stop")
    
    try:
        while True:
            # Check for 'Q' key press
            key = check_keyboard_input()
            if key == 'q':
                break
            
            # Process each arm
            for arm_idx, arm in enumerate(arms):
                # Get active positions and indices
                active_positions, active_indices = arm.get_active_positions()
                
                if not active_positions:
                    continue  # Skip this arm if no active motors
                
                # Compute gravity compensation torques
                tuned_torques = gravity_comp.compute(active_positions, position=arm.position)
                
                # Prepare MIT control commands for all motors
                control_tasks = []
                motor_indices = []
                
                for motor_idx, motor in enumerate(arm.motors):
                    if motor is not None:
                        try:
                            # Find the torque for this motor
                            active_idx = active_indices.index(motor_idx)
                            torque = tuned_torques[active_idx]
                            
                            # MIT control with torque only
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
                            raw_print(f"Error preparing control for motor {motor_idx} on arm {arm_idx + 1}: {e}")
                
                # Send all MIT control commands at once
                if control_tasks:
                    try:
                        states = await asyncio.gather(*control_tasks, return_exceptions=True)
                        
                        # Update positions from motor responses
                        for i, state in enumerate(states):
                            motor_idx = motor_indices[i]
                            if isinstance(state, Exception):
                                raw_print(f"Error controlling motor {motor_idx} on arm {arm_idx + 1}: {state}")
                            elif state:
                                arm.positions[motor_idx] = state.position
                                
                    except Exception as e:
                        raw_print(f"Error in batch control on arm {arm_idx + 1}: {e}")

            # Small delay
            await asyncio.sleep(0.01)

        raw_print("\nStopping gravity compensation...")
        
    except Exception as e:
        raw_print(f"\nError in gravity compensation loop: {e}")
        
    finally:
        # Restore terminal settings (Unix/Linux/Mac)
        if old_settings is not None and HAS_TERMIOS:
            try:
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
            except:
                pass
        
        # SAFETY: Disable all motors to avoid unwanted movements
        raw_print("Disabling all motors for safety...")
        for arm in arms:
            await arm.disable_all_motors()
    
    # Return arms for cleanup in main()
    return arms


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