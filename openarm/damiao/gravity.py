import argparse
import asyncio
import sys
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
from openarm.damiao import Arm, ControlMode, MitControlParams, Motor, detect_motors
from openarm.damiao.can_buses import create_can_bus
from openarm.damiao.config import MOTOR_CONFIGS
from openarm.simulation.models import OPENARM_MODEL_PATH


class ArmWithGravity(Arm):
    """Extended Arm class with gravity compensation support."""

    def __init__(self, motors: list[Motor], position: str, can_bus: can.BusABC):
        # Initialize parent Arm with all motors
        super().__init__(motors)

        # Store additional attributes needed for gravity compensation
        self.position = position  # "left" or "right"
        self.can_bus = can_bus
        self.positions = [0.0] * len(motors)  # Position for each motor


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

    def compute_inverse_dynamics(
        self, q: np.ndarray, qdot: np.ndarray, qdotdot: np.ndarray, side: str = "left"
    ) -> np.ndarray:
        assert len(q) == len(qdot) == len(qdotdot)
        assert side in ["left", "right"], "side must be 'left' or 'right'"

        length = len(q)

        if side == "left":
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
            q, np.zeros(q.shape), np.zeros(q.shape), side=position
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
            parts = port_spec.split(":")
            if len(parts) != 2:
                raise ValueError(f"Invalid format: {port_spec}")
            port_name, position = parts
            if position not in ["left", "right"]:
                raise ValueError(f"Invalid position: {position}")
            port_configs.append((port_name, position))
        except ValueError as e:
            print(
                f"Error: Invalid port format '{port_spec}'. Use PORT:POSITION where POSITION is 'left' or 'right'"
            )
            for bus in all_can_buses:
                bus.shutdown()
            return

    # Filter buses based on specified ports and attach position
    selected_buses = []  # List of (bus, position)
    for bus in all_can_buses:
        bus_channel = (
            str(bus.channel_info) if hasattr(bus, "channel_info") else str(bus.channel)
        )
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
        bus_channel = (
            str(bus.channel_info) if hasattr(bus, "channel_info") else str(bus.channel)
        )
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
    arms: list[ArmWithGravity] = []

    try:
        # Run gravity compensation for all selected buses together
        arms = await _main(args, selected_buses)
    finally:
        # SAFETY: Disable all motors first to avoid unwanted movements
        if arms:
            print("\nDisabling all motors for safety...")
            for arm in arms:
                await arm.disable()

        # Then shutdown all CAN buses
        for bus in all_can_buses:
            bus.shutdown()


def check_keyboard_input():
    """Check if a key has been pressed (non-blocking)."""
    if HAS_MSVCRT:
        # Windows
        if msvcrt.kbhit():
            return msvcrt.getch().decode("utf-8", errors="ignore").lower()
    elif HAS_TERMIOS:
        # Unix/Linux/Mac
        if select.select([sys.stdin], [], [], 0)[0]:
            return sys.stdin.read(1).lower()
    return None


async def _main(args: argparse.Namespace, selected_buses: list) -> list[ArmWithGravity]:
    """Main gravity compensation loop for all selected buses with their positions.

    Returns:
        List of Arm objects for cleanup in main()
    """
    print("Setting up gravity compensation...")

    # Initialize gravity compensator
    gravity_comp = GravityCompensator()

    # Setup motors on all selected buses
    print("Testing motor connectivity on all selected buses...")

    # Create Arm objects for each bus
    arms: list[ArmWithGravity] = []

    for bus_idx, (can_bus, arm_position) in enumerate(selected_buses):
        print(f"\nBus {bus_idx + 1}/{len(selected_buses)} ({arm_position} arm):")

        # First use detect_motors to check if ALL motors are present
        print("  Checking for motors...")
        slave_ids = [config.slave_id for config in MOTOR_CONFIGS]
        detected = list(detect_motors(can_bus, slave_ids, timeout=0.1))
        detected_ids = {info.slave_id for info in detected}

        # Check if ALL required motors are detected
        missing_motors = []
        for config in MOTOR_CONFIGS:
            if config.slave_id not in detected_ids:
                missing_motors.append(config.name)

        if missing_motors:
            print(f"  ERROR: Missing motors: {', '.join(missing_motors)}")
            print(f"  Skipping this arm (incomplete - needs ALL motors)")
            continue

        print(f"  All {len(MOTOR_CONFIGS)} motors detected!")

        # Create ALL motors for this arm
        motors = []
        for config in MOTOR_CONFIGS:
            bus = Bus(can_bus)
            motor = Motor(
                bus,
                slave_id=config.slave_id,
                master_id=config.master_id,
                motor_type=config.type,
            )
            motors.append(motor)

        # Create ArmWithGravity with ALL motors
        arm = ArmWithGravity(motors=motors, position=arm_position, can_bus=can_bus)

        print(f"  Enabling all {len(motors)} motors...")
        try:
            # Enable all motors at once
            states = await arm.enable()

            # Set control mode for all motors at once
            await arm.set_control_mode(ControlMode.MIT)

            # Initialize positions from enable response
            for i, state in enumerate(states):
                if state:
                    arm.positions[i] = state.position
                    print(
                        f"    {MOTOR_CONFIGS[i].name}: Initial position {state.position:.3f} rad"
                    )

            # Successfully initialized, add to arms list
            arms.append(arm)
            print(f"  Arm ready with {len(motors)} motors")

        except Exception as e:
            print(f"  ERROR: Arm initialization failed - {e}")
            print(f"  Skipping this arm (unusable)")
            # Don't add to arms list - this arm is broken

    # Count total motors across all working arms
    total_motors = sum(len(arm.motors) for arm in arms)

    if total_motors == 0:
        print("\nError: No working arms found.")
        return []

    # Report motors per arm
    for arm_idx, arm in enumerate(arms):
        print(f"Arm {arm_idx + 1} ({arm.position}): {len(arm.motors)} motors")

    print(f"\nStarting gravity compensation loop with {total_motors} total motors...")
    print("Press 'Q' to stop")

    # NOW set terminal to raw mode for keyboard detection during the main loop
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
            print(msg.replace("\n", "\r\n"), end="")
            sys.stdout.flush()
        else:
            print(msg)

    try:
        while True:
            # Check for 'Q' key press
            key = check_keyboard_input()
            if key == "q":
                break

            # Process each arm
            for arm_idx, arm in enumerate(arms):
                # Compute gravity compensation torques for all motors
                torques = gravity_comp.compute(arm.positions, position=arm.position)

                # Use Arm's batch control method for all motors at once
                try:
                    states = await arm.control_mit(
                        kp=0,  # No position gain
                        kd=0,  # No damping gain
                        q=0,  # No position control
                        dq=0,  # No velocity control
                        tau=torques,  # Gravity compensation torques for all motors
                    )

                    # Update positions from motor responses
                    for i, state in enumerate(states):
                        if state:
                            arm.positions[i] = state.position

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
            await arm.disable()

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
