"""Gravity compensation for robotic arms using MuJoCo physics simulation."""
# ruff: noqa: BLE001, C901, PLR0912, SIM108, SIM102, TRY301, PLC0415, PLR2004, ANN201, ARG001, PERF401, S110

import argparse
import asyncio
import sys

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

import builtins
import contextlib

import can

from openarm.bus import Bus
from openarm.damiao import Arm, ControlMode, Motor, detect_motors
from openarm.damiao.config import MOTOR_CONFIGS
from openarm.simulation.models import OPENARM_MODEL_PATH


class ArmWithGravity(Arm):
    """Extended Arm class with gravity compensation support."""

    def __init__(self, motors: list[Motor], position: str, can_bus: can.BusABC) -> None:
        """Initialize the GravityArm with motors and position."""
        # Initialize parent Arm with all motors
        super().__init__(motors)

        # Store additional attributes needed for gravity compensation
        self.position = position  # "left" or "right"
        self.can_bus = can_bus
        self.positions = [0.0] * len(motors)  # Position for each motor


class MuJoCoKDL:
    """A simple class for computing inverse dynamics using MuJoCo."""

    def __init__(self) -> None:
        """Initialize MuJoCo model for kinematic/dynamic calculations."""
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
        """Compute inverse dynamics for the given joint states."""
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

    def __init__(self) -> None:
        """Initialize the gravity compensator with MuJoCo model."""
        self.kdl = MuJoCoKDL()
        self.tuning_factors = [0.8, 0.8, 1.0, 1.0, 1.0, 1.0, 1.0, 0.0]

    def compute(self, angles: list[float], position: str = "left") -> list[float]:
        """Compute gravity compensation torques for given joint angles.

        Args:
            angles: List of joint angles in radians
            position: "left" or "right" - determines if mirror motors should have
                negated torques

        Returns:
            List of gravity compensation torques for each joint

        """
        q = np.array(angles)

        gravity_torques = self.kdl.compute_inverse_dynamics(
            q, np.zeros(q.shape), np.zeros(q.shape), side=position
        )

        # Apply tuning factors
        return [
            torque * factor
            for torque, factor in zip(
                gravity_torques, self.tuning_factors, strict=False
            )
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
        help=(
            "CAN ports with position to use (e.g., --port can0:left --port can1:right)"
        ),
    )

    return parser.parse_args()


async def main(args: argparse.Namespace) -> None:
    """Run main gravity compensation loop with proper shutdown handling."""
    # Create all CAN buses
    try:
        all_can_buses = [
            can.Bus(channel=config["channel"], interface=config["interface"])
            for config in can.detect_available_configs("socketcan")
        ]
    except Exception:
        all_can_buses = []

    if not all_can_buses:
        return

    # Parse port:position pairs
    port_configs = []  # List of (port_name, position)
    for port_spec in args.port:
        try:
            parts = port_spec.split(":")
            if len(parts) != 2:
                msg = f"Invalid format: {port_spec}"
                raise ValueError(msg)
            port_name, position = parts
            if position not in ["left", "right"]:
                msg = f"Invalid position: {position}"
                raise ValueError(msg)
            port_configs.append((port_name, position))
        except ValueError:
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
        for bus in all_can_buses:
            bus.shutdown()
        return

    for bus, _position in selected_buses:
        bus_channel = (
            str(bus.channel_info) if hasattr(bus, "channel_info") else str(bus.channel)
        )
        # Extract just the channel name for cleaner display
        if "channel" in bus_channel:
            import re

            match = re.search(r"channel ['\"]?(\w+)", bus_channel)
            match.group(1) if match else bus_channel
        else:
            bus_channel.split()[-1] if bus_channel else "unknown"

    # Store arms for cleanup
    arms: list[ArmWithGravity] = []

    try:
        # Run gravity compensation for all selected buses together
        arms = await _main(args, selected_buses)
    finally:
        # SAFETY: Disable all motors first to avoid unwanted movements
        if arms:
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
    """Run gravity compensation loop for all selected buses with their positions.

    Returns:
        List of Arm objects for cleanup in main()

    """
    # Initialize gravity compensator
    gravity_comp = GravityCompensator()

    # Setup motors on all selected buses

    # Create Arm objects for each bus
    arms: list[ArmWithGravity] = []

    for _bus_idx, (can_bus, arm_position) in enumerate(selected_buses):
        # First use detect_motors to check if ALL motors are present
        slave_ids = [config.slave_id for config in MOTOR_CONFIGS]
        detected = list(detect_motors(can_bus, slave_ids, timeout=0.1))
        detected_ids = {info.slave_id for info in detected}

        # Check if ALL required motors are detected
        missing_motors = []
        for config in MOTOR_CONFIGS:
            if config.slave_id not in detected_ids:
                missing_motors.append(config.name)

        if missing_motors:
            continue

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

        try:
            # Enable all motors at once
            states = await arm.enable()

            # Set control mode for all motors at once
            await arm.set_control_mode(ControlMode.MIT)

            # Initialize positions from enable response
            for i, state in enumerate(states):
                if state:
                    arm.positions[i] = state.position

            # Successfully initialized, add to arms list
            arms.append(arm)

        except Exception:
            pass
            # Don't add to arms list - this arm is broken

    # Count total motors across all working arms
    total_motors = sum(len(arm.motors) for arm in arms)

    if total_motors == 0:
        return []

    # Report motors per arm
    for _arm_idx, _arm in enumerate(arms):
        pass

    # NOW set terminal to raw mode for keyboard detection during the main loop
    old_settings = None
    raw_mode = False
    if HAS_TERMIOS:
        try:
            old_settings = termios.tcgetattr(sys.stdin)
            tty.setraw(sys.stdin.fileno())
            raw_mode = True
        except (OSError, termios.error):
            # Might fail in some environments (e.g., when piped)
            pass

    # Helper function for printing in raw mode
    def raw_print(msg: str = "") -> None:
        """Print with proper line endings in raw mode."""
        if raw_mode:
            sys.stdout.flush()
        else:
            pass

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
            with contextlib.suppress(builtins.BaseException):
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

        # SAFETY: Disable all motors to avoid unwanted movements
        raw_print("Disabling all motors for safety...")
        for arm in arms:
            await arm.disable()

    # Return arms for cleanup in main()
    return arms


def run() -> None:
    """Run the gravity compensation script."""
    args = parse_arguments()

    try:
        asyncio.run(main(args))
    except KeyboardInterrupt:
        sys.exit(0)
    except Exception:
        sys.exit(1)


if __name__ == "__main__":
    run()
