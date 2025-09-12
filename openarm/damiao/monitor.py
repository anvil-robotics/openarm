"""Simple motor angle monitor - disables motors and shows current angles.

This script disables all Damiao motors and displays their current angles.
"""
# ruff: noqa: T201, E501, C901, BLE001, PLR0912, TRY400, ARG001, PLC0415, SIM108, TRY301, TRY003, EM102, PLR2004

import argparse
import asyncio
import logging
import sys
from dataclasses import dataclass, field
from math import pi

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

from . import ControlMode, MitControlParams, Motor, PosVelControlParams
from .can_buses import create_can_bus
from .config import MOTOR_CONFIGS
from .detect import detect_motors
from .gravity import GravityCompensator

# ANSI color codes for terminal output
RED = "\033[91m"
GREEN = "\033[92m"
RESET = "\033[0m"

# Set up logging
logger = logging.getLogger(__name__)


def check_keyboard_input() -> str | None:
    """Check if a key has been pressed (non-blocking)."""
    if HAS_MSVCRT and msvcrt.kbhit():
        return msvcrt.getch().decode("utf-8", errors="ignore").lower()
    if HAS_TERMIOS and select.select([sys.stdin], [], [], 0)[0]:
        return sys.stdin.read(1).lower()
    return None


@dataclass
class Arm:
    """Represents a single robotic arm with its motors and configuration."""

    position: str  # "left" or "right"
    can_bus: can.BusABC  # The CAN bus for this arm
    channel: str  # Channel name (e.g., "can0", "can1")
    motors: list[Motor | None] = field(default_factory=list)
    states: list = field(default_factory=list)  # Current states for each motor
    is_master: bool = False  # Whether this arm is a master
    is_slave: bool = False  # Whether this arm is a slave
    mirror_mode: bool = False  # Whether mirror mode is enabled (for slaves)
    follows: str | None = None  # Channel name of master (for slaves)

    @property
    def active_motors(self) -> list[Motor]:
        """Get list of active (non-None) motors."""
        return [m for m in self.motors if m is not None]

    @property
    def active_count(self) -> int:
        """Count of active motors."""
        return len(self.active_motors)

    async def disable_all_motors(self) -> None:
        """Safely disable all active motors."""
        for motor in self.motors:
            if motor is not None:
                try:
                    await motor.disable()
                except Exception as e:
                    logger.debug("Failed to disable motor: %s", e)

    async def enable_all_motors(self, control_mode: ControlMode) -> None:
        """Enable all active motors with specified control mode."""
        for idx, motor in enumerate(self.motors):
            if motor is not None:
                try:
                    await motor.enable()
                    await motor.set_control_mode(control_mode)
                    logger.info("Motor %d: Enabled", idx + 1)
                    print(f"    Motor {idx + 1}: Enabled")
                except Exception as e:
                    logger.error("Motor %d: Error - %s", idx + 1, e)
                    print(f"{RED}    Motor {idx + 1}: Error - {e}{RESET}")

    async def refresh_states(self) -> None:
        """Refresh states for all motors."""
        new_states = []
        for motor in self.motors:
            if motor:
                try:
                    state = await motor.refresh_status()
                    new_states.append(state)
                except Exception as e:
                    logger.debug("Failed to refresh motor status: %s", e)
                    new_states.append(None)
            else:
                new_states.append(None)
        self.states = new_states


async def main(args: argparse.Namespace) -> None:
    """Run the monitor with the provided arguments."""
    # Create CAN buses
    can_buses = create_can_bus(args.interface)
    if not can_buses:
        return None

    print(f"\nDetected {len(can_buses)} CAN bus(es)")

    try:
        return await _main(args, can_buses)
    finally:
        for bus in can_buses:
            bus.shutdown()

async def _main(args: argparse.Namespace, can_buses: list[can.BusABC]) -> None:
    # Detect motors on each bus
    all_bus_motors = []
    has_missing_motor = False

    for bus_idx, can_bus in enumerate(can_buses):
        print(f"\nScanning for motors on bus {bus_idx + 1}...")
        slave_ids = [config.slave_id for config in MOTOR_CONFIGS]

        # Detect motors using raw CAN bus
        detected = list(detect_motors(can_bus, slave_ids, timeout=0.1))

        print(f"\nBus {bus_idx + 1} Motor Status:")

        # Create lookup for detected motors by slave ID
        detected_lookup = {info.slave_id: info for info in detected}

        # Check all expected motors and their status
        bus_motors = []
        for config in MOTOR_CONFIGS:
            if config.slave_id not in detected_lookup:
                # Motor is not detected
                print(
                    f"  {RED}✗{RESET} {config.name}: ID 0x{config.slave_id:02X} "
                    f"(Master: 0x{config.master_id:02X}) {RED}[NOT DETECTED]{RESET}"
                )
                bus_motors.append(None)
                has_missing_motor = True
            elif detected_lookup[config.slave_id].master_id != config.master_id:
                # Motor is detected but master ID doesn't match
                detected_info = detected_lookup[config.slave_id]
                print(
                    f"  {RED}✗{RESET} {config.name}: ID 0x{config.slave_id:02X} "
                    f"{RED}[MASTER ID MISMATCH: Expected 0x{config.master_id:02X}, "
                    f"Got 0x{detected_info.master_id:02X}]{RESET}"
                )
                bus_motors.append(None)
                has_missing_motor = True
            else:
                # Motor is connected and configured correctly
                print(
                    f"  {GREEN}✓{RESET} {config.name}: ID 0x{config.slave_id:02X} "
                    f"(Master: 0x{config.master_id:02X})"
                )
                # Create motor instance
                bus = Bus(can_bus)
                motor = Motor(
                    bus,
                    slave_id=config.slave_id,
                    master_id=config.master_id,
                    motor_type=config.type,
                )
                bus_motors.append(motor)

        all_bus_motors.append(bus_motors)

    # Exit if any motor is missing
    if has_missing_motor:
        print(
            f"\n{RED}Error: Not all motors are detected or configured "
            f"correctly. Exiting.{RESET}"
        )
        return

    # Count total detected motors
    total_motors = sum(
        1 for bus_motors in all_bus_motors for m in bus_motors if m is not None
    )
    if total_motors == 0:
        print(f"\n{RED}Error: No motors detected on any bus.{RESET}")
        return

    print(
        f"\n{GREEN}Total {total_motors} motors detected across "
        f"{len(can_buses)} bus(es){RESET}"
    )

    # Disable all motors on all buses
    print("\nDisabling all motors...")
    all_state_results = []
    for bus_idx, bus_motors in enumerate(all_bus_motors):
        bus_states = []
        for motor in bus_motors:
            if motor:
                try:
                    state = await motor.disable()
                    bus_states.append(state)
                except Exception as e:
                    logger.error(
                        "Error disabling motor on bus %d: %s", bus_idx + 1, e
                    )
                    print(
                        f"{RED}Error disabling motor on bus {bus_idx + 1}: {e}{RESET}"
                    )
                    bus_states.append(None)
            else:
                bus_states.append(None)
        all_state_results.append(bus_states)

    # Call teleop or monitor based on flag
    if args.teleop:
        await teleop(can_buses, all_bus_motors, all_state_results, args)
    else:
        await monitor_motors(can_buses, all_bus_motors, all_state_results, args)

async def monitor_motors(
    can_buses: list[can.BusABC],
    all_bus_motors: list[list[Motor | None]],
    all_state_results: list[list],
    args: argparse.Namespace,
) -> None:
    """Monitor motor angles continuously and display them in a table format.

    Args:
        can_buses: List of CAN bus interfaces.
        all_bus_motors: List of motor lists for each bus.
        all_state_results: Initial state results for each motor.
        args: Command-line arguments.

    """
    # Start continuous monitoring with column display
    print("\nContinuously monitoring motor angles (Ctrl+C to stop):\n")

    # Print header with bus labels
    header = "  Motor"
    for bus_idx in range(len(can_buses)):
        header += f"        Bus {bus_idx + 1}     "
    print(header)
    print("  " + "-" * (len(header) - 2))

    # Print initial lines for each motor
    for config in MOTOR_CONFIGS:
        line = f"  {config.name:<12}"
        for _ in range(len(can_buses)):
            line += "  Initializing...  "
        print(line)

    # Number of motors (lines to move up)
    num_motors = len(MOTOR_CONFIGS)

    # Use disable results for first display
    all_current_states = all_state_results

    try:
        while True:
            # Move cursor up to the first motor line
            print(f"\033[{num_motors}A", end="")

            # Print current states for all buses
            for motor_idx, config in enumerate(MOTOR_CONFIGS):
                line = f"\r  {config.name:<12}"
                for bus_idx in range(len(can_buses)):
                    state = all_current_states[bus_idx][motor_idx]
                    if state:
                        # Show absolute angle
                        angle_deg = state.position * 180 / pi
                        line += f"  {angle_deg:+8.2f}°     "
                    elif all_bus_motors[bus_idx][motor_idx] is None:
                        line += "       N/A        "
                    else:
                        line += "    No state      "
                print(line + "\033[K")

            # Small delay before refresh
            await asyncio.sleep(0.1)

            # Refresh states for all buses
            new_all_states = []
            for bus_motors in all_bus_motors:
                bus_states = []
                for motor in bus_motors:
                    if motor:
                        try:
                            state = await motor.refresh_status()
                            bus_states.append(state)
                        except Exception as e:
                            logger.debug("Failed to refresh motor status: %s", e)
                            bus_states.append(None)
                    else:
                        bus_states.append(None)
                new_all_states.append(bus_states)
            all_current_states = new_all_states

    except KeyboardInterrupt:
        # Move cursor below all motor lines
        print(f"\033[{num_motors}B")
        print("\nMonitoring stopped.")


async def teleop(
    can_buses: list[can.BusABC],
    all_bus_motors: list[list[Motor | None]],
    all_state_results: list[list],
    args: argparse.Namespace,
) -> None:
    """Teleoperation mode - masters use MIT, slaves use PosVel."""
    # Initialize gravity compensator if enabled
    gravity_comp = None
    if args.gravity:
        print("Initializing gravity compensation...")
        gravity_comp = GravityCompensator()

    # Create Arm objects for each bus
    arms: list[Arm] = []
    channel_to_arm = {}  # Maps channel name to Arm object

    for bus_idx, (can_bus, bus_motors, bus_states) in enumerate(zip(can_buses, all_bus_motors, all_state_results, strict=False)):
        # Get channel info from the bus
        channel_info = str(can_bus.channel_info) if hasattr(can_bus, "channel_info") else str(can_bus.channel)
        # Extract channel name (e.g., "can0" from various formats)
        if "channel" in channel_info:
            # For socketcan: extract from "SocketcanBus channel 'can0'"
            import re
            match = re.search(r"channel ['\"]?(\w+)", channel_info)
            if match:
                channel_name = match.group(1)
            else:
                channel_name = f"bus{bus_idx}"
        else:
            # For USB devices, use the product name or bus index
            channel_name = channel_info.split()[-1] if channel_info else f"bus{bus_idx}"

        # Create Arm object
        arm = Arm(
            position="unknown",  # Will be set based on --follow arguments
            can_bus=can_bus,
            channel=channel_name,
            motors=bus_motors,
            states=bus_states
        )
        arms.append(arm)
        channel_to_arm[channel_name] = arm
        print(f"Bus {bus_idx + 1}: Channel '{channel_name}'")

    # Parse --follow arguments if provided
    if args.follow:
        for follow_spec in args.follow:
            try:
                parts = follow_spec.split(":")
                if len(parts) != 4:
                    raise ValueError(f"Invalid format: {follow_spec}")

                # Parse MASTER:POSITION:SLAVE:POSITION
                master_ch, master_pos, slave_ch, slave_pos = parts

                # Validate positions
                if master_pos not in ["left", "right"]:
                    raise ValueError(f"Invalid master position: {master_pos}")
                if slave_pos not in ["left", "right"]:
                    raise ValueError(f"Invalid slave position: {slave_pos}")

                # Validate channels exist
                if master_ch not in channel_to_arm:
                    print(f"{RED}Error: Master channel '{master_ch}' not found{RESET}")
                    return
                if slave_ch not in channel_to_arm:
                    print(f"{RED}Error: Slave channel '{slave_ch}' not found{RESET}")
                    return

                # Get Arm objects
                master_arm = channel_to_arm[master_ch]
                slave_arm = channel_to_arm[slave_ch]

                # Check for conflicts
                if slave_arm.is_slave:
                    print(f"{RED}Error: Slave '{slave_ch}' already follows '{slave_arm.follows}'{RESET}")
                    return

                # Configure master arm
                master_arm.position = master_pos
                master_arm.is_master = True

                # Configure slave arm
                slave_arm.position = slave_pos
                slave_arm.is_slave = True
                slave_arm.follows = master_ch
                slave_arm.mirror_mode = (master_pos != slave_pos)  # Auto-detect mirror mode

            except ValueError:
                print(f"{RED}Error: Invalid follow format '{follow_spec}'. Use MASTER:POSITION:SLAVE:POSITION where POSITION is 'left' or 'right'{RESET}")
                print(f"{RED}Example: --follow can0:left:can1:right (mirror) or --follow can0:left:can1:left (no mirror){RESET}")
                return

        # Validate no channel is both master and slave
        for arm in arms:
            if arm.is_master and arm.is_slave:
                print(f"{RED}Error: Channel {arm.channel} cannot be both master and slave{RESET}")
                return
    # Default behavior: first arm is master, others are slaves
    elif len(arms) > 1:
        master_arm = arms[0]
        master_arm.is_master = True
        master_arm.position = "left"  # Default position

        for slave_arm in arms[1:]:
            slave_arm.is_slave = True
            slave_arm.follows = master_arm.channel
            slave_arm.mirror_mode = False  # Default: no mirror
            slave_arm.position = "left"  # Default position

    print("\nMaster-Slave Configuration:")
    # Group slaves by master
    for master_arm in [arm for arm in arms if arm.is_master]:
        slaves = []
        for slave_arm in [arm for arm in arms if arm.is_slave and arm.follows == master_arm.channel]:
            mirror_str = "(mirror)" if slave_arm.mirror_mode else ""
            slave_str = f"{slave_arm.channel}:{slave_arm.position}{mirror_str}"
            slaves.append(slave_str)
        if slaves:
            print(f"  Master: {master_arm.channel}:{master_arm.position} -> Slaves: {', '.join(slaves)}")

    # Enable all motors (masters with MIT, slaves with PosVel)
    print("\nEnabling motors for teleoperation...")
    for arm in arms:
        if arm.is_slave:
            print(f"  {arm.channel}: Enabling motors with Position-Velocity control (slave)")
            await arm.enable_all_motors(ControlMode.POS_VEL)
        else:  # master
            print(f"  {arm.channel}: Enabling motors with MIT control (master)")
            await arm.enable_all_motors(ControlMode.MIT)

    # Start teleoperation with monitoring display
    print("\nTeleoperation mode starting...\n")

    # Print header with bus labels showing channel names and roles
    header = "  Motor"
    for arm in arms:
        if arm.is_slave:
            # Slave: show S* for mirror mode, S for normal
            role = "S*" if arm.mirror_mode else "S"
        else:
            # Master
            role = "M"
        header += f"   {arm.channel}({role})   "
    print(header)
    print("  " + "-" * (len(header) - 2))

    # Print initial lines for each motor
    for config in MOTOR_CONFIGS:
        line = f"  {config.name:<12}"
        for _ in arms:
            line += "  Initializing...  "
        print(line)

    # Number of motors (lines to move up)
    num_motors = len(MOTOR_CONFIGS)

    # Print stop instruction before entering raw mode
    print("Press 'Q' to stop" if HAS_TERMIOS else "Press Ctrl+C to stop")

    # Set terminal to raw mode for keyboard detection
    old_settings = None
    raw_mode = False
    if HAS_TERMIOS:
        try:
            old_settings = termios.tcgetattr(sys.stdin)
            tty.setraw(sys.stdin.fileno())
            raw_mode = True
        except (OSError, termios.error) as e:
            # Might fail in some environments
            logger.debug("Failed to set raw mode: %s", e)

    # Helper for raw mode printing
    def raw_print(msg: str = "") -> None:
        if raw_mode:
            print(msg.replace("\n", "\r\n"), end="")
            sys.stdout.flush()
        else:
            print(msg)

    try:
        # Small initial delay to ensure display is ready
        await asyncio.sleep(0.1)

        while True:
            # Check for 'Q' key press
            if raw_mode:
                key = check_keyboard_input()
                if key == "q":
                    raw_print("\nStopping teleoperation...")
                    break

            # Move cursor up to the first motor line (add +1 for the "Press Q" line)
            print(f"\033[{num_motors + 1}A", end="")

            # Print current states for all arms
            for motor_idx, config in enumerate(MOTOR_CONFIGS):
                line = f"\r  {config.name:<12}"
                for arm in arms:
                    if motor_idx < len(arm.states):
                        state = arm.states[motor_idx]
                        if state:
                            # Show absolute angle
                            angle_deg = state.position * 180 / pi
                            line += f"  {angle_deg:+8.2f}°     "
                        elif motor_idx >= len(arm.motors) or arm.motors[motor_idx] is None:
                            line += "       N/A        "
                        else:
                            line += "    No state      "
                    else:
                        line += "       N/A        "
                print(line + "\033[K")

            # Small delay before refresh
            await asyncio.sleep(0.01)

            # Control master arms with MIT (gravity comp or zero torque)
            master_arms = {arm.channel: arm for arm in arms if arm.is_master}
            for master_arm in master_arms.values():
                new_states = []

                # Calculate gravity compensation if enabled
                gravity_torques = None
                active_indices = []

                if gravity_comp and master_arm.position in ["left", "right"]:
                    # Get only active motor positions (like gravity.py does)
                    active_positions = []
                    for idx, (motor, state) in enumerate(zip(master_arm.motors, master_arm.states, strict=False)):
                        if motor is not None and state:
                            active_positions.append(state.position)
                            active_indices.append(idx)

                    if active_positions:
                        # Compute gravity compensation with only active positions
                        gravity_torques = gravity_comp.compute(active_positions, position=master_arm.position)

                # Apply MIT control to all motors
                for motor_idx, motor in enumerate(master_arm.motors):
                    if motor:
                        try:
                            # Determine torque value
                            torque = 0.0
                            if gravity_torques and motor_idx in active_indices:
                                active_idx = active_indices.index(motor_idx)
                                if active_idx < len(gravity_torques):
                                    torque = gravity_torques[active_idx]

                            # MIT control with zero torque or gravity compensation
                            params = MitControlParams(
                                q=0,         # No position control
                                dq=0,        # No velocity control
                                kp=0,        # Zero position gain (passive)
                                kd=0,        # No damping
                                tau=torque   # Zero or gravity compensation
                            )
                            state = await motor.control_mit(params)
                            new_states.append(state)
                        except Exception as e:
                            logger.debug("MIT control failed: %s", e)
                            new_states.append(None)
                    else:
                        new_states.append(None)

                master_arm.states = new_states

            # Update slave arms based on their masters
            for slave_arm in [arm for arm in arms if arm.is_slave]:
                if slave_arm.follows and slave_arm.follows in master_arms:
                    master_arm = master_arms[slave_arm.follows]

                    # Control each motor
                    new_states = []
                    for motor_idx, (slave_motor, master_state) in enumerate(zip(slave_arm.motors, master_arm.states, strict=False)):
                        if slave_motor and master_state:
                            try:
                                # Get master position
                                position = master_state.position

                                # Apply mirror if enabled for this slave AND motor supports it
                                if slave_arm.mirror_mode and motor_idx < len(MOTOR_CONFIGS) and MOTOR_CONFIGS[motor_idx].mirror:
                                    position = -position

                                # Always use Position-Velocity control for slaves
                                params = PosVelControlParams(
                                    position=position,
                                    velocity=1
                                )
                                state = await slave_motor.control_pos_vel(params)
                                new_states.append(state)
                            except Exception as e:
                                logger.debug("Position-velocity control failed: %s", e)
                                new_states.append(None)
                        else:
                            new_states.append(None)
                    slave_arm.states = new_states

    except KeyboardInterrupt:
        # Move cursor below all motor lines
        print(f"\033[{num_motors}B")
        if not raw_mode:
            print("\nTeleoperation stopped.")
        else:
            raw_print("\nTeleoperation stopped.")

    finally:
        # Restore terminal settings first
        if old_settings is not None and HAS_TERMIOS:
            try:
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
            except (OSError, termios.error) as e:
                logger.debug("Failed to restore terminal settings: %s", e)

        # SAFETY: Disable ALL motors (not just slaves) for safety
        print("\nDisabling ALL motors for safety...")
        for arm in arms:
            await arm.disable_all_motors()
        print("All motors disabled.")

def parse_arguments() -> argparse.Namespace:
    """Parse command-line arguments."""
    parser = argparse.ArgumentParser(
        description="Monitor Damiao motor angles"
    )

    parser.add_argument(
        "--interface",
        "-i",
        default="can0",
        help="CAN interface name (default: can0, ignored on Windows/macOS)",
    )

    parser.add_argument(
        "--teleop",
        "-t",
        action="store_true",
        default=False,
        help="Enable teleoperation mode (enables motors with control mode)",
    )

    parser.add_argument(
        "--follow",
        action="append",
        help=(
            "Define follower mappings as MASTER:POSITION:SLAVE:POSITION "
            "where POSITION is 'left' or 'right'. "
            "Mirror mode is automatic when positions differ. "
            "(e.g., --follow can0:left:can1:right for mirror, "
            "--follow can0:left:can1:left for no mirror)"
        ),
    )

    parser.add_argument(
        "--gravity",
        "-g",
        action="store_true",
        help="Enable gravity compensation (MIT mode only)",
    )

    return parser.parse_args()


def run() -> None:
    """Entry point for the monitor script."""
    args = parse_arguments()

    try:
        asyncio.run(main(args))
    except KeyboardInterrupt:
        print("\nInterrupted by user.")
        sys.exit(0)
    except Exception as e:
        logger.error("Fatal error: %s", e)
        print(f"Error: {e}")
        sys.exit(1)


if __name__ == "__main__":
    run()
