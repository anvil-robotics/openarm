"""Track min/max angle ranges for Damiao motors.

This script disables all Damiao motors and continuously displays their current angles
along with the minimum and maximum angles observed during the session.
"""

import argparse
import asyncio
import contextlib
import sys
from dataclasses import dataclass
from math import inf, pi

from openarm.utils import Display, TableDisplay

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

from .config import MOTOR_CONFIGS
from .detect import detect_motors
from .encoding import ControlMode, MitControlParams, PosVelControlParams
from .motor import Motor

# ANSI color codes for terminal output
RED = "\033[91m"
GREEN = "\033[92m"
YELLOW = "\033[93m"
CYAN = "\033[96m"
RESET = "\033[0m"

# Coverage and position thresholds
MIN_COVERAGE_PERCENT = 98
MAX_COVERAGE_PERCENT = 102
MAX_POSITION_ERROR_DEG = 10


def check_keyboard_input() -> str | None:
    """Check if a key has been pressed (non-blocking)."""
    if HAS_MSVCRT and msvcrt.kbhit():
        return msvcrt.getch().decode("utf-8", errors="ignore").lower()
    if HAS_TERMIOS and select.select([sys.stdin], [], [], 0)[0]:
        return sys.stdin.read(1).lower()
    return None


@dataclass
class AngleTracker:
    """Track min/max angles for a motor."""

    min_angle: float = inf
    max_angle: float = -inf
    current_angle: float = 0.0

    def update(self, angle: float) -> None:
        """Update current angle and min/max tracking."""
        self.current_angle = angle
        self.min_angle = min(self.min_angle, angle)
        self.max_angle = max(self.max_angle, angle)

    def reset(self) -> None:
        """Reset min/max tracking."""
        self.min_angle = inf
        self.max_angle = -inf


def target_angle(
    config: tuple[float, float], tracker: tuple[float, float], angle: float = 0
) -> float:
    """Calculate target position to map a desired angle based on tracked range.

    Args:
        config: (min_angle, max_angle) from motor config
        tracker: (min_angle, max_angle) from observed tracking
        angle: Desired angle in config space to map (default: 0)

    Returns:
        Target position in degrees in tracker space

    """
    config_min, config_max = config
    tracker_min, tracker_max = tracker

    observed_span = tracker_max - tracker_min
    config_span = config_max - config_min

    if config_span == 0:
        msg = "Config range span is zero"
        raise ValueError(msg)

    # Map desired angle from config space to tracker space
    # Formula: pos = tracker.min + (tracker.max - tracker.min) *
    #               (angle - config.min) / (config.max - config.min)
    return tracker_min + observed_span * (angle - config_min) / config_span


async def set_zero(
    motors_list: list[Motor | None],
    trackers_list: list[AngleTracker | None],
    side: str,
) -> None:
    """Set zero position for all motors based on tracked ranges."""
    sys.stdout.write(f"\r\n{CYAN}Setting zero position for all motors...{RESET}\r\n")

    for motor, tracker, config in zip(
        motors_list, trackers_list, MOTOR_CONFIGS, strict=False
    ):
        if motor is None or tracker is None:
            continue

        if tracker.min_angle == inf or tracker.max_angle == -inf:
            sys.stdout.write(
                f"  {YELLOW}✗{RESET} {config.name}: No data collected, skipping\r\n"
            )
            continue

        try:
            # Get side-specific config angles
            config_min_angle = (
                config.min_angle_left if side == "left" else config.min_angle_right
            )
            config_max_angle = (
                config.max_angle_left if side == "left" else config.max_angle_right
            )

            # Calculate target position where zero should be set
            target_pos_deg = target_angle(
                config=(config_min_angle, config_max_angle),
                tracker=(tracker.min_angle, tracker.max_angle),
                angle=0,
            )
            target_pos_rad = target_pos_deg * pi / 180

            # Set to PosVel control mode
            await motor.set_control_mode(ControlMode.POS_VEL)

            # Move to calculated position
            params = PosVelControlParams(position=target_pos_rad, velocity=1.0)
            await motor.control_pos_vel(params)

            # Small delay to reach position
            await asyncio.sleep(2)

            # Disable motor (required for set_zero_position)
            await motor.disable()

            # Set zero position and save
            await motor.set_zero_position()
            await motor.save_parameters()

            sys.stdout.write(f"  {GREEN}✓{RESET} {config.name}: Zero set\r\n")

        except Exception as e:  # noqa: BLE001
            sys.stdout.write(f"  {RED}✗{RESET} {config.name}: Error - {e}\r\n")

    sys.stdout.write(f"\r\n{GREEN}Zero position setting complete!{RESET}\r\n")


async def main(args: argparse.Namespace) -> None:
    """Run the angle range tracker."""
    # Create single CAN bus from arguments
    try:
        can_bus = can.Bus(channel=args.channel, interface=args.interface)
    except Exception as e:  # noqa: BLE001
        sys.stderr.write(f"{RED}Error: Failed to create CAN bus: {e}{RESET}\n")
        return None

    sys.stdout.write(
        f"\n{GREEN}Connected to {args.channel} (interface: {args.interface}){RESET}\n"
    )

    try:
        return await _main(can_bus, args.side)
    finally:
        can_bus.shutdown()


async def _main(can_bus: can.BusABC, side: str) -> None:
    """Process motors on the bus and track angle ranges."""
    # Detect motors on the bus
    sys.stdout.write(f"\r\n{CYAN}Scanning for motors...{RESET}\r\n")
    slave_ids = [config.slave_id for config in MOTOR_CONFIGS]

    # Detect motors using raw CAN bus
    detected = list(detect_motors(can_bus, slave_ids, timeout=0.1))

    sys.stdout.write("\r\nMotor Status:\r\n")

    # Create lookup for detected motors by slave ID
    detected_lookup = {info.slave_id: info for info in detected}

    # Check all expected motors and their status
    motors_list = []
    trackers_list = []
    has_missing_motor = False

    for config in MOTOR_CONFIGS:
        if config.slave_id not in detected_lookup:
            # Motor is not detected
            sys.stderr.write(
                f"  {RED}✗{RESET} {config.name}: ID 0x{config.slave_id:02X} "
                f"(Master: 0x{config.master_id:02X}) {RED}[NOT DETECTED]{RESET}\n"
            )
            motors_list.append(None)
            trackers_list.append(None)
            has_missing_motor = True
        elif detected_lookup[config.slave_id].master_id != config.master_id:
            # Motor is detected but master ID doesn't match
            detected_info = detected_lookup[config.slave_id]
            sys.stderr.write(
                f"  {RED}✗{RESET} {config.name}: ID 0x{config.slave_id:02X} "
                f"{RED}[MASTER ID MISMATCH: Expected 0x{config.master_id:02X}, "
                f"Got 0x{detected_info.master_id:02X}]{RESET}\n"
            )
            motors_list.append(None)
            trackers_list.append(None)
            has_missing_motor = True
        else:
            # Motor is connected and configured correctly
            sys.stdout.write(
                f"  {GREEN}✓{RESET} {config.name}: ID 0x{config.slave_id:02X} "
                f"(Master: 0x{config.master_id:02X})\n"
            )
            # Create motor instance
            bus = Bus(can_bus)
            motor = Motor(
                bus,
                slave_id=config.slave_id,
                master_id=config.master_id,
                motor_type=config.type,
            )
            motors_list.append(motor)
            trackers_list.append(AngleTracker())

    # Exit if any motor is missing
    if has_missing_motor:
        sys.stderr.write(
            f"\n{RED}Error: Not all motors are detected or configured "
            f"correctly. Exiting.{RESET}\n"
        )
        return

    # Count total detected motors
    total_motors = sum(1 for m in motors_list if m is not None)
    if total_motors == 0:
        sys.stderr.write(f"\n{RED}Error: No motors detected.{RESET}\n")
        return

    sys.stdout.write(f"\r\n{GREEN}Total {total_motors} motors detected{RESET}\r\n")

    # Enable motors with MIT control mode (zero torque for passive tracking)
    sys.stdout.write("\r\nEnabling motors with MIT control (zero torque)...\r\n")
    for motor in motors_list:
        if motor:
            try:
                await motor.enable()
                await motor.set_control_mode(ControlMode.MIT)
                # Send zero torque command (passive mode)
                params = MitControlParams(q=0, dq=0, kp=0, kd=0, tau=0)
                await motor.control_mit(params)
            except Exception as e:  # noqa: BLE001
                sys.stderr.write(f"{RED}Error enabling motor: {e}{RESET}\n")

    # Start angle tracking
    await track_angles(motors_list, trackers_list, side)


async def track_angles(  # noqa: C901, PLR0912
    motors_list: list[Motor | None],
    trackers_list: list[AngleTracker | None],
    side: str,
) -> None:
    """Track angle ranges for all motors continuously."""
    sys.stdout.write(
        f"\n{GREEN}Tracking angle ranges "
        f"(Press 'S' to set zero and exit, 'Q' to quit){RESET}\n\n"
    )

    # Initialize table display
    num_motors = len(MOTOR_CONFIGS)
    # +2 for header and separator line
    display = Display()
    display.set_height(num_motors + 2)

    # Define column widths: Motor(10), Target Zero(12), Current(12), Min(12),
    # Max(12), Config Min(12), Config Max(12), Coverage(12), Status(30)
    # Alignment: Motor=left, all numeric columns=right, Status=left
    table = TableDisplay(
        display,
        columns_length=[10, 12, 12, 12, 12, 12, 12, 12, 30],
        align=[
            "left",
            "right",
            "right",
            "right",
            "right",
            "right",
            "right",
            "right",
            "right",
        ],
    )

    # Set header row (row 0)
    table.row(
        0,
        [
            "Motor",
            "Target Zero",
            "Current",
            "Min",
            "Max",
            "Config Min",
            "Config Max",
            "Coverage",
            "Status",
        ],
    )

    # Set separator line (row 1) using display.line directly
    display.line(1, "-" * 124)

    # Set initial data lines (starting from row 2)
    for idx, config in enumerate(MOTOR_CONFIGS):
        table.row(idx + 2, [config.name, "", "Initializing...", "", "", "", "", "", ""])

    # Render initial table
    display.render()

    # Set terminal to raw mode for keyboard detection
    old_settings = None
    raw_mode = False
    if HAS_TERMIOS:
        try:
            old_settings = termios.tcgetattr(sys.stdin)
            tty.setraw(sys.stdin.fileno())
            raw_mode = True
        except (OSError, termios.error):
            pass

    try:
        while True:
            # Check for keyboard input
            if raw_mode:
                key = check_keyboard_input()
                if key == "q":
                    break
                if key == "s":
                    # Set zero position and exit
                    await set_zero(motors_list, trackers_list, side)
                    break  # Exit after setting zero

            # Update and display each motor's angles
            for motor_idx, config in enumerate(MOTOR_CONFIGS):
                motor = motors_list[motor_idx]
                tracker = trackers_list[motor_idx]

                row_idx = motor_idx + 2  # +2 for header and separator

                if motor is None:
                    table.row(row_idx, [config.name, "", "N/A", "", "", "", "", "", ""])
                elif tracker is None:
                    table.row(
                        row_idx, [config.name, "", "No tracker", "", "", "", "", "", ""]
                    )
                else:
                    try:
                        # Refresh motor status
                        state = await motor.refresh_status()
                        if state:
                            # Update tracker with current angle in degrees
                            angle_deg = state.position * 180 / pi
                            tracker.update(angle_deg)

                            # Format display values (no manual spacing)
                            current = f"{tracker.current_angle:+.2f}°"
                            min_val = (
                                f"{tracker.min_angle:+.2f}°"
                                if tracker.min_angle != inf
                                else "N/A"
                            )
                            max_val = (
                                f"{tracker.max_angle:+.2f}°"
                                if tracker.max_angle != -inf
                                else "N/A"
                            )
                            config_min_angle = (
                                config.min_angle_left
                                if side == "left"
                                else config.min_angle_right
                            )
                            config_max_angle = (
                                config.max_angle_left
                                if side == "left"
                                else config.max_angle_right
                            )
                            config_min = f"{config_min_angle:+.0f}°"
                            config_max = f"{config_max_angle:+.0f}°"

                            # Calculate coverage percentage
                            if tracker.min_angle != inf and tracker.max_angle != -inf:
                                observed_span = tracker.max_angle - tracker.min_angle
                                config_span = config_max_angle - config_min_angle
                                if config_span > 0:
                                    coverage = (observed_span / config_span) * 100
                                    # Color code based on coverage range
                                    if (
                                        MIN_COVERAGE_PERCENT
                                        <= coverage
                                        <= MAX_COVERAGE_PERCENT
                                    ):
                                        color = GREEN
                                    elif coverage < MIN_COVERAGE_PERCENT:
                                        color = YELLOW
                                    else:  # coverage > MAX_COVERAGE_PERCENT
                                        color = RED
                                    coverage_str = f"{color}{coverage:.1f}%{RESET}"
                                else:
                                    coverage_str = "N/A"

                                # Calculate target zero position
                                try:
                                    target_zero_deg = target_angle(
                                        config=(config_min_angle, config_max_angle),
                                        tracker=(tracker.min_angle, tracker.max_angle),
                                        angle=0,
                                    )
                                    # Color target zero red if too far from current
                                    distance = abs(
                                        tracker.current_angle - target_zero_deg
                                    )
                                    if distance > MAX_POSITION_ERROR_DEG:
                                        target_zero_str = (
                                            f"{RED}{target_zero_deg:+.2f}°{RESET}"
                                        )
                                    else:
                                        target_zero_str = f"{target_zero_deg:+.2f}°"
                                except ValueError:
                                    target_zero_deg = None
                                    target_zero_str = "N/A"

                                # Determine status message
                                if coverage < MIN_COVERAGE_PERCENT:
                                    status_str = f"{YELLOW}Cover more angles{RESET}"
                                elif (
                                    target_zero_deg is not None
                                    and abs(tracker.current_angle - target_zero_deg)
                                    > MAX_POSITION_ERROR_DEG
                                ):
                                    status_str = f"{RED}Move near zero{RESET}"
                                else:
                                    status_str = f"{GREEN}Ready{RESET}"
                            else:
                                coverage_str = "N/A"
                                target_zero_str = "N/A"
                                status_str = ""

                            table.row(
                                row_idx,
                                [
                                    config.name,
                                    target_zero_str,
                                    current,
                                    min_val,
                                    max_val,
                                    config_min,
                                    config_max,
                                    coverage_str,
                                    status_str,
                                ],
                            )
                        else:
                            table.row(
                                row_idx,
                                [config.name, "", "No state", "", "", "", "", "", ""],
                            )
                    except Exception:  # noqa: BLE001
                        table.row(
                            row_idx, [config.name, "", "Error", "", "", "", "", "", ""]
                        )

            # Render updated table
            display.render()

            # Small delay before refresh
            await asyncio.sleep(0.1)

    except KeyboardInterrupt:
        pass
    finally:
        # Restore terminal settings
        if old_settings is not None and HAS_TERMIOS:
            with contextlib.suppress(OSError, termios.error):
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

        # Disable all motors for safety
        sys.stdout.write("\r\nDisabling all motors...\r\n")
        for motor in motors_list:
            if motor:
                with contextlib.suppress(Exception):
                    await motor.disable()

        sys.stdout.write("Angle tracking stopped.\r\n")


def parse_arguments() -> argparse.Namespace:
    """Parse command-line arguments."""
    parser = argparse.ArgumentParser(
        description="Track min/max angle ranges for Damiao motors"
    )

    parser.add_argument(
        "--channel",
        "-c",
        required=True,
        help="CAN channel name (e.g., can0, can1)",
    )

    parser.add_argument(
        "--interface",
        "-i",
        default="socketcan",
        help="CAN interface type (default: socketcan)",
    )

    parser.add_argument(
        "--side",
        "-s",
        required=True,
        choices=["left", "right"],
        help="Arm side (left or right)",
    )

    return parser.parse_args()


def run() -> None:
    """Entry point for the track_range script."""
    args = parse_arguments()
    try:
        asyncio.run(main(args))
    except KeyboardInterrupt:
        sys.stderr.write("\nInterrupted by user.\n")
        sys.exit(0)
    except Exception as e:  # noqa: BLE001
        sys.stderr.write(f"{RED}Error: {e}{RESET}\n")
        sys.exit(1)


if __name__ == "__main__":
    run()
