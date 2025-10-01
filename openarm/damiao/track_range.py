"""Track min/max angle ranges for Damiao motors.

This script disables all Damiao motors and continuously displays their current angles
along with the minimum and maximum angles observed during the session.
"""

import argparse
import asyncio
import sys
from dataclasses import dataclass
from math import inf, pi

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

from . import Motor
from .config import MOTOR_CONFIGS
from .detect import detect_motors

# ANSI color codes for terminal output
RED = "\033[91m"
GREEN = "\033[92m"
YELLOW = "\033[93m"
CYAN = "\033[96m"
RESET = "\033[0m"


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
        if angle < self.min_angle:
            self.min_angle = angle
        if angle > self.max_angle:
            self.max_angle = angle

    def reset(self) -> None:
        """Reset min/max tracking."""
        self.min_angle = inf
        self.max_angle = -inf




async def main(args: argparse.Namespace) -> None:
    """Run the angle range tracker."""
    # Create single CAN bus from arguments
    try:
        can_bus = can.Bus(channel=args.channel, interface=args.interface)
    except Exception as e:
        sys.stderr.write(f"{RED}Error: Failed to create CAN bus: {e}{RESET}\n")
        return None

    sys.stdout.write(
        f"\n{GREEN}Connected to {args.channel} (interface: {args.interface}){RESET}\n"
    )

    try:
        return await _main(can_bus)
    finally:
        can_bus.shutdown()


async def _main(can_bus: can.BusABC) -> None:  # noqa: C901
    """Process motors on the bus and track angle ranges."""
    # Detect motors on the bus
    sys.stdout.write(f"\n{CYAN}Scanning for motors...{RESET}\n")
    slave_ids = [config.slave_id for config in MOTOR_CONFIGS]

    # Detect motors using raw CAN bus
    detected = list(detect_motors(can_bus, slave_ids, timeout=0.1))

    sys.stdout.write(f"\nMotor Status:\n")

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

    sys.stdout.write(f"\n{GREEN}Total {total_motors} motors detected{RESET}\n")

    # Disable all motors
    sys.stdout.write("\nDisabling all motors...\n")
    for motor in motors_list:
        if motor:
            try:
                await motor.disable()
            except Exception as e:  # noqa: BLE001
                sys.stderr.write(f"{RED}Error disabling motor: {e}{RESET}\n")

    # Start angle tracking
    await track_angles(motors_list, trackers_list)


async def track_angles(
    motors_list: list[Motor | None], trackers_list: list[AngleTracker | None]
) -> None:  # noqa: C901
    """Track angle ranges for all motors continuously."""
    sys.stdout.write(
        f"\n{GREEN}Tracking angle ranges (Press 'Q' or Ctrl+C to quit){RESET}\n\n"
    )

    # Print header
    sys.stdout.write("  Motor      Current       Min         Max        Config Range        Coverage\n")
    sys.stdout.write("  " + "-" * 82 + "\n")

    # Print initial lines for each motor
    for config in MOTOR_CONFIGS:
        sys.stdout.write(f"  {config.name:<8}     Initializing...\n")

    num_motors = len(MOTOR_CONFIGS)

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

            # Move cursor up to the first motor line
            sys.stdout.write(f"\033[{num_motors}A")

            # Update and display each motor's angles
            for motor_idx, config in enumerate(MOTOR_CONFIGS):
                line = f"\r  {config.name:<6}"
                motor = motors_list[motor_idx]
                tracker = trackers_list[motor_idx]

                if motor is None:
                    line += "       N/A                                                      "
                elif tracker is None:
                    line += "    No tracker                                                  "
                else:
                    try:
                        # Refresh motor status
                        state = await motor.refresh_status()
                        if state:
                            # Update tracker with current angle in degrees
                            angle_deg = state.position * 180 / pi
                            tracker.update(angle_deg)

                            # Format display values
                            current = f"{tracker.current_angle:+9.2f}°"
                            min_val = (
                                f"{tracker.min_angle:+9.2f}°"
                                if tracker.min_angle != inf
                                else "      N/A  "
                            )
                            max_val = (
                                f"{tracker.max_angle:+9.2f}°"
                                if tracker.max_angle != -inf
                                else "      N/A  "
                            )
                            config_range = f"[{config.min_angle:+4.0f}° to {config.max_angle:+4.0f}°]"

                            # Calculate coverage percentage
                            if tracker.min_angle != inf and tracker.max_angle != -inf:
                                observed_span = tracker.max_angle - tracker.min_angle
                                config_span = config.max_angle - config.min_angle
                                if config_span > 0:
                                    coverage = (observed_span / config_span) * 100
                                    coverage_str = f"{coverage:6.1f}%"
                                else:
                                    coverage_str = "   N/A "
                            else:
                                coverage_str = "   N/A "

                            line += (
                                f"  {current:>11}  {min_val:>11}  "
                                f"{max_val:>11}  {config_range:>20}  {coverage_str:>9}"
                            )
                        else:
                            line += "  No state                                                "
                    except Exception:  # noqa: BLE001
                        line += "  Error reading                                               "

                sys.stdout.write(line + "\033[K\n")

            # Small delay before refresh
            await asyncio.sleep(0.1)

    except KeyboardInterrupt:
        pass
    finally:
        # Restore terminal settings
        if old_settings is not None and HAS_TERMIOS:
            try:
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
            except (OSError, termios.error):
                pass

        # Move cursor below all motor lines
        sys.stdout.write(f"\033[{num_motors}B\n")
        sys.stdout.write("\nAngle tracking stopped.\n")


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

    return parser.parse_args()


def run() -> None:
    """Entry point for the track_range script."""
    args = parse_arguments()
    try:
        asyncio.run(main(args))
    except KeyboardInterrupt:
        sys.stderr.write("\nInterrupted by user.\n")
        sys.exit(0)
    except Exception as e:
        sys.stderr.write(f"{RED}Error: {e}{RESET}\n")
        sys.exit(1)


if __name__ == "__main__":
    run()