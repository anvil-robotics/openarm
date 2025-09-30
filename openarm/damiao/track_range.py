"""Track min/max angle ranges for Damiao motors.

This script disables all Damiao motors and continuously displays their current angles
along with the minimum and maximum angles observed during the session.
"""

import asyncio
import sys
from dataclasses import dataclass, field
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


@dataclass
class BusMotors:
    """Motors and tracking data for a single CAN bus."""

    bus_idx: int
    can_bus: can.BusABC
    motors: list[Motor | None] = field(default_factory=list)
    trackers: list[AngleTracker | None] = field(default_factory=list)


async def main() -> None:
    """Run the angle range tracker."""
    # Create CAN buses
    try:
        can_buses = [
            can.Bus(channel=config["channel"], interface=config["interface"])
            for config in can.detect_available_configs("socketcan")
        ]
    except Exception:  # noqa: BLE001
        can_buses = []

    if not can_buses:
        sys.stderr.write(f"{RED}Error: No CAN buses detected.{RESET}\n")
        return None

    sys.stdout.write(f"\n{GREEN}Detected {len(can_buses)} CAN bus(es){RESET}\n")

    try:
        return await _main(can_buses)
    finally:
        for bus in can_buses:
            bus.shutdown()


async def _main(can_buses: list[can.BusABC]) -> None:  # noqa: C901
    """Process motors on all buses and track angle ranges."""
    # Detect motors on each bus
    all_bus_motors: list[BusMotors] = []
    has_missing_motor = False

    for bus_idx, can_bus in enumerate(can_buses):
        sys.stdout.write(f"\n{CYAN}Scanning for motors on bus {bus_idx + 1}...{RESET}\n")
        slave_ids = [config.slave_id for config in MOTOR_CONFIGS]

        # Detect motors using raw CAN bus
        detected = list(detect_motors(can_bus, slave_ids, timeout=0.1))

        sys.stdout.write(f"\nBus {bus_idx + 1} Motor Status:\n")

        # Create lookup for detected motors by slave ID
        detected_lookup = {info.slave_id: info for info in detected}

        # Check all expected motors and their status
        bus_motors_list = []
        trackers_list = []

        for config in MOTOR_CONFIGS:
            if config.slave_id not in detected_lookup:
                # Motor is not detected
                sys.stderr.write(
                    f"  {RED}✗{RESET} {config.name}: ID 0x{config.slave_id:02X} "
                    f"(Master: 0x{config.master_id:02X}) {RED}[NOT DETECTED]{RESET}\n"
                )
                bus_motors_list.append(None)
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
                bus_motors_list.append(None)
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
                bus_motors_list.append(motor)
                trackers_list.append(AngleTracker())

        bus_motors = BusMotors(
            bus_idx=bus_idx,
            can_bus=can_bus,
            motors=bus_motors_list,
            trackers=trackers_list,
        )
        all_bus_motors.append(bus_motors)

    # Exit if any motor is missing
    if has_missing_motor:
        sys.stderr.write(
            f"\n{RED}Error: Not all motors are detected or configured "
            f"correctly. Exiting.{RESET}\n"
        )
        return

    # Count total detected motors
    total_motors = sum(
        1 for bm in all_bus_motors for m in bm.motors if m is not None
    )
    if total_motors == 0:
        sys.stderr.write(f"\n{RED}Error: No motors detected on any bus.{RESET}\n")
        return

    sys.stdout.write(
        f"\n{GREEN}Total {total_motors} motors detected across "
        f"{len(can_buses)} bus(es){RESET}\n"
    )

    # Disable all motors
    sys.stdout.write("\nDisabling all motors...\n")
    for bus_motors in all_bus_motors:
        for motor in bus_motors.motors:
            if motor:
                try:
                    await motor.disable()
                except Exception as e:  # noqa: BLE001
                    sys.stderr.write(
                        f"{RED}Error disabling motor on bus "
                        f"{bus_motors.bus_idx + 1}: {e}{RESET}\n"
                    )

    # Start angle tracking
    await track_angles(all_bus_motors)


async def track_angles(all_bus_motors: list[BusMotors]) -> None:  # noqa: C901
    """Track angle ranges for all motors continuously."""
    sys.stdout.write(
        f"\n{GREEN}Tracking angle ranges (Press 'Q' or Ctrl+C to quit){RESET}\n\n"
    )

    # Print header
    header = "  Motor        Bus"
    header += "     Current      Min         Max       Config Range       Coverage"
    sys.stdout.write(header + "\n")
    sys.stdout.write("  " + "-" * (len(header) - 2) + "\n")

    # Print initial lines for each motor
    for config in MOTOR_CONFIGS:
        line = f"  {config.name:<8}"
        for _ in all_bus_motors:
            line += "     Initializing..."
        sys.stdout.write(line + "\n")

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
                line = f"\r  {config.name:<8}"

                for bus_idx, bus_motors in enumerate(all_bus_motors):
                    motor = bus_motors.motors[motor_idx]
                    tracker = bus_motors.trackers[motor_idx]

                    if motor is None:
                        line += f"  {bus_idx + 1}       N/A                             "
                    elif tracker is None:
                        line += f"  {bus_idx + 1}    No tracker                       "
                    else:
                        try:
                            # Refresh motor status
                            state = await motor.refresh_status()
                            if state:
                                # Update tracker with current angle in degrees
                                angle_deg = state.position * 180 / pi
                                tracker.update(angle_deg)

                                # Format display values
                                current = f"{tracker.current_angle:+8.2f}°"
                                min_val = (
                                    f"{tracker.min_angle:+8.2f}°"
                                    if tracker.min_angle != inf
                                    else "     N/A  "
                                )
                                max_val = (
                                    f"{tracker.max_angle:+8.2f}°"
                                    if tracker.max_angle != -inf
                                    else "     N/A  "
                                )
                                config_range = (
                                    f"[{config.min_angle:+.0f}° to {config.max_angle:+.0f}°]"
                                )

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
                                    f"  {bus_idx + 1}  {current}  {min_val}  "
                                    f"{max_val}  {config_range}  {coverage_str}"
                                )
                            else:
                                line += f"  {bus_idx + 1}  No state                         "
                        except Exception:  # noqa: BLE001
                            line += f"  {bus_idx + 1}  Error reading                    "

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
        sys.stdout.write("\n" + "=" * 70 + "\n")
        sys.stdout.write(f"{GREEN}Final Angle Ranges:{RESET}\n")
        sys.stdout.write("=" * 70 + "\n\n")

        # Print summary for each bus
        for bus_motors in all_bus_motors:
            sys.stdout.write(f"{CYAN}Bus {bus_motors.bus_idx + 1}:{RESET}\n")
            for motor_idx, config in enumerate(MOTOR_CONFIGS):
                motor = bus_motors.motors[motor_idx]
                tracker = bus_motors.trackers[motor_idx]

                if motor and tracker:
                    if tracker.min_angle != inf and tracker.max_angle != -inf:
                        range_span = tracker.max_angle - tracker.min_angle
                        config_span = config.max_angle - config.min_angle
                        coverage = (range_span / config_span * 100) if config_span > 0 else 0
                        sys.stdout.write(
                            f"  {config.name:<8}: {tracker.min_angle:+8.2f}° to "
                            f"{tracker.max_angle:+8.2f}°  (span: {range_span:7.2f}°)  "
                            f"Config: [{config.min_angle:+.0f}° to {config.max_angle:+.0f}°]  "
                            f"Coverage: {coverage:5.1f}%\n"
                        )
                    else:
                        sys.stdout.write(f"  {config.name:<8}: No data collected\n")
            sys.stdout.write("\n")

        sys.stdout.write("Angle tracking stopped.\n")


def run() -> None:
    """Entry point for the track_range script."""
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        sys.stderr.write("\nInterrupted by user.\n")
        sys.exit(0)
    except Exception as e:
        sys.stderr.write(f"{RED}Error: {e}{RESET}\n")
        sys.exit(1)


if __name__ == "__main__":
    run()