"""Damiao motor configuration utility.

This script allows configuring Damiao motors by setting master/slave IDs,
setting zero position, and saving parameters to flash memory.
"""

import argparse
import asyncio
import sys

import can

from openarm.bus import Bus

from .config import MOTOR_CONFIGS
from .detect import detect_motors
from .motor import Motor, MotorType

# ANSI color codes for terminal output
RED = "\033[91m"
GREEN = "\033[92m"
YELLOW = "\033[93m"
RESET = "\033[0m"


def parse_arguments() -> argparse.Namespace:
    """Parse command-line arguments."""
    parser = argparse.ArgumentParser(
        description="Configure Damiao motors - set IDs, zero position, and save"
    )

    parser.add_argument(
        "--interface",
        type=str,
        default="socketcan",
        help="CAN interface type (default: socketcan)",
    )

    parser.add_argument(
        "--channel",
        type=str,
        required=True,
        help="CAN channel (required)",
    )

    parser.add_argument(
        "--set-master",
        type=lambda x: int(x, 0),
        help="Set master ID to specified value (hex or decimal)",
    )

    parser.add_argument(
        "--set-slave",
        type=lambda x: int(x, 0),
        help="Set slave ID to specified value (hex or decimal)",
    )

    parser.add_argument(
        "--set-motor",
        type=str,
        choices=["J1", "J2", "J3", "J4", "J5", "J6", "J7", "J8"],
        help="Configure motor with predefined IDs from config (J1-J8)",
    )

    parser.add_argument(
        "--set-zero",
        action="store_true",
        help="Set zero position for configured motors",
    )

    parser.add_argument(
        "--allow-multiple",
        action="store_true",
        help="Allow operation on multiple motors (default: false)",
    )

    parser.add_argument(
        "--save",
        action="store_true",
        help="Save parameters to motor flash memory",
    )

    return parser.parse_args()


async def main() -> None:
    """Run main configuration process."""
    args = parse_arguments()

    # Create CAN bus
    try:
        can_bus = can.Bus(channel=args.channel, interface=args.interface)
    except OSError as e:
        sys.stderr.write(f"{RED}Error opening CAN bus: {e}{RESET}\n")
        sys.exit(1)

    try:
        return await _main(can_bus, args)
    finally:
        can_bus.shutdown()


async def _main(can_bus: can.BusABC, args: argparse.Namespace) -> None:
    """Process motor configuration."""
    sys.stdout.write(f"\n{GREEN}Damiao Motor Configuration Tool{RESET}\n")
    sys.stdout.write("=" * 40 + "\n")

    # Detect motors
    sys.stdout.write(f"Scanning for motors on {args.interface}:{args.channel}...\n")
    slave_ids = [config.slave_id for config in MOTOR_CONFIGS]
    detected = list(detect_motors(can_bus, slave_ids, timeout=0.1))

    if not detected:
        sys.stderr.write(f"{RED}No motors detected. Exiting.{RESET}\n")
        sys.exit(1)

    # Check multiple motors constraint
    if len(detected) > 1 and not args.allow_multiple:
        sys.stderr.write(
            f"{RED}Multiple motors detected ({len(detected)}) "
            f"but --allow-multiple not set. Exiting.{RESET}\n"
        )
        sys.exit(1)

    sys.stdout.write(f"\nDetected {len(detected)} motor(s):\n")
    for info in detected:
        sys.stdout.write(
            f"  - Slave ID: 0x{info.slave_id:02X}, Master ID: 0x{info.master_id:02X}\n"
        )

    # Process each detected motor
    for info in detected:
        sys.stdout.write(
            f"\n{YELLOW}Processing motor (Slave: 0x{info.slave_id:02X}, "
            f"Master: 0x{info.master_id:02X}){RESET}\n"
        )

        # Assume motor type without config lookup
        motor_type = MotorType.DM8009

        # Create motor instance
        bus = Bus(can_bus)
        motor = Motor(
            bus,
            slave_id=info.slave_id,
            master_id=info.master_id,
            motor_type=motor_type,
        )

        # Apply configurations
        await process_motor_configuration(motor, args)


async def process_motor_configuration(motor: Motor, args: argparse.Namespace) -> None:  # noqa: C901
    """Process configuration for a single motor."""
    # Get target config from args.set_motor if specified
    target_config = None
    if args.set_motor:
        for config in MOTOR_CONFIGS:
            if config.name == args.set_motor:
                target_config = config
                break

        if not target_config:
            sys.stderr.write(
                f" {RED}✗ Motor config {args.set_motor} not found{RESET}\n"
            )
            return

    # Set master ID
    if args.set_master is not None or args.set_motor:
        master_id = (
            args.set_master if args.set_master is not None else target_config.master_id
        )
        sys.stdout.write(f"  Setting master ID to 0x{master_id:02X}...")
        try:
            result = await motor.set_master_id(master_id)
            sys.stdout.write(f" {GREEN}✓ Set to 0x{result:02X}{RESET}\n")
        except Exception as e:  # noqa: BLE001  # noqa: BLE001
            sys.stderr.write(f" {RED}✗ Failed: {e}{RESET}\n")
            return

    # Set slave ID
    if args.set_slave is not None or args.set_motor:
        slave_id = (
            args.set_slave if args.set_slave is not None else target_config.slave_id
        )
        sys.stdout.write(f"  Setting slave ID to 0x{slave_id:02X}...")
        try:
            result = await motor.set_slave_id(slave_id)
            sys.stdout.write(f" {GREEN}✓ Set to 0x{result:02X}{RESET}\n")
        except Exception as e:  # noqa: BLE001  # noqa: BLE001
            sys.stderr.write(f" {RED}✗ Failed: {e}{RESET}\n")
            return

    # Set zero position
    if args.set_zero:
        sys.stdout.write("  Setting zero position...")
        try:
            # Disable motor first (required for setting zero)
            await motor.disable()
            await motor.set_zero_position()
            sys.stdout.write(f" {GREEN}✓ Zero position set{RESET}\n")
        except Exception as e:  # noqa: BLE001  # noqa: BLE001
            sys.stderr.write(f" {RED}✗ Failed: {e}{RESET}\n")
            return

    # Save parameters
    if args.save:
        sys.stdout.write("  Saving parameters...")
        try:
            result = await motor.save_parameters()
            sys.stdout.write(f" {GREEN}✓ Parameters saved{RESET}\n")
        except Exception as e:  # noqa: BLE001  # noqa: BLE001
            sys.stderr.write(f" {RED}✗ Failed: {e}{RESET}\n")
            return


def run() -> None:
    """Entry point for the configuration script."""
    try:
        asyncio.run(main())
        sys.stdout.write(f"\n{GREEN}Configuration complete!{RESET}\n")
    except KeyboardInterrupt:
        sys.stderr.write(f"\n{YELLOW}Interrupted by user.{RESET}\n")
        sys.exit(0)
    except Exception as e:  # noqa: BLE001
        sys.stderr.write(f"{RED}Error: {e}{RESET}\n")
        sys.exit(1)


if __name__ == "__main__":
    run()
