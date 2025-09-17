"""Set zero position for all Damiao motors on all buses."""
# ruff: noqa: BLE001

import argparse
import asyncio
import sys

import can

from openarm.bus import Bus

from . import Motor
from .config import MOTOR_CONFIGS
from .detect import detect_motors

# ANSI color codes for terminal output
RED = "\033[91m"
GREEN = "\033[92m"
YELLOW = "\033[93m"
RESET = "\033[0m"


async def main(args: argparse.Namespace) -> None:
    """Set zero position for all motors."""
    # Create CAN buses
    try:
        can_buses = [
            can.Bus(channel=config["channel"], interface=config["interface"])
            for config in can.detect_available_configs("socketcan")
        ]
    except Exception:
        can_buses = []

    if not can_buses:
        sys.stderr.write(f"{RED}Error: No CAN buses detected.{RESET}\n")
        return None

    sys.stdout.write(f"\n{GREEN}Detected {len(can_buses)} CAN bus(es){RESET}\n")

    try:
        return await _main(args, can_buses)
    finally:
        for bus in can_buses:
            bus.shutdown()


async def _main(args: argparse.Namespace, can_buses: list) -> None:  # noqa: ARG001
    """Process motors on all buses."""
    # Scan and set zero for each bus
    for bus_idx, can_bus in enumerate(can_buses):
        sys.stdout.write(f"\n{'=' * 50}\n")
        sys.stdout.write(f"Bus {bus_idx + 1} of {len(can_buses)}\n")
        sys.stdout.write(f"{'=' * 50}\n")

        sys.stdout.write(f"Scanning for motors on bus {bus_idx + 1}...\n")
        slave_ids = [config.slave_id for config in MOTOR_CONFIGS]

        # Detect motors using raw CAN bus
        detected = list(detect_motors(can_bus, slave_ids, timeout=0.1))

        if not detected:
            sys.stdout.write(
                f"{YELLOW}No motors detected on bus {bus_idx + 1}, skipping...{RESET}\n"
            )
            continue

        sys.stdout.write(f"\nDetected {len(detected)} motor(s) on bus {bus_idx + 1}\n")

        # Create lookup for detected motors by slave ID
        detected_lookup = {info.slave_id: info for info in detected}

        # Process each detected motor
        for config in MOTOR_CONFIGS:
            if config.slave_id not in detected_lookup:
                continue

            detected_info = detected_lookup[config.slave_id]

            # Check if master ID matches expected
            if detected_info.master_id != config.master_id:
                sys.stderr.write(
                    f"\n{YELLOW}⚠ {config.name} (ID 0x{config.slave_id:02X}): "
                    f"Master ID mismatch (expected 0x{config.master_id:02X}, "
                    f"got 0x{detected_info.master_id:02X}){RESET}\n"
                )
                continue

            # Create motor instance
            bus = Bus(can_bus)
            motor = Motor(
                bus,
                slave_id=config.slave_id,
                master_id=config.master_id,
                motor_type=config.type,
            )

            # Process motor - disable and set zero
            sys.stdout.write(f"  {config.name} (ID 0x{config.slave_id:02X}): ")

            # Disable motor first (required for setting zero)
            try:
                await motor.disable()
            except Exception as e:
                sys.stderr.write(f"{RED}✗ Failed to disable: {e}{RESET}\n")
                continue

            # Set zero position
            try:
                await motor.set_zero_position()
                await motor.save_parameters()
                sys.stdout.write(f"{GREEN}✓ Zero set{RESET}\n")
            except Exception as e:
                sys.stderr.write(f"{RED}✗ Failed to set zero: {e}{RESET}\n")
                continue

    sys.stdout.write(f"\n{'=' * 50}\n")
    sys.stdout.write(f"{GREEN}Zero position setting complete for all buses!{RESET}\n")
    sys.stdout.write(f"{'=' * 50}\n\n")


def parse_arguments() -> argparse.Namespace:
    """Parse command-line arguments."""
    parser = argparse.ArgumentParser(
        description="Set zero position for all Damiao motors on all buses"
    )

    parser.add_argument(
        "--interface",
        "-i",
        default="can0",
        help="CAN interface name (default: can0, ignored on Windows/macOS)",
    )

    return parser.parse_args()


def run() -> None:
    """Run the set_zero script."""
    args = parse_arguments()

    try:
        asyncio.run(main(args))
    except KeyboardInterrupt:
        sys.stderr.write(f"\n{YELLOW}Interrupted by user.{RESET}\n")
        sys.exit(0)
    except Exception as e:
        sys.stderr.write(f"{RED}Error: {e}{RESET}\n")
        sys.exit(1)


if __name__ == "__main__":
    run()
