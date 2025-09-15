"""Motor detection utilities for Damiao motors.

This module provides functionality to scan and detect Damiao motors on the CAN bus.
"""

import argparse
import struct
import sys
import time
from collections.abc import Generator, Iterable
from dataclasses import dataclass

import can

from .config import MOTOR_CONFIGS
from .encoding import RegisterAddress, encode_read_register


@dataclass
class MotorInfo:
    """Information about a detected motor."""

    slave_id: int
    master_id: int


def detect_motors(
    bus: can.BusABC,
    slave_ids: Iterable[int],
    timeout: float = 0.05,
) -> Generator[MotorInfo, None, None]:
    """Detect motors by sending register read requests and collecting responses.

    This function sends register read requests to all specified slave IDs
    simultaneously, then collects responses to identify which motors are present.

    Args:
        bus: CAN bus instance for communication
        slave_ids: Iterable of slave IDs to scan
        timeout: Total timeout for detection process in seconds (default: 0.05)

    Yields:
        MotorInfo objects as they are detected

    Example:
        for motor_info in detect_motors(bus, range(1, 11)):
            print(f"Found motor: {motor_info}")

    """
    # Send register read request to all slave IDs at once
    # Read ESC_ID register (0x08) which contains the slave ID
    for slave_id in slave_ids:
        encode_read_register(bus, slave_id, RegisterAddress.ESC_ID)

    # Collect all responses until timeout
    end_time = time.time() + timeout

    while time.time() < end_time:
        remaining_time = end_time - time.time()
        if remaining_time <= 0:
            break

        # Read one message at a time
        message = bus.recv(timeout=remaining_time)

        if message is None:
            # Timeout, no more messages
            break

        if message.is_error_frame:
            continue

        if not message.is_rx:
            continue

        # The arbitration_id IS the master_id!
        master_id = message.arbitration_id

        try:
            # Decode register response to get slave_id from the value
            # Format: '<HBBI' = slave_id(H) + cmd(B) + reg_id(B) + value(I)
            _, _, _, slave_id = struct.unpack("<HBBI", message.data[:8])

            yield MotorInfo(
                slave_id=slave_id,
                master_id=master_id,
            )
        except struct.error:
            # Not a valid register response, ignore
            continue


def main(args: argparse.Namespace) -> None:  # noqa: PLR0912
    """Detect motors on all available CAN buses.

    Args:
        args: Command-line arguments containing timeout value.

    """
    # ANSI color codes for terminal output
    red = "\033[91m"
    green = "\033[92m"
    yellow = "\033[93m"
    reset = "\033[0m"

    # Get available CAN bus configs
    # If no interfaces specified, default to socketcan
    interfaces = args.interface if args.interface else ["socketcan"]
    bus_configs = list(can.detect_available_configs(interfaces=interfaces))

    if not bus_configs:
        print(  # noqa: T201
            f"{red}No CAN buses detected. Please check your CAN configuration.{reset}"
        )
        return

    print(f"\n{green}Detected {len(bus_configs)} CAN bus(es){reset}")  # noqa: T201
    print("-" * 60)  # noqa: T201

    # Get slave IDs from motor configs
    slave_ids = [config.slave_id for config in MOTOR_CONFIGS]

    # Create lookup for motor configs by slave ID
    config_lookup = {config.slave_id: config for config in MOTOR_CONFIGS}

    # Scan each bus
    for bus_config in bus_configs:
        print(  # noqa: T201
            f"\n{yellow}Bus {bus_config['channel']}: Scanning for motors...{reset}"
        )

        # Create bus
        try:
            can_bus = can.Bus(
                channel=bus_config["channel"], interface=bus_config["interface"]
            )
        except Exception as e:  # noqa: BLE001
            print(f"  {red}Error opening bus: {e}{reset}")  # noqa: T201
            continue

        # Scan the bus
        try:
            # Detect motors on this bus
            detected = list(detect_motors(can_bus, slave_ids, timeout=args.timeout))

            if not detected:
                print(f"  {red}No motors detected on this bus{reset}")  # noqa: T201
            else:
                print(f"  Found {len(detected)} motor(s):")  # noqa: T201

            # Create lookup for detected motors by slave ID
            detected_lookup = {info.slave_id: info for info in detected}

            # Check each expected motor
            for config in MOTOR_CONFIGS:
                if config.slave_id in detected_lookup:
                    info = detected_lookup[config.slave_id]
                    # Check if master ID matches
                    if info.master_id == config.master_id:
                        print(  # noqa: T201
                            f"  {green}✓{reset} {config.name}: "
                            f"Slave ID 0x{info.slave_id:02X}, "
                            f"Master ID 0x{info.master_id:02X}"
                        )
                    else:
                        print(  # noqa: T201
                            f"  {yellow}⚠{reset} {config.name}: "
                            f"Slave ID 0x{info.slave_id:02X}, "
                            f"Master ID 0x{info.master_id:02X} "
                            f"{yellow}(Expected Master: "
                            f"0x{config.master_id:02X}){reset}"
                        )
                else:
                    print(  # noqa: T201
                        f"  {red}✗{reset} {config.name}: "
                        f"Slave ID 0x{config.slave_id:02X} "
                        f"{red}[NOT DETECTED]{reset}"
                    )

            # Show any unknown motors (not in our config)
            for info in detected:
                if info.slave_id not in config_lookup:
                    print(  # noqa: T201
                        f"  {yellow}?{reset} Unknown motor: "
                        f"Slave ID 0x{info.slave_id:02X}, "
                        f"Master ID 0x{info.master_id:02X}"
                    )
        finally:
            can_bus.shutdown()


def parse_arguments() -> argparse.Namespace:
    """Parse command-line arguments."""
    parser = argparse.ArgumentParser(
        description="Detect Damiao motors on all available CAN buses"
    )

    parser.add_argument(
        "--interface",
        "-i",
        type=str,
        nargs="*",
        help="CAN interface type(s) to scan (default: socketcan if not specified)",
    )

    parser.add_argument(
        "--timeout",
        "-t",
        type=float,
        default=0.1,
        help="Timeout for motor detection in seconds (default: 0.1)",
    )

    return parser.parse_args()


def run() -> None:
    """Entry point for the motor detection script."""
    args = parse_arguments()

    try:
        main(args)
    except KeyboardInterrupt:
        print("\nInterrupted by user.")  # noqa: T201
        sys.exit(0)
    except Exception as e:  # noqa: BLE001
        print(f"Error: {e}")  # noqa: T201
        sys.exit(1)


if __name__ == "__main__":
    run()
