#!/usr/bin/env python
"""Set zero position for all Damiao motors on all buses."""

import argparse
import asyncio
import platform
import sys
import time
from dataclasses import dataclass

import can
import usb.core

from openarm.bus import Bus

from . import Motor, MotorType
from .detect import detect_motors

# ANSI color codes for terminal output
RED = "\033[91m"
GREEN = "\033[92m"
YELLOW = "\033[93m"
RESET = "\033[0m"


@dataclass
class MotorConfig:
    """Configuration for a single motor."""

    name: str
    slave_id: int  # Target slave ID
    master_id: int  # Target master ID
    type: MotorType


# Motor configurations
MOTOR_CONFIGS: list[MotorConfig] = [
    MotorConfig("J1", slave_id=0x01, master_id=0x11, type=MotorType.DM8009),
    MotorConfig("J2", slave_id=0x02, master_id=0x12, type=MotorType.DM8009),
    MotorConfig("J3", slave_id=0x03, master_id=0x13, type=MotorType.DM4340),
    MotorConfig("J4", slave_id=0x04, master_id=0x14, type=MotorType.DM4340),
    MotorConfig("J5", slave_id=0x05, master_id=0x15, type=MotorType.DM4310),
    MotorConfig("J6", slave_id=0x06, master_id=0x16, type=MotorType.DM4310),
    MotorConfig("J7", slave_id=0x07, master_id=0x17, type=MotorType.DM4310),
    MotorConfig("J8", slave_id=0x08, master_id=0x18, type=MotorType.DM4310),
]


def create_can_bus(interface: str = "can0", max_attempts: int = 10) -> list[can.BusABC]:
    """Create and initialize CAN bus based on platform.
    
    Args:
        interface: CAN interface name for Linux (ignored on Windows/macOS)
        max_attempts: Maximum connection attempts for USB devices
        
    Returns:
        List of initialized CAN buses (empty if failed)
    """
    if platform.system() in ["Windows", "Darwin"]:  # Darwin is macOS
        print("Searching for USB CAN devices...")
        devs = usb.core.find(idVendor=0x1D50, idProduct=0x606F, find_all=True)
        if not devs:
            print("Error: No USB CAN devices found (VID:0x1D50, PID:0x606F)")
            return []
        
        devs = list(devs)  # Convert to list
        print(f"Found {len(devs)} USB CAN device(s)")

        can_buses = []
        for dev in devs:
            print(f"Initializing USB CAN device: {dev.product}")
            
            # Retry connection for USB devices
            last_error = None
            for attempt in range(max_attempts):
                try:
                    can_bus = can.Bus(
                        interface="gs_usb",
                        channel=dev.product,
                        bitrate=1000000,
                        bus=dev.bus,
                        address=dev.address
                    )
                    print(f"  CAN bus initialized successfully (attempt {attempt + 1})")
                    can_buses.append(can_bus)
                    break
                except Exception as e:
                    last_error = e
                    if attempt < max_attempts - 1:
                        time.sleep(0.1)
            else:
                print(f"  Failed to initialize CAN bus after {max_attempts} attempts: {last_error}")
        
        return can_buses

    # Linux
    print(f"Initializing CAN bus on {interface}...")
    try:
        return [can.Bus(channel=interface, interface="socketcan")]
    except Exception as e:
        print(f"Failed to initialize socketcan: {e}")
        return []


async def main(args: argparse.Namespace) -> None:
    """Main function to set zero position for all motors."""
    # Create CAN buses
    can_buses = create_can_bus(args.interface)
    if not can_buses:
        print(f"{RED}Error: No CAN buses detected.{RESET}")
        return
    
    print(f"\n{GREEN}Detected {len(can_buses)} CAN bus(es){RESET}")

    # Scan and set zero for each bus
    for bus_idx, can_bus in enumerate(can_buses):
        print(f"\n{'='*50}")
        print(f"Bus {bus_idx + 1} of {len(can_buses)}")
        print(f"{'='*50}")
        
        print(f"Scanning for motors on bus {bus_idx + 1}...")
        slave_ids = [config.slave_id for config in MOTOR_CONFIGS]
        
        # Detect motors using raw CAN bus
        detected = list(detect_motors(can_bus, slave_ids, timeout=0.1))
        
        if not detected:
            print(f"{YELLOW}No motors detected on bus {bus_idx + 1}, skipping...{RESET}")
            continue
        
        print(f"\nDetected {len(detected)} motor(s) on bus {bus_idx + 1}")
        
        # Create lookup for detected motors by slave ID
        detected_lookup = {info.slave_id: info for info in detected}
        
        # Process each detected motor
        for config in MOTOR_CONFIGS:
            if config.slave_id not in detected_lookup:
                continue
                
            detected_info = detected_lookup[config.slave_id]
            
            # Check if master ID matches expected
            if detected_info.master_id != config.master_id:
                print(f"\n{YELLOW}⚠ {config.name} (ID 0x{config.slave_id:02X}): "
                      f"Master ID mismatch (expected 0x{config.master_id:02X}, "
                      f"got 0x{detected_info.master_id:02X}){RESET}")
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
            print(f"  {config.name} (ID 0x{config.slave_id:02X}): ", end="")
            
            # Disable motor first (required for setting zero)
            try:
                await motor.disable()
            except Exception as e:
                print(f"{RED}✗ Failed to disable: {e}{RESET}")
                continue
            
            # Set zero position
            try:
                await motor.set_zero_position()
                print(f"{GREEN}✓ Zero set{RESET}")
            except Exception as e:
                print(f"{RED}✗ Failed to set zero: {e}{RESET}")
                continue
    
    print(f"\n{'='*50}")
    print(f"{GREEN}Zero position setting complete for all buses!{RESET}")
    print(f"{'='*50}\n")


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
    """Entry point for the set_zero script."""
    args = parse_arguments()

    try:
        asyncio.run(main(args))
    except KeyboardInterrupt:
        print(f"\n{YELLOW}Interrupted by user.{RESET}")
        sys.exit(0)
    except Exception as e:
        print(f"{RED}Error: {e}{RESET}")
        sys.exit(1)


if __name__ == "__main__":
    run()