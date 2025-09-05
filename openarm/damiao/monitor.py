"""Simple motor angle monitor - disables motors and shows current angles.

This script disables all Damiao motors and displays their current angles.
"""

import argparse
import asyncio
from math import pi
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
    """Main function for the monitor."""
    # Create CAN bus
    can_buses = create_can_bus(args.interface)
    if not can_buses:
        return
    
    # Use only the first bus
    can_bus = can_buses[0]

    # First detect which motors are actually connected
    print("\nScanning for motors...")
    slave_ids = [config.slave_id for config in MOTOR_CONFIGS]

    # Detect motors using raw CAN bus
    detected = list(detect_motors(can_bus, slave_ids, timeout=0.1))

    print("\nMotor Status:")

    # Create lookup for detected motors by slave ID
    detected_lookup = {info.slave_id: info for info in detected}

    # Check all expected motors and their status
    has_error = False
    for config in MOTOR_CONFIGS:
        if config.slave_id not in detected_lookup:
            # Motor is not detected
            print(f"  {RED}✗{RESET} {config.name}: ID 0x{config.slave_id:02X} (Master: 0x{config.master_id:02X}) {RED}[NOT DETECTED]{RESET}")
            has_error = True
        elif detected_lookup[config.slave_id].master_id != config.master_id:
            # Motor is detected but master ID doesn't match
            detected_info = detected_lookup[config.slave_id]
            print(f"  {RED}✗{RESET} {config.name}: ID 0x{config.slave_id:02X} {RED}[MASTER ID MISMATCH: Expected 0x{config.master_id:02X}, Got 0x{detected_info.master_id:02X}]{RESET}")
            has_error = True
        else:
            # Motor is connected and configured correctly
            print(f"  {GREEN}✓{RESET} {config.name}: ID 0x{config.slave_id:02X} (Master: 0x{config.master_id:02X})")

    # Exit if any motors have issues
    if has_error:
        print(f"\n{RED}Error: Some motors have issues. Please check connections and configuration.{RESET}")
        return

    # Create Bus wrapper and Motor instances only if all motors are present
    bus = Bus(can_bus)
    motors: list[Motor] = [
        Motor(
            bus,
            slave_id=config.slave_id,
            master_id=config.master_id,
            motor_type=config.type,
        )
        for config in MOTOR_CONFIGS
    ]

    print(f"\n{GREEN}All {len(motors)} motors detected and initialized{RESET}")

    # Disable all motors
    print("\nDisabling all motors...")
    try:
        state_results = await asyncio.gather(
            *[motor.disable() for motor in motors]
        )
    except Exception as e:
        print(f"{RED}Error disabling motors: {e}{RESET}")
        return
    
    # Start continuous monitoring
    print("\nContinuously monitoring motor angles (Ctrl+C to stop):\n")
    
    # Print initial lines for each motor
    for config in MOTOR_CONFIGS:
        print(f"  {config.name}: Initializing...")
    
    # Number of motors (lines to move up)
    num_motors = len(MOTOR_CONFIGS)
    
    # Use disable results for first display
    current_states = state_results
    
    try:
        while True:
            # Move cursor up to the first motor line
            print(f"\033[{num_motors}A", end="")
            
            # Print current states
            for config, state in zip(MOTOR_CONFIGS, current_states, strict=False):
                if state:
                    angle_deg = state.position * 180 / pi
                    # Clear line and print updated status
                    print(f"\r  {config.name}: {angle_deg:+8.2f}°\033[K")
                else:
                    print(f"\r  {config.name}: No state received\033[K")
            
            # Small delay before refresh
            await asyncio.sleep(0.1)
            
            # Refresh states
            try:
                current_states = await asyncio.gather(
                    *[motor.refresh_status() for motor in motors]
                )
            except Exception as e:
                # Move to bottom and print error
                print(f"\n{RED}Error refreshing: {e}{RESET}", end="")
                # Move back up
                print(f"\033[{num_motors + 1}A", end="")
                await asyncio.sleep(1)  # Wait longer on error
                
    except KeyboardInterrupt:
        # Move cursor below all motor lines
        print(f"\033[{num_motors}B")
        print("\nMonitoring stopped.")


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
        print(f"Error: {e}")
        sys.exit(1)


if __name__ == "__main__":
    run()
