#!/usr/bin/env python
"""Teleop forward script - forwards motor positions from source to destination CAN buses.

Usage:
    python teleop_forward.py can0:can2 can1:can3
    
This forwards positions from can0->can2 and can1->can3
Source buses are read-only, destination buses are write-only.
"""

import argparse
import asyncio
import sys
from math import pi

import can

from openarm.bus import Bus
from openarm.damiao.config import MOTOR_CONFIGS
from openarm.damiao.encoding import (
    ControlMode,
    MOTOR_LIMITS,
    PosVelControlParams,
    RegisterAddress,
    decode_motor_state,
    decode_register_int,
    encode_control_pos_vel,
    encode_disable_motor,
    encode_enable_motor,
    encode_write_register_int,
)

# ANSI color codes
RED = "\033[91m"
GREEN = "\033[92m"
YELLOW = "\033[93m"
RESET = "\033[0m"


async def setup_destination_motor(bus: Bus, slave_id: int, master_id: int) -> None:
    """Enable a destination motor and set it to POS_VEL mode.
    
    Args:
        bus: Bus object for the motor
        slave_id: Motor slave ID
        master_id: Motor master ID
    """
    # Enable motor
    encode_enable_motor(bus, slave_id)
    await decode_motor_state(bus, master_id, MOTOR_LIMITS[MOTOR_CONFIGS[slave_id-1].type])
    
    # Set control mode to POS_VEL
    encode_write_register_int(bus, slave_id, RegisterAddress.CTRL_MODE, ControlMode.POS_VEL)
    await decode_register_int(bus, master_id)
    
    print(f"    Motor J{slave_id}: Enabled with POS_VEL mode")  # noqa: T201


async def main(args: argparse.Namespace) -> None:
    """Main function for teleop forwarding.
    
    Args:
        args: Command-line arguments with bus pairs
    """
    # Parse bus pairs
    bus_pairs = []
    for pair in args.pairs:
        parts = pair.split(":")
        if len(parts) != 2:
            print(f"{RED}Error: Invalid pair format '{pair}'. Use SOURCE:DEST{RESET}")  # noqa: T201
            sys.exit(1)
        bus_pairs.append((parts[0], parts[1]))
    
    if not bus_pairs:
        print(f"{RED}Error: No bus pairs specified{RESET}")  # noqa: T201
        sys.exit(1)
    
    print(f"\n{GREEN}Teleop Forward Configuration:{RESET}")  # noqa: T201
    for src, dst in bus_pairs:
        print(f"  {src} -> {dst}")  # noqa: T201
    
    # Open CAN buses
    print("\nOpening CAN buses...")  # noqa: T201
    can_buses = {}
    bus_objects = {}
    
    # Get all unique bus names
    all_bus_names = set()
    for src, dst in bus_pairs:
        all_bus_names.add(src)
        all_bus_names.add(dst)
    
    # Open each bus
    for bus_name in all_bus_names:
        try:
            can_bus = can.Bus(channel=bus_name, interface="socketcan")
            can_buses[bus_name] = can_bus
            bus_objects[bus_name] = Bus(can_bus)
            print(f"  {GREEN}✓{RESET} Opened {bus_name}")  # noqa: T201
        except Exception as e:
            print(f"  {RED}✗{RESET} Failed to open {bus_name}: {e}")  # noqa: T201
            # Close already opened buses
            for b in can_buses.values():
                b.shutdown()
            sys.exit(1)
    
    # Setup destination motors
    print("\nEnabling destination motors...")  # noqa: T201
    destination_buses = {dst for _, dst in bus_pairs}
    
    for dst_name in destination_buses:
        print(f"  Bus {dst_name}:")  # noqa: T201
        dst_bus = bus_objects[dst_name]
        
        # Enable all 8 motors on destination bus
        for config in MOTOR_CONFIGS:
            try:
                await setup_destination_motor(dst_bus, config.slave_id, config.master_id)
            except Exception as e:
                print(f"    {RED}Motor J{config.slave_id}: Failed - {e}{RESET}")  # noqa: T201
    
    # Main forwarding loop
    print("\n" + "="*60)  # noqa: T201
    print("Position Forwarding Active (Ctrl+C to stop)")  # noqa: T201
    print("="*60 + "\n")  # noqa: T201
    
    # Print header
    header = "Bus Pair      "
    for config in MOTOR_CONFIGS:
        header += f"   {config.name:>5}"
    print(header)  # noqa: T201
    print("-" * len(header))  # noqa: T201
    
    try:
        while True:
            # Process each bus pair
            for src_name, dst_name in bus_pairs:
                src_bus = bus_objects[src_name]
                dst_bus = bus_objects[dst_name]
                
                line = f"{src_name}->{dst_name}   "
                
                # Process each motor
                for config in MOTOR_CONFIGS:
                    try:
                        # Read from source
                        src_state = await decode_motor_state(
                            src_bus, 
                            config.master_id, 
                            MOTOR_LIMITS[config.type]
                        )
                        
                        # Forward to destination
                        params = PosVelControlParams(
                            position=src_state.position,
                            velocity=1.0  # Default velocity
                        )
                        encode_control_pos_vel(dst_bus, config.slave_id, params)
                        
                        # Read destination state
                        dst_state = await decode_motor_state(
                            dst_bus,
                            config.master_id,
                            MOTOR_LIMITS[config.type]
                        )
                        
                        # Display angle
                        angle_deg = src_state.position * 180 / pi
                        line += f"  {angle_deg:+7.1f}°"
                        
                    except Exception:  # noqa: BLE001
                        line += "      N/A  "
                
                # Clear line and print
                print(f"\r{line}", end="")  # noqa: T201
                
            # Small delay
            await asyncio.sleep(0.01)
            
    except KeyboardInterrupt:
        print("\n\nStopping forwarding...")  # noqa: T201
    
    # Disable all destination motors
    print("\nDisabling destination motors...")  # noqa: T201
    for dst_name in destination_buses:
        dst_bus = bus_objects[dst_name]
        print(f"  Bus {dst_name}:")  # noqa: T201
        
        for config in MOTOR_CONFIGS:
            try:
                encode_disable_motor(dst_bus, config.slave_id)
                await decode_motor_state(dst_bus, config.master_id, MOTOR_LIMITS[config.type])
                print(f"    Motor J{config.slave_id}: Disabled")  # noqa: T201
            except Exception as e:
                print(f"    {RED}Motor J{config.slave_id}: Failed to disable - {e}{RESET}")  # noqa: T201
    
    # Close all buses
    print("\nClosing CAN buses...")  # noqa: T201
    for bus_name, can_bus in can_buses.items():
        can_bus.shutdown()
        print(f"  Closed {bus_name}")  # noqa: T201
    
    print(f"\n{GREEN}Teleop forward stopped successfully{RESET}")  # noqa: T201


def parse_arguments() -> argparse.Namespace:
    """Parse command-line arguments."""
    parser = argparse.ArgumentParser(
        description="Forward motor positions from source to destination CAN buses"
    )
    
    parser.add_argument(
        "pairs",
        nargs="+",
        help="Bus pairs in format SOURCE:DEST (e.g., can0:can2 can1:can3)"
    )
    
    return parser.parse_args()


def run() -> None:
    """Entry point for the script."""
    args = parse_arguments()
    
    try:
        asyncio.run(main(args))
    except KeyboardInterrupt:
        print("\nInterrupted by user.")  # noqa: T201
        sys.exit(0)
    except Exception as e:
        print(f"{RED}Error: {e}{RESET}")  # noqa: T201
        sys.exit(1)


if __name__ == "__main__":
    run()