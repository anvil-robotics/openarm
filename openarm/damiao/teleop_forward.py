#!/usr/bin/env python
"""Teleop forward script - forwards motor positions from source to destination CAN buses.

Usage:
    python teleop_forward.py can0:can2 can1:can3
    
This forwards positions from can0->can2 and can1->can3
Source buses are read-only, destination buses are write-only.
"""

import argparse
import asyncio
import struct
import sys
from math import pi

import can

from openarm.bus import Bus

from .config import MOTOR_CONFIGS
from .encoding import (
    ControlMode,
    MotorLimits,
    MotorState,
    PosVelControlParams,
    RegisterAddress,
    decode_motor_state,
    decode_register_int,
    encode_control_pos_vel,
    encode_disable_motor,
    encode_enable_motor,
    encode_refresh_status,
    encode_write_register_int,
)
from .motor import MOTOR_LIMITS

# ANSI color codes
RED = "\033[91m"
GREEN = "\033[92m"
YELLOW = "\033[93m"
RESET = "\033[0m"


def _uint_to_float(value: int, min_val: float, max_val: float, bits: int) -> float:
    """Convert unsigned integer to float value with scaling.
    
    Args:
        value: Unsigned integer value to convert
        min_val: Minimum value in float range
        max_val: Maximum value in float range
        bits: Number of bits for the unsigned integer
    
    Returns:
        Scaled float value
    """
    # Scale from unsigned integer range to normalized [0, 1]
    scale = (1 << bits) - 1
    normalized = value / scale
    
    # Scale to actual float range
    return normalized * (max_val - min_val) + min_val


async def decode_motor_state_any(
    src_bus: can.BusABC,
    motor_limits: MotorLimits
) -> tuple[int, MotorState | None]:
    """Decode motor state from any message on the bus.
    
    Args:
        src_bus: Source CAN bus to receive from
        motor_limits: Motor physical limits for scaling
    
    Returns:
        Tuple of (master_id, motor_state) where motor_state is None if decode failed
    """
    try:
        # Receive any message from the bus
        message = src_bus.recv(timeout=0.01)
        if message is None:
            return 0, None
        
        master_id = message.arbitration_id
        
        # Check if we have enough data for motor state
        if len(message.data) < 8:
            return master_id, None
        
        # Unpack motor state response data
        byte0, q_uint, vel_h, vel_t, torque_l, t_mos, t_rotor = struct.unpack(
            ">BHBBBBB", message.data[:8]
        )
        
        # Extract slave_id and status
        slave_id = byte0 & 0xF
        status = (byte0 >> 4) & 0xF
        
        # Decode velocity (12 bits)
        dq_uint = (vel_h << 4) | ((vel_t >> 4) & 0xF)
        
        # Decode torque (12 bits)
        tau_uint = ((vel_t & 0xF) << 8) | torque_l
        
        # Get motor limits for scaling
        q_max, dq_max, tau_max = (
            motor_limits.q_max,
            motor_limits.dq_max,
            motor_limits.tau_max,
        )
        
        # Convert to float values
        position = _uint_to_float(q_uint, -q_max, q_max, 16)
        velocity = _uint_to_float(dq_uint, -dq_max, dq_max, 12)
        torque = _uint_to_float(tau_uint, -tau_max, tau_max, 12)
        
        motor_state = MotorState(
            status=status,
            slave_id=slave_id,
            position=position,
            velocity=velocity,
            torque=torque,
            temp_mos=t_mos,
            temp_rotor=t_rotor,
        )
        
        return master_id, motor_state
        
    except Exception:  # noqa: BLE001
        return 0, None


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
            print(f"{RED}Error: Invalid pair format '{pair}'. Use SOURCE:DEST or SOURCE:{RESET}")  # noqa: T201
            sys.exit(1)
        # Allow empty destination for read-only mode
        src = parts[0]
        dst = parts[1] if parts[1] else None
        bus_pairs.append((src, dst))
    
    if not bus_pairs:
        print(f"{RED}Error: No bus pairs specified{RESET}")  # noqa: T201
        sys.exit(1)
    
    print(f"\n{GREEN}Teleop Forward Configuration:{RESET}")  # noqa: T201
    for src, dst in bus_pairs:
        if dst:
            print(f"  {src} -> {dst}")  # noqa: T201
        else:
            print(f"  {src} (read-only)")  # noqa: T201
    
    # Open CAN buses
    print("\nOpening CAN buses...")  # noqa: T201
    can_buses = {}
    bus_objects = {}
    
    # Get all unique bus names
    all_bus_names = set()
    for src, dst in bus_pairs:
        all_bus_names.add(src)
        if dst:  # Only add destination if not None
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
    
    # Setup destination motors (only if there are any)
    destination_buses = {dst for _, dst in bus_pairs if dst is not None}
    
    if destination_buses:
        print("\nEnabling destination motors...")  # noqa: T201
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
    header = "Source/Dest   "
    for config in MOTOR_CONFIGS:
        header += f"   {config.name:>7}"
    print(header)  # noqa: T201
    print("-" * len(header))  # noqa: T201
    
    # Calculate number of lines needed for display
    num_lines = len(bus_pairs) * 2  # Each pair needs 2 lines (source + dest)
    
    # Print initial empty lines for the display
    for _ in range(num_lines):
        print()  # noqa: T201
    
    # Create mapping from master_id to config index
    master_to_config = {}
    for idx, config in enumerate(MOTOR_CONFIGS):
        master_to_config[config.master_id] = idx
    
    try:
        while True:
            # Move cursor up to overwrite previous output
            if num_lines > 0:
                print(f"\033[{num_lines}A", end="")  # noqa: T201
            
            # Process each bus pair
            for src_name, dst_name in bus_pairs:
                src_can = can_buses[src_name]
                dst_can = can_buses[dst_name] if dst_name else None
                dst_bus = bus_objects[dst_name] if dst_name else None
                
                # Store states for display
                src_states = [None] * len(MOTOR_CONFIGS)
                dst_states = [None] * len(MOTOR_CONFIGS)
                
                # Decode any motor state from source
                master_id, src_state = await decode_motor_state_any(
                    src_can,
                    MOTOR_LIMITS[MOTOR_CONFIGS[0].type]  # Temp limits, will fix below
                )
                
                if src_state is not None and master_id in master_to_config:
                    # Get the motor config for this master_id
                    motor_idx = master_to_config[master_id]
                    config = MOTOR_CONFIGS[motor_idx]
                    motor_limits = MOTOR_LIMITS[config.type]
                    
                    # Re-decode with correct limits and forward if needed
                    if dst_can and dst_bus:
                        # Forward position
                        params = PosVelControlParams(
                            position=src_state.position,
                            velocity=args.velocity
                        )
                        encode_control_pos_vel(dst_bus, config.slave_id, params)
                        
                        # Read destination state
                        try:
                            dst_state = await decode_motor_state(
                                dst_bus,
                                config.master_id,
                                motor_limits
                            )
                            dst_states[motor_idx] = dst_state
                        except Exception:  # noqa: BLE001
                            pass
                    
                    src_states[motor_idx] = src_state
                
                # Display source line
                src_line = f"\r{GREEN}{src_name:12}{RESET}  "
                for state in src_states:
                    if state:
                        angle_deg = state.position * 180 / pi
                        src_line += f"  {angle_deg:+7.1f}°"
                    else:
                        src_line += "      N/A  "
                print(src_line + "\033[K")  # noqa: T201
                
                # Display destination line (or read-only indicator)
                if dst_name:
                    dst_line = f"\r{YELLOW}  -> {dst_name:8}{RESET}  "
                    for state in dst_states:
                        if state:
                            angle_deg = state.position * 180 / pi
                            dst_line += f"  {angle_deg:+7.1f}°"
                        else:
                            dst_line += "      N/A  "
                else:
                    # Read-only mode - no destination
                    dst_line = f"\r  (read-only)    " + " " * (len(src_states) * 11)
                print(dst_line + "\033[K")  # noqa: T201
            
            # Send refresh to J8 (motor 8) on all source buses
            source_buses = {src for src, _ in bus_pairs}
            for src_name in source_buses:
                src_bus = bus_objects[src_name]
                # J8 is motor with slave_id 0x08
                encode_refresh_status(src_bus, 0x08)
            
            # Small delay
            # await asyncio.sleep(0.01)
            
    except KeyboardInterrupt:
        print("\n\nStopping forwarding...")  # noqa: T201
    
    # Disable all destination motors (if any)
    if destination_buses:
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
    
    parser.add_argument(
        "--velocity",
        type=float,
        default=1.0,
        help="Velocity for position control (default: 1.0)"
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