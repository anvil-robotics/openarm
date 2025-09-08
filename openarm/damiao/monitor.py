"""Simple motor angle monitor - disables motors and shows current angles.

This script disables all Damiao motors and displays their current angles.
"""

import argparse
import asyncio
from math import pi
import sys

import can

from openarm.bus import Bus

from . import ControlMode, MitControlParams, Motor, PosVelControlParams
from .can_buses import create_can_bus
from .config import MOTOR_CONFIGS
from .detect import detect_motors

# ANSI color codes for terminal output
RED = "\033[91m"
GREEN = "\033[92m"
RESET = "\033[0m"


async def main(args: argparse.Namespace) -> None:
    """Main function for the monitor."""
    # Create CAN buses
    can_buses = create_can_bus(args.interface)
    if not can_buses:
        return
    
    print(f"\nDetected {len(can_buses)} CAN bus(es)")

    try:
        return await _main(args, can_buses)
    finally:
        for bus in can_buses:
            bus.shutdown()

async def _main(args: argparse.Namespace, can_buses: list[can.BusABC]):
    # Detect motors on each bus
    all_bus_motors = []
    has_missing_motor = False

    for bus_idx, can_bus in enumerate(can_buses):
        print(f"\nScanning for motors on bus {bus_idx + 1}...")
        slave_ids = [config.slave_id for config in MOTOR_CONFIGS]
        
        # Detect motors using raw CAN bus
        detected = list(detect_motors(can_bus, slave_ids, timeout=0.1))
        
        print(f"\nBus {bus_idx + 1} Motor Status:")
        
        # Create lookup for detected motors by slave ID
        detected_lookup = {info.slave_id: info for info in detected}
        
        # Check all expected motors and their status
        bus_motors = []
        for config in MOTOR_CONFIGS:
            if config.slave_id not in detected_lookup:
                # Motor is not detected
                print(f"  {RED}✗{RESET} {config.name}: ID 0x{config.slave_id:02X} (Master: 0x{config.master_id:02X}) {RED}[NOT DETECTED]{RESET}")
                bus_motors.append(None)
                has_missing_motor = True
            elif detected_lookup[config.slave_id].master_id != config.master_id:
                # Motor is detected but master ID doesn't match
                detected_info = detected_lookup[config.slave_id]
                print(f"  {RED}✗{RESET} {config.name}: ID 0x{config.slave_id:02X} {RED}[MASTER ID MISMATCH: Expected 0x{config.master_id:02X}, Got 0x{detected_info.master_id:02X}]{RESET}")
                bus_motors.append(None)
                has_missing_motor = True
            else:
                # Motor is connected and configured correctly
                print(f"  {GREEN}✓{RESET} {config.name}: ID 0x{config.slave_id:02X} (Master: 0x{config.master_id:02X})")
                # Create motor instance
                bus = Bus(can_bus)
                motor = Motor(
                    bus,
                    slave_id=config.slave_id,
                    master_id=config.master_id,
                    motor_type=config.type,
                )
                bus_motors.append(motor)
        
        all_bus_motors.append(bus_motors)
    
    # Exit if any motor is missing
    if has_missing_motor:
        print(f"\n{RED}Error: Not all motors are detected or configured correctly. Exiting.{RESET}")
        return
    
    # Count total detected motors
    total_motors = sum(1 for bus_motors in all_bus_motors for m in bus_motors if m is not None)
    if total_motors == 0:
        print(f"\n{RED}Error: No motors detected on any bus.{RESET}")
        return
    
    print(f"\n{GREEN}Total {total_motors} motors detected across {len(can_buses)} bus(es){RESET}")
    
    # Disable all motors on all buses
    print("\nDisabling all motors...")
    all_state_results = []
    for bus_idx, bus_motors in enumerate(all_bus_motors):
        bus_states = []
        for motor in bus_motors:
            if motor:
                try:
                    state = await motor.disable()
                    bus_states.append(state)
                except Exception as e:
                    print(f"{RED}Error disabling motor on bus {bus_idx + 1}: {e}{RESET}")
                    bus_states.append(None)
            else:
                bus_states.append(None)
        all_state_results.append(bus_states)
    
    # Call teleop or monitor based on flag
    if args.teleop:
        await teleop(can_buses, all_bus_motors, all_state_results, args)
    else:
        await monitor_motors(can_buses, all_bus_motors, all_state_results, args)

async def monitor_motors(
    can_buses: list[can.BusABC],
    all_bus_motors: list[list[Motor | None]],
    all_state_results: list[list],
    args: argparse.Namespace,
) -> None:

    # Start continuous monitoring with column display
    print(f"\nContinuously monitoring motor angles (Ctrl+C to stop):\n")
    
    # Print header with bus labels
    header = "  Motor"
    for bus_idx in range(len(can_buses)):
        header += f"        Bus {bus_idx + 1}     "
    print(header)
    print("  " + "-" * (len(header) - 2))
    
    # Print initial lines for each motor
    for config in MOTOR_CONFIGS:
        line = f"  {config.name:<12}"
        for _ in range(len(can_buses)):
            line += "  Initializing...  "
        print(line)
    
    # Number of motors (lines to move up)
    num_motors = len(MOTOR_CONFIGS)
    
    # Use disable results for first display
    all_current_states = all_state_results
    
    try:
        while True:
            # Move cursor up to the first motor line
            print(f"\033[{num_motors}A", end="")
            
            # Print current states for all buses
            for motor_idx, config in enumerate(MOTOR_CONFIGS):
                line = f"\r  {config.name:<12}"
                for bus_idx in range(len(can_buses)):
                    state = all_current_states[bus_idx][motor_idx]
                    if state:
                        # Show absolute angle
                        angle_deg = state.position * 180 / pi
                        line += f"  {angle_deg:+8.2f}°     "
                    elif all_bus_motors[bus_idx][motor_idx] is None:
                        line += "       N/A        "
                    else:
                        line += "    No state      "
                print(line + "\033[K")
            
            # Small delay before refresh
            await asyncio.sleep(0.1)
            
            # Refresh states for all buses
            new_all_states = []
            for bus_idx, bus_motors in enumerate(all_bus_motors):
                bus_states = []
                for motor in bus_motors:
                    if motor:
                        try:
                            state = await motor.refresh_status()
                            bus_states.append(state)
                        except Exception:
                            bus_states.append(None)
                    else:
                        bus_states.append(None)
                new_all_states.append(bus_states)
            all_current_states = new_all_states
                
    except KeyboardInterrupt:
        # Move cursor below all motor lines
        print(f"\033[{num_motors}B")
        print("\nMonitoring stopped.")


async def teleop(
    can_buses: list[can.BusABC],
    all_bus_motors: list[list[Motor | None]],
    all_state_results: list[list],
    args: argparse.Namespace,
) -> None:
    """Teleoperation mode - enables motors with MIT control."""
    
    # Determine control mode
    control_mode = ControlMode.POS_VEL if args.control == "posvel" else ControlMode.MIT
    control_mode_str = "Position-Velocity" if args.control == "posvel" else "MIT"
    
    # Parse master-slave relationships from --follow arguments
    slave_to_master = {}  # Maps slave channel to master channel
    slave_mirror_mode = {}  # Maps slave channel to mirror mode (True/False)
    channel_to_idx = {}    # Maps channel name to bus index
    
    # Build channel to index mapping
    for bus_idx, can_bus in enumerate(can_buses):
        # Get channel info from the bus
        channel_info = str(can_bus.channel_info) if hasattr(can_bus, 'channel_info') else str(can_bus.channel)
        # Extract channel name (e.g., "can0" from various formats)
        if 'channel' in channel_info:
            # For socketcan: extract from "SocketcanBus channel 'can0'"
            import re
            match = re.search(r"channel ['\"]?(\w+)", channel_info)
            if match:
                channel_name = match.group(1)
            else:
                channel_name = f"bus{bus_idx}"
        else:
            # For USB devices, use the product name or bus index
            channel_name = channel_info.split()[-1] if channel_info else f"bus{bus_idx}"
        
        channel_to_idx[channel_name] = bus_idx
        print(f"Bus {bus_idx + 1}: Channel '{channel_name}'")
    
    # Parse --follow arguments if provided
    if args.follow:
        for follow_spec in args.follow:
            try:
                parts = follow_spec.split(':')
                if len(parts) == 2:
                    # Standard follow: MASTER:SLAVE
                    master_ch, slave_ch = parts
                    is_mirror = False
                elif len(parts) == 3 and parts[2] == 'mirror':
                    # Mirror follow: MASTER:SLAVE:mirror
                    master_ch, slave_ch = parts[0], parts[1]
                    is_mirror = True
                else:
                    raise ValueError(f"Invalid format: {follow_spec}")
                
                # Validate channels exist
                if master_ch not in channel_to_idx:
                    print(f"{RED}Error: Master channel '{master_ch}' not found{RESET}")
                    return
                if slave_ch not in channel_to_idx:
                    print(f"{RED}Error: Slave channel '{slave_ch}' not found{RESET}")
                    return
                
                # Check for conflicts
                if slave_ch in slave_to_master:
                    print(f"{RED}Error: Slave '{slave_ch}' already follows '{slave_to_master[slave_ch]}'{RESET}")
                    return
                
                slave_to_master[slave_ch] = master_ch
                slave_mirror_mode[slave_ch] = is_mirror
                
            except ValueError:
                print(f"{RED}Error: Invalid follow format '{follow_spec}'. Use MASTER:SLAVE or MASTER:SLAVE:mirror{RESET}")
                return
        
        # Validate no channel is both master and slave
        masters = set(slave_to_master.values())
        slaves = set(slave_to_master.keys())
        overlap = masters & slaves
        if overlap:
            print(f"{RED}Error: Channel(s) {overlap} cannot be both master and slave{RESET}")
            return
    else:
        # Default behavior: first bus is master, others are slaves
        channels = list(channel_to_idx.keys())
        if len(channels) > 1:
            master_ch = channels[0]
            for slave_ch in channels[1:]:
                slave_to_master[slave_ch] = master_ch
                slave_mirror_mode[slave_ch] = False  # Default: no mirror
    
    # Determine master and slave buses
    slave_channels = set(slave_to_master.keys())
    master_channels = set(slave_to_master.values())
    
    print(f"\nMaster-Slave Configuration:")
    for master in sorted(master_channels):
        slaves = []
        for s, m in slave_to_master.items():
            if m == master:
                slave_str = f"{s}(mirror)" if slave_mirror_mode.get(s, False) else s
                slaves.append(slave_str)
        print(f"  Master: {master} -> Slaves: {', '.join(slaves)}")
    
    # Reverse mapping to get channel from index
    idx_to_channel = {v: k for k, v in channel_to_idx.items()}
    
    # Enable motors on slave buses only
    print(f"\nEnabling motors for teleoperation with {control_mode_str} control...")
    for bus_idx, bus_motors in enumerate(all_bus_motors):
        channel = idx_to_channel.get(bus_idx, f"bus{bus_idx}")
        
        if channel not in slave_channels:
            print(f"  Bus {bus_idx + 1} ({channel}): Keeping motors disabled (master)")
            continue
        
        print(f"  Bus {bus_idx + 1} ({channel}): Enabling motors with {control_mode_str} control")
        for motor_idx, motor in enumerate(bus_motors):
            if motor:
                try:
                    await motor.enable()
                    await motor.set_control_mode(control_mode)
                    print(f"    Motor {motor_idx + 1}: Enabled")
                except Exception as e:
                    print(f"{RED}    Motor {motor_idx + 1}: Error - {e}{RESET}")
    
    # Start teleoperation with monitoring display
    print(f"\nTeleoperation mode (Ctrl+C to stop):\n")
    
    # Print header with bus labels showing channel names and roles
    header = "  Motor"
    for bus_idx in range(len(can_buses)):
        channel = idx_to_channel.get(bus_idx, f"bus{bus_idx}")
        if channel in slave_channels:
            # Slave: show S* for mirror mode, S for normal
            role = "S*" if slave_mirror_mode.get(channel, False) else "S"
        else:
            # Master
            role = "M"
        header += f"   {channel}({role})   "
    print(header)
    print("  " + "-" * (len(header) - 2))
    
    # Print initial lines for each motor
    for config in MOTOR_CONFIGS:
        line = f"  {config.name:<12}"
        for _ in range(len(can_buses)):
            line += "  Initializing...  "
        print(line)
    
    # Number of motors (lines to move up)
    num_motors = len(MOTOR_CONFIGS)
    
    # Use disable results for first display
    all_current_states = all_state_results
    
    try:
        while True:
            # Move cursor up to the first motor line
            print(f"\033[{num_motors}A", end="")
            
            # Print current states for all buses
            for motor_idx, config in enumerate(MOTOR_CONFIGS):
                line = f"\r  {config.name:<12}"
                for bus_idx in range(len(can_buses)):
                    state = all_current_states[bus_idx][motor_idx]
                    if state:
                        # Show absolute angle
                        angle_deg = state.position * 180 / pi
                        line += f"  {angle_deg:+8.2f}°     "
                    elif all_bus_motors[bus_idx][motor_idx] is None:
                        line += "       N/A        "
                    else:
                        line += "    No state      "
                print(line + "\033[K")
            
            # Small delay before refresh
            await asyncio.sleep(0.01)
            
            # Read positions from all master buses
            master_positions = {}  # channel -> states
            new_all_states = []
            
            for bus_idx, bus_motors in enumerate(all_bus_motors):
                channel = idx_to_channel.get(bus_idx, f"bus{bus_idx}")
                
                if channel in master_channels:
                    # This is a master bus - read positions
                    bus_states = []
                    for motor in bus_motors:
                        if motor:
                            try:
                                state = await motor.refresh_status()
                                bus_states.append(state)
                            except Exception:
                                bus_states.append(None)
                        else:
                            bus_states.append(None)
                    master_positions[channel] = bus_states
                    new_all_states.append(bus_states)
                    
                elif channel in slave_channels:
                    # This is a slave bus - follow its master
                    master_channel = slave_to_master[channel]
                    master_states = master_positions.get(master_channel)
                    
                    if master_states is None:
                        # Master hasn't been read yet or doesn't exist
                        new_all_states.append([None] * len(bus_motors))
                        continue
                    
                    bus_states = []
                    for motor_idx, motor in enumerate(bus_motors):
                        if motor and motor_idx < len(master_states) and master_states[motor_idx]:
                            try:
                                # Get master position
                                master_pos = master_states[motor_idx].position
                                
                                # Apply mirror if enabled for this slave AND motor supports it
                                if slave_mirror_mode.get(channel, False) and MOTOR_CONFIGS[motor_idx].mirror:
                                    master_pos = -master_pos
                                
                                # Send control command based on mode
                                if args.control == "posvel":
                                    # Position-Velocity control
                                    params = PosVelControlParams(
                                        position=master_pos,
                                        velocity=1
                                    )
                                    state = await motor.control_pos_vel(params)
                                else:
                                    # MIT control
                                    params = MitControlParams(
                                        q=master_pos,     # Desired position in radians
                                        dq=0,            # Desired velocity
                                        kp=10.0,         # Position gain
                                        kd=10,           # Damping gain
                                        tau=0            # Torque feedforward
                                    )
                                    state = await motor.control_mit(params)
                                bus_states.append(state)
                            except Exception:
                                bus_states.append(None)
                        else:
                            bus_states.append(None)
                    new_all_states.append(bus_states)
                else:
                    # Shouldn't happen, but handle gracefully
                    new_all_states.append([None] * len(bus_motors))
            
            all_current_states = new_all_states
                
    except KeyboardInterrupt:
        # Move cursor below all motor lines
        print(f"\033[{num_motors}B")
        print("\nTeleoperation stopped.")
        
        # Disable all motors on slave buses
        print("\nDisabling slave motors...")
        for bus_idx, bus_motors in enumerate(all_bus_motors):
            channel = idx_to_channel.get(bus_idx, f"bus{bus_idx}")
            if channel in slave_channels:
                for motor in bus_motors:
                    if motor:
                        try:
                            await motor.disable()
                        except Exception:
                            pass

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
    
    parser.add_argument(
        "--teleop",
        "-t",
        action="store_true",
        default=False,
        help="Enable teleoperation mode (enables motors with control, except first bus)",
    )
    
    parser.add_argument(
        "--control",
        "-c",
        choices=["posvel", "mit"],
        default="posvel",
        help="Control mode for teleoperation: posvel (position-velocity, default) or mit",
    )
    
    parser.add_argument(
        "--follow",
        action="append",
        help="Define follower mappings as MASTER:SLAVE or MASTER:SLAVE:mirror (e.g., --follow can0:can1 --follow can0:can2:mirror)",
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
