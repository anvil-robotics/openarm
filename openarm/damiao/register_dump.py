"""Dump all register values for detected Damiao motors.

This script detects motors on all CAN buses and reads all their register values,
displaying them in a tabular format for easy comparison across motors.
"""

import argparse
import asyncio
import sys
from collections.abc import Callable, Coroutine
from typing import Any

import can

from openarm.bus import Bus

from . import Motor
from .config import MOTOR_CONFIGS
from .detect import detect_motors
from .encoding import RegisterAddress

# ANSI color codes for terminal output
RED = "\033[91m"
GREEN = "\033[92m"
YELLOW = "\033[93m"
RESET = "\033[0m"


# Map of register addresses to their types and readable names
REGISTER_INFO: dict[RegisterAddress, tuple[str, str]] = {
    # Voltage Protection
    RegisterAddress.UV_VALUE: ("under_voltage", "float"),
    RegisterAddress.OV_VALUE: ("over_voltage", "float"),
    # Motor Characteristics
    RegisterAddress.KT_VALUE: ("torque_coefficient", "float"),
    RegisterAddress.GREF: ("gear_efficiency", "float"),
    # Protection Limits
    RegisterAddress.OT_VALUE: ("over_temperature", "float"),
    RegisterAddress.OC_VALUE: ("over_current", "float"),
    # Mapping Limits
    RegisterAddress.PMAX: ("position_limit", "float"),
    RegisterAddress.VMAX: ("velocity_limit", "float"),
    RegisterAddress.TMAX: ("torque_limit", "float"),
    # Control Loop Parameters
    RegisterAddress.KP_ASR: ("velocity_kp", "float"),
    RegisterAddress.KI_ASR: ("velocity_ki", "float"),
    RegisterAddress.KP_APR: ("position_kp", "float"),
    RegisterAddress.KI_APR: ("position_ki", "float"),
    # Current and Speed Loop Parameters
    RegisterAddress.I_BW: ("current_loop_bandwidth", "float"),
    RegisterAddress.DETA: ("speed_loop_damping", "float"),
    RegisterAddress.V_BW: ("speed_loop_filter_bandwidth", "float"),
    RegisterAddress.IQ_C1: ("current_loop_gain", "float"),
    RegisterAddress.VL_C1: ("speed_loop_gain", "float"),
    # Motor Information (Read-Only)
    RegisterAddress.HW_VER: ("hardware_version", "int"),
    RegisterAddress.SW_VER: ("software_version", "int"),
    RegisterAddress.SN: ("serial_number", "int"),
    RegisterAddress.SUB_VER: ("sub_version", "int"),
    RegisterAddress.GR: ("gear_ratio", "float"),
    RegisterAddress.DAMP: ("motor_damping", "float"),
    RegisterAddress.INERTIA: ("motor_inertia", "float"),
    RegisterAddress.NPP: ("motor_pole_pairs", "int"),
    RegisterAddress.RS: ("motor_phase_resistance", "float"),
    RegisterAddress.LS: ("motor_phase_inductance", "float"),
    RegisterAddress.FLUX: ("motor_flux", "float"),
    # Motion Parameters
    RegisterAddress.ACC: ("acceleration", "float"),
    RegisterAddress.DEC: ("deceleration", "float"),
    RegisterAddress.MAX_SPD: ("max_speed", "float"),
    # Communication Parameters
    RegisterAddress.MST_ID: ("master_id", "int"),
    RegisterAddress.ESC_ID: ("slave_id", "int"),
    RegisterAddress.TIMEOUT: ("timeout", "int"),
    RegisterAddress.CAN_BR: ("can_baudrate", "int"),
    RegisterAddress.CTRL_MODE: ("control_mode", "int"),
    # Calibration Parameters (Read-Only)
    RegisterAddress.U_OFF: ("phase_u_offset", "float"),
    RegisterAddress.V_OFF: ("phase_v_offset", "float"),
    RegisterAddress.K1: ("compensation_factor_1", "float"),
    RegisterAddress.K2: ("compensation_factor_2", "float"),
    RegisterAddress.M_OFF: ("angle_offset", "float"),
    RegisterAddress.DIR: ("direction", "float"),
    # Position Parameters (Read-Only)
    RegisterAddress.P_M: ("motor_position", "float"),
    RegisterAddress.XOUT: ("output_shaft_position", "float"),
}


async def read_register(
    motor: Motor, register: RegisterAddress, reg_type: str
) -> str | None:
    """Read a single register from a motor.

    Args:
        motor: Motor instance to read from
        register: Register address to read
        reg_type: Type of register ("int" or "float")

    Returns:
        String representation of the value, or None if read fails

    """
    try:
        # Import encoding functions to read registers directly
        from .encoding import decode_register_float, decode_register_int, encode_read_register  # noqa: PLC0415

        # Send read request
        encode_read_register(motor.bus, motor.slave_id, register)

        # Decode response based on type
        if reg_type == "int":
            value = await decode_register_int(motor.bus, motor.master_id)
            return str(value)
        else:  # float
            value = await decode_register_float(motor.bus, motor.master_id)
            return f"{value:.4f}"
    except Exception:  # noqa: BLE001
        return None


async def dump_registers_for_bus(
    can_bus: can.BusABC, bus_idx: int, total_buses: int
) -> None:
    """Dump register values for all motors on a single CAN bus.

    Args:
        can_bus: CAN bus instance
        bus_idx: Index of this bus (0-based)
        total_buses: Total number of buses being scanned

    """
    sys.stdout.write(f"\n{'=' * 80}\n")
    sys.stdout.write(f"Bus {bus_idx + 1} of {total_buses}\n")
    sys.stdout.write(f"{'=' * 80}\n")

    # Detect motors on this bus
    sys.stdout.write(f"Scanning for motors on bus {bus_idx + 1}...\n")
    slave_ids = [config.slave_id for config in MOTOR_CONFIGS]
    detected = list(detect_motors(can_bus, slave_ids, timeout=0.1))

    if not detected:
        sys.stdout.write(
            f"{YELLOW}No motors detected on bus {bus_idx + 1}, skipping...{RESET}\n"
        )
        return

    sys.stdout.write(f"\nDetected {len(detected)} motor(s) on bus {bus_idx + 1}\n\n")

    # Create lookup for detected motors
    detected_lookup = {info.slave_id: info for info in detected}

    # Build list of detected motors with their configs
    motors_data: list[tuple[str, Motor]] = []
    for config in MOTOR_CONFIGS:
        if config.slave_id in detected_lookup:
            detected_info = detected_lookup[config.slave_id]
            if detected_info.master_id == config.master_id:
                bus = Bus(can_bus)
                motor = Motor(
                    bus,
                    slave_id=config.slave_id,
                    master_id=config.master_id,
                    motor_type=config.type,
                )
                motors_data.append((config.name, motor))

    if not motors_data:
        sys.stdout.write(
            f"{YELLOW}No motors with matching configuration found{RESET}\n"
        )
        return

    # Read all registers for all motors
    register_values: dict[str, dict[str, str]] = {}  # {register_name: {motor_name: value}}

    for register, (reg_name, reg_type) in REGISTER_INFO.items():
        register_values[reg_name] = {}
        for motor_name, motor in motors_data:
            value = await read_register(motor, register, reg_type)
            if value is not None:
                register_values[reg_name][motor_name] = value
            else:
                register_values[reg_name][motor_name] = "-"

    # Print table
    motor_names = [name for name, _ in motors_data]

    # Calculate column widths
    name_col_width = max(len(reg_name) for reg_name in register_values.keys()) + 2
    motor_col_width = 15

    # Print header
    header = f"{'Register':<{name_col_width}}"
    for motor_name in motor_names:
        header += f"{motor_name:^{motor_col_width}}"
    sys.stdout.write(f"{GREEN}{header}{RESET}\n")
    sys.stdout.write("-" * len(header) + "\n")

    # Print each register row
    for reg_name in register_values:
        row = f"{reg_name:<{name_col_width}}"
        for motor_name in motor_names:
            value = register_values[reg_name].get(motor_name, "-")
            row += f"{value:^{motor_col_width}}"
        sys.stdout.write(f"{row}\n")

    sys.stdout.write("\n")


async def main(args: argparse.Namespace) -> None:
    """Main function to dump registers from all buses.

    Args:
        args: Command-line arguments

    """
    # Get available CAN bus configs
    interfaces = args.interface if args.interface else ["socketcan"]
    bus_configs = list(can.detect_available_configs(interfaces=interfaces))

    if not bus_configs:
        sys.stderr.write(
            f"{RED}No CAN buses detected. Please check your CAN configuration.{RESET}\n"
        )
        return

    sys.stdout.write(f"\n{GREEN}Detected {len(bus_configs)} CAN bus(es){RESET}\n")

    # Process each bus
    for bus_idx, bus_config in enumerate(bus_configs):
        try:
            can_bus = can.Bus(
                channel=bus_config["channel"], interface=bus_config["interface"]
            )
        except Exception as e:  # noqa: BLE001
            sys.stderr.write(
                f"{RED}Error opening bus {bus_config['channel']}: {e}{RESET}\n"
            )
            continue

        try:
            await dump_registers_for_bus(can_bus, bus_idx, len(bus_configs))
        finally:
            can_bus.shutdown()

    sys.stdout.write(f"\n{GREEN}Register dump complete!{RESET}\n\n")


def parse_arguments() -> argparse.Namespace:
    """Parse command-line arguments."""
    parser = argparse.ArgumentParser(
        description="Dump all register values for detected Damiao motors"
    )

    parser.add_argument(
        "--interface",
        "-i",
        type=str,
        nargs="*",
        help="CAN interface type(s) to scan (default: socketcan)",
    )

    return parser.parse_args()


def run() -> None:
    """Entry point for the register dump script."""
    args = parse_arguments()

    try:
        asyncio.run(main(args))
    except KeyboardInterrupt:
        sys.stderr.write(f"\n{YELLOW}Interrupted by user.{RESET}\n")
        sys.exit(0)
    except Exception as e:  # noqa: BLE001
        sys.stderr.write(f"{RED}Error: {e}{RESET}\n")
        sys.exit(1)


if __name__ == "__main__":
    run()
