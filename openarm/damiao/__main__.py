"""Command-line interface for controlling Damiao motors over CAN.

This module provides a CLI to control Damiao motors via CAN bus communication.
Uses the high-level Motor class and encode/decode functions for proper async handling.

Commands:
    enable           Enable the specified motor
    disable          Disable the specified motor
    set-zero         Set motor zero position
    refresh          Get current motor status
    control          Control motor in various modes (MIT, pos_vel, vel, pos_force)
    param            Get/set semantic motor parameters
    save             Save motor parameters to flash

Examples:
    # When slave_id and master_id are the same:
    python -m openarm.damiao enable --iface can0 --motor-type DM4310 1 1
    python -m openarm.damiao control mit --iface can0 --motor-type DM4310 1 1 \
        50 0.3 0 0 0
    python -m openarm.damiao param get --iface can0 --motor-type DM4310 1 1 \
        over_voltage
    python -m openarm.damiao param set --iface can0 --motor-type DM4310 1 1 \
        max_speed 10.0

    # When slave_id and master_id are different:
    python -m openarm.damiao enable --iface can0 --motor-type DM4310 1 2

"""

import argparse
import asyncio
import sys
from collections.abc import Coroutine
from typing import Any

from openarm.bus import Bus

from . import (
    MitControlParams,
    Motor,
    MotorState,
    MotorType,
    PosForceControlParams,
    PosVelControlParams,
    VelControlParams,
)


# Output utilities for CLI
def _output(message: str) -> None:
    """Output message to stdout."""
    print(message)  # noqa: T201


def _error(message: str) -> None:
    """Output error message to stderr."""
    print(message, file=sys.stderr)  # noqa: T201


def _format_motor_state(state: MotorState, slave_id: int) -> str:
    """Format motor state for display."""
    lines = [
        f"Motor {slave_id} state:",
        f"  Position: {state.position:.6f} rad",
        f"  Velocity: {state.velocity:.6f} rad/s",
        f"  Torque: {state.torque:.6f} Nm",
        f"  MOS Temp: {state.temp_mos}°C",
        f"  Rotor Temp: {state.temp_rotor}°C",
    ]
    return "\n".join(lines)


def _create_bus(iface: str) -> Bus:
    """Create CAN bus instance with SocketCAN interface."""
    import can  # noqa: PLC0415

    can_bus = can.Bus(channel=iface, interface="socketcan")
    return Bus(can_bus)


async def _enable(args: argparse.Namespace) -> None:
    """Enable motor using Motor class."""
    bus = _create_bus(args.iface)
    motor_type = MotorType(args.motor_type)
    motor = Motor(
        bus, slave_id=args.slave_id, master_id=args.master_id, motor_type=motor_type
    )

    state = await motor.enable()

    _output(f"Motor {args.slave_id} enabled successfully")
    _output(_format_motor_state(state, args.slave_id))


async def _disable(args: argparse.Namespace) -> None:
    """Disable motor using Motor class."""
    bus = _create_bus(args.iface)
    motor_type = MotorType(args.motor_type)
    motor = Motor(
        bus, slave_id=args.slave_id, master_id=args.master_id, motor_type=motor_type
    )

    state = await motor.disable()

    _output(f"Motor {args.slave_id} disabled successfully")
    _output(_format_motor_state(state, args.slave_id))


async def _set_zero(args: argparse.Namespace) -> None:
    """Set motor zero position using Motor class."""
    bus = _create_bus(args.iface)
    motor_type = MotorType(args.motor_type)
    motor = Motor(
        bus, slave_id=args.slave_id, master_id=args.master_id, motor_type=motor_type
    )

    state = await motor.set_zero_position()

    _output(f"Zero position set for motor {args.slave_id}")
    _output(_format_motor_state(state, args.slave_id))


async def _refresh(args: argparse.Namespace) -> None:
    """Refresh motor status using Motor class."""
    bus = _create_bus(args.iface)
    motor_type = MotorType(args.motor_type)
    motor = Motor(
        bus, slave_id=args.slave_id, master_id=args.master_id, motor_type=motor_type
    )

    state = await motor.refresh_status()
    _output(_format_motor_state(state, args.slave_id))


async def _control_mit(args: argparse.Namespace) -> None:
    """Control motor in MIT mode using Motor class."""
    bus = _create_bus(args.iface)
    motor_type = MotorType(args.motor_type)
    motor = Motor(
        bus, slave_id=args.slave_id, master_id=args.master_id, motor_type=motor_type
    )

    params = MitControlParams(
        kp=args.kp,
        kd=args.kd,
        q=args.q,
        dq=args.dq,
        tau=args.tau,
    )

    state = await motor.control_mit(params)

    _output(f"MIT control sent to motor {args.slave_id}")
    _output(
        f"Response - Position: {state.position:.6f}, "
        f"Velocity: {state.velocity:.6f}, "
        f"Torque: {state.torque:.6f}"
    )


async def _control_pos_vel(args: argparse.Namespace) -> None:
    """Control motor in position/velocity mode using Motor class."""
    bus = _create_bus(args.iface)
    motor_type = MotorType(args.motor_type)
    motor = Motor(
        bus, slave_id=args.slave_id, master_id=args.master_id, motor_type=motor_type
    )

    params = PosVelControlParams(position=args.pos, velocity=args.vel)
    state = await motor.control_pos_vel(params)

    _output(f"Position/velocity control sent to motor {args.slave_id}")
    _output(
        f"Response - Position: {state.position:.6f}, "
        f"Velocity: {state.velocity:.6f}, "
        f"Torque: {state.torque:.6f}"
    )


async def _control_vel(args: argparse.Namespace) -> None:
    """Control motor in velocity mode using Motor class."""
    bus = _create_bus(args.iface)
    motor_type = MotorType(args.motor_type)
    motor = Motor(
        bus, slave_id=args.slave_id, master_id=args.master_id, motor_type=motor_type
    )

    params = VelControlParams(velocity=args.vel)
    state = await motor.control_vel(params)

    _output(f"Velocity control sent to motor {args.slave_id}")
    _output(
        f"Response - Position: {state.position:.6f}, "
        f"Velocity: {state.velocity:.6f}, "
        f"Torque: {state.torque:.6f}"
    )


async def _control_pos_force(args: argparse.Namespace) -> None:
    """Control motor in position/force mode using Motor class."""
    bus = _create_bus(args.iface)
    motor_type = MotorType(args.motor_type)
    motor = Motor(
        bus, slave_id=args.slave_id, master_id=args.master_id, motor_type=motor_type
    )

    params = PosForceControlParams(
        position=args.pos, velocity=args.vel, current_norm=args.i_norm
    )

    state = await motor.control_pos_force(params)

    _output(f"Position/force control sent to motor {args.slave_id}")
    _output(
        f"Response - Position: {state.position:.6f}, "
        f"Velocity: {state.velocity:.6f}, "
        f"Torque: {state.torque:.6f}"
    )


async def _save_parameters(args: argparse.Namespace) -> None:
    """Save motor parameters to flash using Motor class."""
    bus = _create_bus(args.iface)
    motor_type = MotorType(args.motor_type)
    motor = Motor(
        bus, slave_id=args.slave_id, master_id=args.master_id, motor_type=motor_type
    )

    response = await motor.save_parameters()

    if response.success:
        _output(f"Parameters saved successfully for motor {args.slave_id}")
    else:
        _error(f"Failed to save parameters for motor {args.slave_id}")
        sys.exit(1)


# High-level Motor class interface functions
async def _motor_get_param(args: argparse.Namespace) -> None:
    """Get semantic motor parameter using Motor class."""
    bus = _create_bus(args.iface)
    motor_type = MotorType(args.motor_type)
    motor = Motor(
        bus, slave_id=args.slave_id, master_id=args.master_id, motor_type=motor_type
    )

    param_name = args.parameter

    # Map parameter names to Motor class methods
    param_methods = {
        # Voltage Protection
        "under_voltage": motor.get_under_voltage,
        "over_voltage": motor.get_over_voltage,
        # Motor Characteristics
        "torque_coefficient": motor.get_torque_coefficient,
        "gear_efficiency": motor.get_gear_efficiency,
        # Protection Limits
        "over_temperature": motor.get_over_temperature,
        "over_current": motor.get_over_current,
        # Mapping Limits
        "position_limit": motor.get_position_limit,
        "velocity_limit": motor.get_velocity_limit,
        "torque_limit": motor.get_torque_limit,
        # Control Loop Parameters
        "velocity_kp": motor.get_velocity_kp,
        "velocity_ki": motor.get_velocity_ki,
        "position_kp": motor.get_position_kp,
        "position_ki": motor.get_position_ki,
        # Current and Speed Loop Parameters (NEW)
        "current_loop_bandwidth": motor.get_current_loop_bandwidth,
        "speed_loop_damping": motor.get_speed_loop_damping,
        "speed_loop_filter_bandwidth": motor.get_speed_loop_filter_bandwidth,
        "current_loop_gain": motor.get_current_loop_gain,
        "speed_loop_gain": motor.get_speed_loop_gain,
        # Read-Only Motor Information
        "hardware_version": motor.get_hardware_version,
        "software_version": motor.get_software_version,
        "serial_number": motor.get_serial_number,
        "gear_ratio": motor.get_gear_ratio,
        "motor_damping": motor.get_motor_damping,  # NEW
        "motor_inertia": motor.get_motor_inertia,  # NEW
        "motor_pole_pairs": motor.get_motor_pole_pairs,  # NEW
        "motor_phase_resistance": motor.get_motor_phase_resistance,  # NEW
        "motor_phase_inductance": motor.get_motor_phase_inductance,  # NEW
        "motor_flux": motor.get_motor_flux,  # NEW
        "sub_version": motor.get_sub_version,  # NEW
        # Motion Parameters
        "acceleration": motor.get_acceleration,
        "deceleration": motor.get_deceleration,
        "max_speed": motor.get_max_speed,
        # Communication Parameters
        "master_id": motor.get_master_id,
        "slave_id": motor.get_slave_id,
        "timeout": motor.get_timeout,
        "can_baudrate": motor.get_can_baudrate,
        # Read-Only Calibration and Position
        "phase_u_offset": motor.get_phase_u_offset,  # NEW
        "phase_v_offset": motor.get_phase_v_offset,  # NEW
        "compensation_factor_1": motor.get_compensation_factor_1,  # NEW
        "compensation_factor_2": motor.get_compensation_factor_2,  # NEW
        "angle_offset": motor.get_angle_offset,  # NEW
        "direction": motor.get_direction,  # NEW
        "motor_position": motor.get_motor_position,  # NEW
        "output_shaft_position": motor.get_output_shaft_position,  # NEW
    }

    if param_name not in param_methods:
        _error(f"Unknown parameter: {param_name}")
        _error(f"Available parameters: {', '.join(param_methods.keys())}")
        sys.exit(1)

    value = await param_methods[param_name]()
    _output(f"{param_name}: {value}")


async def _motor_set_param(args: argparse.Namespace) -> None:
    """Set semantic motor parameter using Motor class."""
    bus = _create_bus(args.iface)
    motor_type = MotorType(args.motor_type)
    motor = Motor(
        bus, slave_id=args.slave_id, master_id=args.master_id, motor_type=motor_type
    )

    param_name = args.parameter
    value = args.value

    # Map parameter names to Motor class setter methods
    param_methods = {
        # Voltage Protection
        "under_voltage": motor.set_under_voltage,
        "over_voltage": motor.set_over_voltage,
        # Motor Characteristics
        "torque_coefficient": motor.set_torque_coefficient,
        "gear_efficiency": motor.set_gear_efficiency,
        # Protection Limits
        "over_temperature": motor.set_over_temperature,
        "over_current": motor.set_over_current,
        # Mapping Limits
        "position_limit": motor.set_position_limit,
        "velocity_limit": motor.set_velocity_limit,
        "torque_limit": motor.set_torque_limit,
        # Control Loop Parameters
        "velocity_kp": motor.set_velocity_kp,
        "velocity_ki": motor.set_velocity_ki,
        "position_kp": motor.set_position_kp,
        "position_ki": motor.set_position_ki,
        # Current and Speed Loop Parameters (NEW)
        "current_loop_bandwidth": motor.set_current_loop_bandwidth,
        "speed_loop_damping": motor.set_speed_loop_damping,
        "speed_loop_filter_bandwidth": motor.set_speed_loop_filter_bandwidth,
        "current_loop_gain": motor.set_current_loop_gain,
        "speed_loop_gain": motor.set_speed_loop_gain,
        # Motion Parameters
        "acceleration": motor.set_acceleration,
        "deceleration": motor.set_deceleration,
        "max_speed": motor.set_max_speed,
        # Communication Parameters
        "master_id": motor.set_master_id,
        "slave_id": motor.set_slave_id,
        "timeout": motor.set_timeout,
        "can_baudrate": motor.set_can_baudrate,
        # Note: Read-only parameters are not included in setter methods
    }

    if param_name not in param_methods:
        _error(f"Unknown parameter: {param_name}")
        _error(f"Available parameters: {', '.join(param_methods.keys())}")
        sys.exit(1)

    # Handle int vs float parameters
    if param_name in ["master_id", "slave_id", "timeout", "can_baudrate"]:
        value = int(value)
    else:
        value = float(value)

    result = await param_methods[param_name](value)
    _output(f"{param_name} set to: {result}")


def _run_async(coro: Coroutine[Any, Any, None]) -> None:
    """Run async coroutine in CLI context."""
    asyncio.run(coro)


def _main() -> None:
    """Execute main CLI entry point."""
    parser = argparse.ArgumentParser(
        description="Damiao motor control CLI",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    subparsers = parser.add_subparsers(dest="command", required=True)

    # Common arguments
    def add_common_args(parser: argparse.ArgumentParser) -> None:
        parser.add_argument("--iface", default="can0", help="CAN interface to use")
        parser.add_argument(
            "slave_id", type=int, help="Motor slave ID (for sending commands)"
        )
        parser.add_argument(
            "master_id", type=int, help="Motor master ID (for receiving responses)"
        )
        parser.add_argument(
            "--motor-type",
            required=True,
            choices=[t.value for t in MotorType],
            help="Motor type",
        )

    # Enable command
    enable_parser = subparsers.add_parser("enable", help="Enable motor")
    add_common_args(enable_parser)
    enable_parser.set_defaults(func=_enable)

    # Disable command
    disable_parser = subparsers.add_parser("disable", help="Disable motor")
    add_common_args(disable_parser)
    disable_parser.set_defaults(func=_disable)

    # Set zero command
    set_zero_parser = subparsers.add_parser("set-zero", help="Set motor zero position")
    add_common_args(set_zero_parser)
    set_zero_parser.set_defaults(func=_set_zero)

    # Refresh command
    refresh_parser = subparsers.add_parser("refresh", help="Get motor status")
    add_common_args(refresh_parser)
    refresh_parser.set_defaults(func=_refresh)

    # Control commands
    control_parser = subparsers.add_parser("control", help="Control motor")
    control_subparsers = control_parser.add_subparsers(
        dest="control_mode", required=True
    )

    # MIT control
    mit_parser = control_subparsers.add_parser("mit", help="Control motor in MIT mode")
    add_common_args(mit_parser)
    mit_parser.add_argument("kp", type=float, help="Proportional gain (0-500)")
    mit_parser.add_argument("kd", type=float, help="Derivative gain (0-5)")
    mit_parser.add_argument("q", type=float, help="Desired position (radians)")
    mit_parser.add_argument("dq", type=float, help="Desired velocity (rad/s)")
    mit_parser.add_argument("tau", type=float, help="Desired torque (Nm)")
    mit_parser.set_defaults(func=_control_mit)

    # Position/velocity control
    pos_vel_parser = control_subparsers.add_parser(
        "pos_vel", help="Control motor in position/velocity mode"
    )
    add_common_args(pos_vel_parser)
    pos_vel_parser.add_argument("pos", type=float, help="Desired position (radians)")
    pos_vel_parser.add_argument("vel", type=float, help="Desired velocity (rad/s)")
    pos_vel_parser.set_defaults(func=_control_pos_vel)

    # Velocity control
    vel_parser = control_subparsers.add_parser(
        "vel", help="Control motor in velocity mode"
    )
    add_common_args(vel_parser)
    vel_parser.add_argument("vel", type=float, help="Desired velocity (rad/s)")
    vel_parser.set_defaults(func=_control_vel)

    # Position/force control
    pos_force_parser = control_subparsers.add_parser(
        "pos_force", help="Control motor in position/force mode"
    )
    add_common_args(pos_force_parser)
    pos_force_parser.add_argument("pos", type=float, help="Desired position (radians)")
    pos_force_parser.add_argument("vel", type=float, help="Desired velocity (rad/s)")
    pos_force_parser.add_argument("i_norm", type=float, help="Normalized current (0-1)")
    pos_force_parser.set_defaults(func=_control_pos_force)

    # Parameter commands (high-level)
    param_parser = subparsers.add_parser("param", help="Semantic parameter operations")
    param_subparsers = param_parser.add_subparsers(dest="param_op", required=True)

    # Parameter get
    param_get_parser = param_subparsers.add_parser("get", help="Get semantic parameter")
    add_common_args(param_get_parser)
    param_get_parser.add_argument(
        "parameter",
        help="Parameter name (e.g., over_voltage, torque_limit, velocity_kp)",
    )
    param_get_parser.set_defaults(func=_motor_get_param)

    # Parameter set
    param_set_parser = param_subparsers.add_parser("set", help="Set semantic parameter")
    add_common_args(param_set_parser)
    param_set_parser.add_argument(
        "parameter",
        help="Parameter name (e.g., over_voltage, torque_limit, velocity_kp)",
    )
    param_set_parser.add_argument("value", type=float, help="Parameter value")
    param_set_parser.set_defaults(func=_motor_set_param)

    # Save command
    save_parser = subparsers.add_parser("save", help="Save motor parameters to flash")
    add_common_args(save_parser)
    save_parser.set_defaults(func=_save_parameters)

    args = parser.parse_args()

    # Run the async function
    _run_async(args.func(args))


if __name__ == "__main__":
    _main()
