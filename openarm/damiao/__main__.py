# ruff: noqa: T201 E501 ANN401
"""Command-line interface for controlling Damiao motors over CAN.

This module provides a CLI to control Damiao motors via CAN bus communication.
Uses the high-level Motor class and encode/decode functions for proper async handling.

Commands:
    enable           Enable the specified motor
    disable          Disable the specified motor
    set-zero         Set motor zero position
    refresh          Get current motor status
    control          Control motor in various modes (MIT, pos_vel, vel, pos_force)
    register         Read/write motor registers
    param            Get/set semantic motor parameters
    save             Save motor parameters to flash

Examples:
    python -m openarm.damiao enable --iface can0 --motor-type DM4310 1
    python -m openarm.damiao control mit --iface can0 --motor-type DM4310 1 50 0.3 0 0 0
    python -m openarm.damiao param get --iface can0 --motor-type DM4310 1 over_voltage
    python -m openarm.damiao register read --iface can0 1 21 --as-float

"""

import argparse
import asyncio
import sys
from typing import Any

from openarm.bus import Bus

from . import (
    MOTOR_LIMITS,
    ControlMode,
    MitControlParams,
    Motor,
    MotorType,
    PosForceControlParams,
    PosVelControlParams,
    RegisterAddress,
    VelControlParams,
)
from .encoding import (
    decode_acknowledgment,
    decode_motor_state,
    decode_register_float,
    decode_register_int,
    encode_control_mit,
    encode_control_pos_vel,
    encode_control_torque_pos,
    encode_control_vel,
    encode_disable_motor,
    encode_enable_motor,
    encode_enable_motor_legacy,
    encode_read_register,
    encode_refresh_status,
    encode_save_parameters,
    encode_set_zero_position,
    encode_write_register_float,
    encode_write_register_int,
)


def _create_bus(iface: str) -> Bus:
    """Create CAN bus instance with SocketCAN interface."""
    import can  # noqa: PLC0415

    can_bus = can.Bus(channel=iface, interface="socketcan")
    return Bus(can_bus)


async def _enable(args: argparse.Namespace) -> None:
    """Enable motor using encode/decode pattern."""
    bus = _create_bus(args.iface)

    if args.legacy:
        control_mode = ControlMode(args.control_mode)
        encode_enable_motor_legacy(bus, args.slave_id, control_mode)
    else:
        encode_enable_motor(bus, args.slave_id)

    response = await decode_acknowledgment(bus)
    if response.success:
        print(f"Motor {args.slave_id} enabled successfully")
    else:
        print(f"Failed to enable motor {args.slave_id}")
        sys.exit(1)


async def _disable(args: argparse.Namespace) -> None:
    """Disable motor using encode/decode pattern."""
    bus = _create_bus(args.iface)

    encode_disable_motor(bus, args.slave_id)
    response = await decode_acknowledgment(bus)

    if response.success:
        print(f"Motor {args.slave_id} disabled successfully")
    else:
        print(f"Failed to disable motor {args.slave_id}")
        sys.exit(1)


async def _set_zero(args: argparse.Namespace) -> None:
    """Set motor zero position using encode/decode pattern."""
    bus = _create_bus(args.iface)

    encode_set_zero_position(bus, args.slave_id)
    response = await decode_acknowledgment(bus)

    if response.success:
        print(f"Zero position set for motor {args.slave_id}")
    else:
        print(f"Failed to set zero position for motor {args.slave_id}")
        sys.exit(1)


async def _refresh(args: argparse.Namespace) -> None:
    """Refresh motor status using encode/decode pattern."""
    bus = _create_bus(args.iface)
    motor_type = MotorType(args.motor_type)
    motor_limits = MOTOR_LIMITS[motor_type]

    encode_refresh_status(bus, args.slave_id)
    state = await decode_motor_state(bus, args.slave_id, motor_limits)

    print(f"Motor {args.slave_id} status:")
    print(f"  Position: {state.position:.6f} rad")
    print(f"  Velocity: {state.velocity:.6f} rad/s")
    print(f"  Torque: {state.torque:.6f} Nm")
    print(f"  MOS Temp: {state.temp_mos}°C")
    print(f"  Rotor Temp: {state.temp_rotor}°C")


async def _control_mit(args: argparse.Namespace) -> None:
    """Control motor in MIT mode using encode/decode pattern."""
    bus = _create_bus(args.iface)
    motor_type = MotorType(args.motor_type)
    motor_limits = MOTOR_LIMITS[motor_type]

    params = MitControlParams(
        kp=args.kp,
        kd=args.kd,
        q=args.q,
        dq=args.dq,
        tau=args.tau,
    )

    encode_control_mit(bus, args.slave_id, motor_limits, params)
    state = await decode_motor_state(bus, args.slave_id, motor_limits)

    print(f"MIT control sent to motor {args.slave_id}")
    print(
        f"Response - Position: {state.position:.6f}, Velocity: {state.velocity:.6f}, Torque: {state.torque:.6f}"
    )


async def _control_pos_vel(args: argparse.Namespace) -> None:
    """Control motor in position/velocity mode using encode/decode pattern."""
    bus = _create_bus(args.iface)
    motor_type = MotorType(args.motor_type)
    motor_limits = MOTOR_LIMITS[motor_type]

    params = PosVelControlParams(position=args.pos, velocity=args.vel)

    encode_control_pos_vel(bus, args.slave_id, params)
    state = await decode_motor_state(bus, args.slave_id, motor_limits)

    print(f"Position/velocity control sent to motor {args.slave_id}")
    print(
        f"Response - Position: {state.position:.6f}, Velocity: {state.velocity:.6f}, Torque: {state.torque:.6f}"
    )


async def _control_vel(args: argparse.Namespace) -> None:
    """Control motor in velocity mode using encode/decode pattern."""
    bus = _create_bus(args.iface)
    motor_type = MotorType(args.motor_type)
    motor_limits = MOTOR_LIMITS[motor_type]

    params = VelControlParams(velocity=args.vel)

    encode_control_vel(bus, args.slave_id, params)
    state = await decode_motor_state(bus, args.slave_id, motor_limits)

    print(f"Velocity control sent to motor {args.slave_id}")
    print(
        f"Response - Position: {state.position:.6f}, Velocity: {state.velocity:.6f}, Torque: {state.torque:.6f}"
    )


async def _control_pos_force(args: argparse.Namespace) -> None:
    """Control motor in position/force mode using encode/decode pattern."""
    bus = _create_bus(args.iface)
    motor_type = MotorType(args.motor_type)
    motor_limits = MOTOR_LIMITS[motor_type]

    params = PosForceControlParams(
        position=args.pos, velocity=args.vel, current_norm=args.i_norm
    )

    encode_control_torque_pos(bus, args.slave_id, params)
    state = await decode_motor_state(bus, args.slave_id, motor_limits)

    print(f"Position/force control sent to motor {args.slave_id}")
    print(
        f"Response - Position: {state.position:.6f}, Velocity: {state.velocity:.6f}, Torque: {state.torque:.6f}"
    )


async def _register_read(args: argparse.Namespace) -> None:
    """Read motor register using encode/decode pattern."""
    bus = _create_bus(args.iface)

    # Convert register address if it's a name
    if isinstance(args.address, str):
        try:
            address = RegisterAddress[args.address.upper()]
        except KeyError:
            print(f"Unknown register name: {args.address}")
            sys.exit(1)
    else:
        address = RegisterAddress(args.address)

    encode_read_register(bus, args.slave_id, address)

    if args.as_float:
        value = await decode_register_float(bus)
        print(f"{value}")
    else:
        value = await decode_register_int(bus)
        print(f"{value}")


async def _register_write(args: argparse.Namespace) -> None:
    """Write motor register using encode/decode pattern."""
    bus = _create_bus(args.iface)

    # Convert register address if it's a name
    if isinstance(args.address, str):
        try:
            address = RegisterAddress[args.address.upper()]
        except KeyError:
            print(f"Unknown register name: {args.address}")
            sys.exit(1)
    else:
        address = RegisterAddress(args.address)

    if args.as_float:
        encode_write_register_float(bus, args.slave_id, address, args.value)
        value = await decode_register_float(bus)
        print(f"Written: {value}")
    else:
        encode_write_register_int(bus, args.slave_id, address, int(args.value))
        value = await decode_register_int(bus)
        print(f"Written: {value}")


async def _save_parameters(args: argparse.Namespace) -> None:
    """Save motor parameters to flash using encode/decode pattern."""
    bus = _create_bus(args.iface)

    encode_save_parameters(bus, args.slave_id)
    response = await decode_acknowledgment(bus)

    if response.success:
        print(f"Parameters saved for motor {args.slave_id}")
    else:
        print(f"Failed to save parameters for motor {args.slave_id}")
        sys.exit(1)


# High-level Motor class interface functions
async def _motor_get_param(args: argparse.Namespace) -> None:
    """Get semantic motor parameter using Motor class."""
    bus = _create_bus(args.iface)
    motor_type = MotorType(args.motor_type)
    motor = Motor(bus, args.slave_id, motor_type)

    param_name = args.parameter

    # Map parameter names to Motor class methods
    param_methods = {
        "under_voltage": motor.get_under_voltage,
        "over_voltage": motor.get_over_voltage,
        "torque_coefficient": motor.get_torque_coefficient,
        "gear_efficiency": motor.get_gear_efficiency,
        "over_temperature": motor.get_over_temperature,
        "over_current": motor.get_over_current,
        "position_limit": motor.get_position_limit,
        "velocity_limit": motor.get_velocity_limit,
        "torque_limit": motor.get_torque_limit,
        "velocity_kp": motor.get_velocity_kp,
        "velocity_ki": motor.get_velocity_ki,
        "position_kp": motor.get_position_kp,
        "position_ki": motor.get_position_ki,
        "hardware_version": motor.get_hardware_version,
        "software_version": motor.get_software_version,
        "serial_number": motor.get_serial_number,
        "gear_ratio": motor.get_gear_ratio,
        "acceleration": motor.get_acceleration,
        "deceleration": motor.get_deceleration,
        "max_speed": motor.get_max_speed,
        "master_id": motor.get_master_id,
        "slave_id": motor.get_slave_id,
        "timeout": motor.get_timeout,
        "can_baudrate": motor.get_can_baudrate,
    }

    if param_name not in param_methods:
        print(f"Unknown parameter: {param_name}")
        print(f"Available parameters: {', '.join(param_methods.keys())}")
        sys.exit(1)

    value = await param_methods[param_name]()
    print(f"{param_name}: {value}")


async def _motor_set_param(args: argparse.Namespace) -> None:
    """Set semantic motor parameter using Motor class."""
    bus = _create_bus(args.iface)
    motor_type = MotorType(args.motor_type)
    motor = Motor(bus, args.slave_id, motor_type)

    param_name = args.parameter
    value = args.value

    # Map parameter names to Motor class setter methods
    param_methods = {
        "under_voltage": motor.set_under_voltage,
        "over_voltage": motor.set_over_voltage,
        "torque_coefficient": motor.set_torque_coefficient,
        "gear_efficiency": motor.set_gear_efficiency,
        "over_temperature": motor.set_over_temperature,
        "over_current": motor.set_over_current,
        "position_limit": motor.set_position_limit,
        "velocity_limit": motor.set_velocity_limit,
        "torque_limit": motor.set_torque_limit,
        "velocity_kp": motor.set_velocity_kp,
        "velocity_ki": motor.set_velocity_ki,
        "position_kp": motor.set_position_kp,
        "position_ki": motor.set_position_ki,
        "acceleration": motor.set_acceleration,
        "deceleration": motor.set_deceleration,
        "max_speed": motor.set_max_speed,
        "master_id": motor.set_master_id,
        "slave_id": motor.set_slave_id,
        "timeout": motor.set_timeout,
        "can_baudrate": motor.set_can_baudrate,
    }

    if param_name not in param_methods:
        print(f"Unknown parameter: {param_name}")
        print(f"Available parameters: {', '.join(param_methods.keys())}")
        sys.exit(1)

    # Handle int vs float parameters
    if param_name in ["master_id", "slave_id", "timeout", "can_baudrate"]:
        value = int(value)
    else:
        value = float(value)

    result = await param_methods[param_name](value)
    print(f"{param_name} set to: {result}")


def _run_async(coro: Any) -> None:
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
        parser.add_argument("slave_id", type=int, help="Motor slave ID")

    def add_motor_type_arg(parser: argparse.ArgumentParser) -> None:
        parser.add_argument(
            "--motor-type",
            required=True,
            choices=[t.value for t in MotorType],
            help="Motor type",
        )

    # Enable command
    enable_parser = subparsers.add_parser("enable", help="Enable motor")
    add_common_args(enable_parser)
    add_motor_type_arg(enable_parser)
    enable_parser.add_argument(
        "--legacy", action="store_true", help="Use legacy enable for old firmware"
    )
    enable_parser.add_argument(
        "--control-mode",
        type=int,
        default=1,
        help="Control mode for legacy enable (1=MIT, 2=POS_VEL, 3=VEL, 4=TORQUE_POS)",
    )
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
    add_motor_type_arg(refresh_parser)
    refresh_parser.set_defaults(func=_refresh)

    # Control commands
    control_parser = subparsers.add_parser("control", help="Control motor")
    control_subparsers = control_parser.add_subparsers(
        dest="control_mode", required=True
    )

    # MIT control
    mit_parser = control_subparsers.add_parser("mit", help="Control motor in MIT mode")
    add_common_args(mit_parser)
    add_motor_type_arg(mit_parser)
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
    add_motor_type_arg(pos_vel_parser)
    pos_vel_parser.add_argument("pos", type=float, help="Desired position (radians)")
    pos_vel_parser.add_argument("vel", type=float, help="Desired velocity (rad/s)")
    pos_vel_parser.set_defaults(func=_control_pos_vel)

    # Velocity control
    vel_parser = control_subparsers.add_parser(
        "vel", help="Control motor in velocity mode"
    )
    add_common_args(vel_parser)
    add_motor_type_arg(vel_parser)
    vel_parser.add_argument("vel", type=float, help="Desired velocity (rad/s)")
    vel_parser.set_defaults(func=_control_vel)

    # Position/force control
    pos_force_parser = control_subparsers.add_parser(
        "pos_force", help="Control motor in position/force mode"
    )
    add_common_args(pos_force_parser)
    add_motor_type_arg(pos_force_parser)
    pos_force_parser.add_argument("pos", type=float, help="Desired position (radians)")
    pos_force_parser.add_argument("vel", type=float, help="Desired velocity (rad/s)")
    pos_force_parser.add_argument("i_norm", type=float, help="Normalized current (0-1)")
    pos_force_parser.set_defaults(func=_control_pos_force)

    # Register commands
    register_parser = subparsers.add_parser(
        "register", help="Motor register operations"
    )
    register_subparsers = register_parser.add_subparsers(
        dest="register_op", required=True
    )

    # Register read
    read_parser = register_subparsers.add_parser("read", help="Read motor register")
    add_common_args(read_parser)
    read_parser.add_argument(
        "address", help="Register address (number or name like 'PMAX')"
    )
    read_parser.add_argument(
        "--as-float", action="store_true", help="Read as float value"
    )
    read_parser.set_defaults(func=_register_read)

    # Register write
    write_parser = register_subparsers.add_parser("write", help="Write motor register")
    add_common_args(write_parser)
    write_parser.add_argument(
        "address", help="Register address (number or name like 'PMAX')"
    )
    write_parser.add_argument("value", type=float, help="Value to write")
    write_parser.add_argument(
        "--as-float", action="store_true", help="Write as float value"
    )
    write_parser.set_defaults(func=_register_write)

    # Parameter commands (high-level)
    param_parser = subparsers.add_parser("param", help="Semantic parameter operations")
    param_subparsers = param_parser.add_subparsers(dest="param_op", required=True)

    # Parameter get
    param_get_parser = param_subparsers.add_parser("get", help="Get semantic parameter")
    add_common_args(param_get_parser)
    add_motor_type_arg(param_get_parser)
    param_get_parser.add_argument(
        "parameter",
        help="Parameter name (e.g., over_voltage, torque_limit, velocity_kp)",
    )
    param_get_parser.set_defaults(func=_motor_get_param)

    # Parameter set
    param_set_parser = param_subparsers.add_parser("set", help="Set semantic parameter")
    add_common_args(param_set_parser)
    add_motor_type_arg(param_set_parser)
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
