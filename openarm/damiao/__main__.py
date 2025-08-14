"""Command-line interface for controlling Damiao motors over CAN.

This module provides a CLI to send enable or disable commands to
Damiao motors via a SocketCAN interface.

Commands:
    enable   Enable the specified motor.
    disable  Disable the specified motor.

Examples:
    python -m openarm.damiao enable --iface can0 1
    python -m openarm.damiao disable 2

"""

import argparse
import sys

import can

from . import (
    MotorType,
    RegisterResponse,
    StateResponse,
    control_mit_command,
    control_pos_force_command,
    control_pos_vel_command,
    control_vel_command,
    decode_response,
    disable_command,
    enable_command,
    read_register_command,
    refresh_command,
    set_zero_command,
    write_register_command,
)


def _enable(args: argparse.Namespace) -> None:
    with can.Bus(channel=args.iface, interface="socketcan") as bus:
        bus.send(enable_command(args.slave_id))


def _disable(args: argparse.Namespace) -> None:
    with can.Bus(channel=args.iface, interface="socketcan") as bus:
        bus.send(disable_command(args.slave_id))


def _set_zero(args: argparse.Namespace) -> None:
    with can.Bus(channel=args.iface, interface="socketcan") as bus:
        bus.send(set_zero_command(args.slave_id))


def _refresh(args: argparse.Namespace) -> None:
    with can.Bus(channel=args.iface, interface="socketcan") as bus:
        bus.send(refresh_command(args.slave_id))
        for msg in bus:
            res = decode_response(msg)
            if isinstance(res, StateResponse) and res.master_id == args.master_id:
                if args.motor_type is not None:
                    res = res.as_motor(MotorType(args.motor_type))

                sys.stdout.write(f"q: {res.q}\n")
                sys.stdout.write(f"dq: {res.dq}\n")
                sys.stdout.write(f"tau: {res.tau}\n")
                sys.stdout.write(f"t mos: {res.t_mos}\n")
                sys.stdout.write(f"t rotor: {res.t_rotor}\n")
                return


def _control_mit(args: argparse.Namespace) -> None:
    with can.Bus(channel=args.iface, interface="socketcan") as bus:
        bus.send(
            control_mit_command(
                MotorType(args.motor_type),
                args.slave_id,
                args.kp,
                args.kd,
                args.q,
                args.dq,
                args.tau,
            )
        )


def _control_pos_vel(args: argparse.Namespace) -> None:
    with can.Bus(channel=args.iface, interface="socketcan") as bus:
        bus.send(control_pos_vel_command(args.slave_id, args.pos, args.vel))


def _control_vel(args: argparse.Namespace) -> None:
    with can.Bus(channel=args.iface, interface="socketcan") as bus:
        bus.send(control_vel_command(args.slave_id, args.vel))


def _control_pos_force(args: argparse.Namespace) -> None:
    with can.Bus(channel=args.iface, interface="socketcan") as bus:
        bus.send(
            control_pos_force_command(args.slave_id, args.pos, args.vel, args.i_norm)
        )


def _register_read(args: argparse.Namespace) -> None:
    with can.Bus(channel=args.iface, interface="socketcan") as bus:
        bus.send(read_register_command(args.slave_id, args.address))
        for msg in bus:
            res = decode_response(msg)
            if isinstance(res, RegisterResponse) and res.slave_id == args.slave_id:
                value = res.as_float() if args.as_float else res.as_int()
                sys.stdout.write(f"{value}\n")
                return


def _register_write(args: argparse.Namespace) -> None:
    with can.Bus(channel=args.iface, interface="socketcan") as bus:
        bus.send(
            write_register_command(
                args.slave_id, args.address, args.value, as_float=args.as_float
            )
        )

        for msg in bus:
            res = decode_response(msg)
            if isinstance(res, RegisterResponse) and res.slave_id == args.slave_id:
                value = res.as_float() if args.as_float else res.as_int()
                sys.stdout.write(f"{value}\n")
                return


def _main() -> None:
    parser = argparse.ArgumentParser()
    subparsers = parser.add_subparsers(dest="command", required=True)

    enable_parser = subparsers.add_parser("enable", help="Enable motor")
    enable_parser.add_argument("--iface", default="can0", help="CAN interface to use")
    enable_parser.add_argument("slave_id", type=int, help="Slave ID of the motor")
    enable_parser.set_defaults(func=_enable)

    disable_parser = subparsers.add_parser("disable", help="Disable motor")
    disable_parser.add_argument("--iface", default="can0", help="CAN interface to use")
    disable_parser.add_argument("slave_id", type=int, help="Slave ID of the motor")
    disable_parser.set_defaults(func=_disable)

    disable_parser = subparsers.add_parser("set_zero", help="Set motor zero position")
    disable_parser.add_argument("--iface", default="can0", help="CAN interface to use")
    disable_parser.add_argument("slave_id", type=int, help="Slave ID of the motor")
    disable_parser.set_defaults(func=_set_zero)

    refresh_parser = subparsers.add_parser("refresh", help="Refresh motor")
    refresh_parser.add_argument("--iface", default="can0", help="CAN interface to use")
    refresh_parser.add_argument("--motor-type", default=None, help="The motor type")
    refresh_parser.add_argument("master_id", type=int, help="Master ID of the motor")
    refresh_parser.add_argument("slave_id", type=int, help="Slave ID of the motor")
    refresh_parser.set_defaults(func=_refresh)

    control_parser = subparsers.add_parser("control", help="Control motor")
    control_subparsers = control_parser.add_subparsers(dest="command", required=True)

    mit_parser = control_subparsers.add_parser("mit", help="Control motor in MIT mode")
    mit_parser.add_argument("--iface", default="can0", help="CAN interface to use")
    mit_parser.add_argument("motor_type", help="The motor type")
    mit_parser.add_argument("slave_id", type=int, help="Slave ID of the motor")
    mit_parser.add_argument("kp", type=float, help="Proportional gain")
    mit_parser.add_argument("kd", type=float, help="Derivative gain")
    mit_parser.add_argument("q", type=float, help="Desired position (radians)")
    mit_parser.add_argument("dq", type=float, help="Desired velocity (radians/second)")
    mit_parser.add_argument("tau", type=float, help="Desired torque (Nm)")
    mit_parser.set_defaults(func=_control_mit)

    pos_vel_parser = control_subparsers.add_parser(
        "pos_vel", help="Control motor in position and velocity mode"
    )
    pos_vel_parser.add_argument("--iface", default="can0", help="CAN interface to use")
    pos_vel_parser.add_argument("slave_id", type=int, help="Slave ID of the motor")
    pos_vel_parser.add_argument("pos", type=float, help="Desired position (radians)")
    pos_vel_parser.add_argument(
        "vel", type=float, help="Desired velocity (radians/second)"
    )
    pos_vel_parser.set_defaults(func=_control_pos_vel)

    vel_parser = control_subparsers.add_parser(
        "vel", help="Control motor in velocity mode"
    )
    vel_parser.add_argument("--iface", default="can0", help="CAN interface to use")
    vel_parser.add_argument("slave_id", type=int, help="Slave ID of the motor")
    vel_parser.add_argument("vel", type=float, help="Desired velocity (radians/second)")
    vel_parser.set_defaults(func=_control_vel)

    pos_force_parser = control_subparsers.add_parser(
        "pos_force", help="Control motor in force-position hybrid mode"
    )
    pos_force_parser.add_argument(
        "--iface", default="can0", help="CAN interface to use"
    )
    pos_force_parser.add_argument("slave_id", type=int, help="Slave ID of the motor")
    pos_force_parser.add_argument("pos", type=float, help="Desired position (radians)")
    pos_force_parser.add_argument(
        "vel", type=float, help="Desired velocity (radians/second)"
    )
    pos_force_parser.add_argument(
        "i_norm", type=float, help="Desired normalized current (0-1)"
    )
    pos_force_parser.set_defaults(func=_control_pos_force)

    register_parser = subparsers.add_parser("register", help="Manage motor register")
    register_subparsers = register_parser.add_subparsers(dest="command", required=True)

    read_parser = register_subparsers.add_parser("read", help="Read motor register")
    read_parser.add_argument("--iface", default="can0", help="CAN interface to use")
    read_parser.add_argument(
        "--as-float", action="store_true", default=False, help="Read as float"
    )
    read_parser.add_argument("slave_id", type=int, help="Slave ID of the motor")
    read_parser.add_argument("address", type=int, help="Register address to read")
    read_parser.set_defaults(func=_register_read)

    write_parser = register_subparsers.add_parser("write", help="Write motor register")
    write_parser.add_argument("--iface", default="can0", help="CAN interface to use")
    write_parser.add_argument(
        "--as-float", action="store_true", default=False, help="Write as float"
    )
    write_parser.add_argument("slave_id", type=int, help="Slave ID of the motor")
    write_parser.add_argument("address", type=int, help="Register address to write")
    write_parser.add_argument("value", type=float, help="Value to write")
    write_parser.set_defaults(func=_register_write)

    args = parser.parse_args()
    args.func(args)


if __name__ == "__main__":
    _main()
