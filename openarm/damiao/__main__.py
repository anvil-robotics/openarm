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
    RegisterResponse,
    StateResponse,
    decode_response,
    disable_command,
    enable_command,
    read_register_command,
    refresh_command,
    write_register_command,
)


def _enable(args: argparse.Namespace) -> None:
    with can.Bus(channel=args.iface, interface="socketcan") as bus:
        bus.send(enable_command(args.slave_id))


def _disable(args: argparse.Namespace) -> None:
    with can.Bus(channel=args.iface, interface="socketcan") as bus:
        bus.send(disable_command(args.slave_id))


def _refresh(args: argparse.Namespace) -> None:
    with can.Bus(channel=args.iface, interface="socketcan") as bus:
        bus.send(refresh_command(args.slave_id))
        for msg in bus:
            res = decode_response(msg)
            if isinstance(res, StateResponse) and res.master_id == args.master_id:
                sys.stdout.write(f"q: {res.q}\n")
                sys.stdout.write(f"dq: {res.dq}\n")
                sys.stdout.write(f"tau: {res.tau}\n")
                sys.stdout.write(f"t mos: {res.t_mos}\n")
                sys.stdout.write(f"t rotor: {res.t_rotor}\n")
                return


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

    refresh_parser = subparsers.add_parser("refresh", help="Refresh motor")
    refresh_parser.add_argument("--iface", default="can0", help="CAN interface to use")
    refresh_parser.add_argument("master_id", type=int, help="Master ID of the motor")
    refresh_parser.add_argument("slave_id", type=int, help="Slave ID of the motor")
    refresh_parser.set_defaults(func=_refresh)

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
