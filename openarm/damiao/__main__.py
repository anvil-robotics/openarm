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

import can

from . import disable_command, enable_command


def _enable(args: argparse.Namespace) -> None:
    with can.Bus(channel=args.iface, interface="socketcan") as bus:
        bus.send(enable_command(args.slave_id))


def _disable(args: argparse.Namespace) -> None:
    with can.Bus(channel=args.iface, interface="socketcan") as bus:
        bus.send(disable_command(args.slave_id))


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

    args = parser.parse_args()
    args.func(args)


if __name__ == "__main__":
    _main()
