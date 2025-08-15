#!/usr/bin/env python3
"""NetCAN main entry point."""

import argparse
import asyncio
import logging
import socket

from can import Bus

from .server import Server
from .transport import SocketTransport


def main() -> None:
    """Start NetCAN server."""
    logging.basicConfig(level=logging.INFO)
    logger = logging.getLogger(__name__)

    parser = argparse.ArgumentParser(description="NetCAN Server")
    parser.add_argument(
        "--port",
        "-p",
        type=int,
        default=11898,
        help="Port to listen on (default: 11898)",
    )
    parser.add_argument(
        "--bus",
        "-b",
        type=str,
        default="socketcan",
        help="CAN bus interface type (default: socketcan)",
    )
    parser.add_argument(
        "--channel",
        "-c",
        type=str,
        default="can0",
        help="CAN channel/interface name (default: can0)",
    )

    args = parser.parse_args()

    # Initialize CAN bus
    bus = Bus(interface=args.bus, channel=args.channel)

    # Create server
    server = Server(bus)

    # Setup socket server
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(("", args.port))
    sock.listen(5)

    logger.info("NetCAN server listening on port %d", args.port)
    logger.info("CAN bus: %s on channel %s", args.bus, args.channel)

    try:
        while True:
            client_sock, addr = sock.accept()
            logger.info("Client connected from %s", addr)
            transport = SocketTransport(client_sock)
            server.attach(transport)

            # Start server loop
            asyncio.run(server.loop())

    except KeyboardInterrupt:
        logger.info("Shutting down server...")
    finally:
        sock.close()
        bus.shutdown()


if __name__ == "__main__":
    main()
