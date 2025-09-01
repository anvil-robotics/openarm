#!/usr/bin/env python3
"""NetCAN main entry point."""

import argparse
import asyncio
import contextlib
import logging
import signal
from types import FrameType

import can

from openarm.bus import AsyncBus

from .server import AsyncServer
from .transport import AsyncSocketTransport


async def async_main() -> None:
    """Start async NetCAN server."""
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
    parser.add_argument(
        "--host",
        type=str,
        default="",
        help="Host to bind to (default: all interfaces)",
    )

    args = parser.parse_args()

    # Initialize CAN bus
    can_bus = can.Bus(interface=args.bus, channel=args.channel)

    # Wrap in openarm.bus.AsyncBus for async support
    bus = AsyncBus(can_bus)

    # Create async server
    server = AsyncServer(bus)

    # Start the bus reader task
    bus_reader_task = asyncio.create_task(server.run())

    async def handle_client(
        reader: asyncio.StreamReader, writer: asyncio.StreamWriter
    ) -> None:
        """Handle a new client connection.

        Args:
            reader: Asyncio stream reader for client
            writer: Asyncio stream writer for client

        """
        transport = AsyncSocketTransport(reader, writer)
        addr = transport.addr
        logger.info("Client connected from %s", addr)

        try:
            # Let the server handle the transport
            await server.handle_transport(transport)
        except Exception:
            logger.exception("Error handling client %s", addr)
        finally:
            await transport.close()
            logger.info("Client disconnected from %s", addr)

    # Setup asyncio TCP server
    tcp_server = await asyncio.start_server(handle_client, args.host, args.port)

    if tcp_server.sockets:
        addr = tcp_server.sockets[0].getsockname()
    else:
        addr = ("", args.port)
    host_display = addr[0] or "all interfaces"
    logger.info("NetCAN server listening on %s:%d", host_display, addr[1])
    logger.info("CAN bus: %s on channel %s", args.bus, args.channel)

    # Handle shutdown signals
    stop_event = asyncio.Event()

    def signal_handler(sig: int, frame: FrameType | None) -> None:  # noqa: ARG001
        logger.info("Received signal %s, shutting down...", sig)
        stop_event.set()

    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    try:
        async with tcp_server:
            # Wait for shutdown signal
            await stop_event.wait()
    finally:
        logger.info("Shutting down server...")
        # Stop accepting new connections
        tcp_server.close()
        await tcp_server.wait_closed()

        # Stop the server
        server.stop()

        # Cancel and wait for bus reader task
        bus_reader_task.cancel()
        with contextlib.suppress(asyncio.CancelledError):
            await bus_reader_task

        # Shutdown CAN bus
        can_bus.shutdown()
        logger.info("Server shutdown complete")


def main() -> None:
    """Entry point for NetCAN server."""
    with contextlib.suppress(KeyboardInterrupt):
        asyncio.run(async_main())


if __name__ == "__main__":
    main()
