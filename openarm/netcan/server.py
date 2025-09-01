"""NetCAN Server implementation."""

import asyncio
import logging

from can import Message

from openarm.bus import AsyncBus

from .transport import AsyncSocketTransport


class AsyncServer:
    """Async NetCAN server that handles CAN bus communication with multiple clients."""

    def __init__(self, bus: AsyncBus) -> None:
        """Initialize async NetCAN server.

        Args:
            bus: Async CAN bus interface to bridge with network clients

        """
        self.bus = bus
        self.clients: set[AsyncSocketTransport] = set()
        self.logger = logging.getLogger(__name__)
        self.running = False

    async def handle_transport(self, transport: AsyncSocketTransport) -> None:
        """Handle a transport connection.

        Args:
            transport: AsyncSocketTransport instance to handle

        """
        self.clients.add(transport)

        try:
            # Read messages from transport and forward to CAN bus
            while True:
                msg = await transport.decode()
                if msg is None:
                    # Transport disconnected
                    break

                # Forward message directly to CAN bus
                self.bus.send(msg)

        finally:
            # Remove transport from clients set
            self.clients.discard(transport)

    async def broadcast(self, msg: Message) -> None:
        """Broadcast a CAN message to all connected clients.

        Args:
            msg: CAN message to broadcast

        """
        # Send to all clients concurrently
        if self.clients:
            # Create tasks for all client sends
            tasks = [transport.encode(msg) for transport in self.clients]
            # Wait for all to complete (or fail)
            await asyncio.gather(*tasks, return_exceptions=True)

    async def run(self) -> None:
        """Run the main server loop that reads from CAN bus and broadcasts."""
        self.running = True

        try:
            # Continuously receive messages
            while self.running:
                msg = await self.bus.recv()
                await self.broadcast(msg)
        except Exception:
            self.logger.exception("Error in bus reader loop")

    def stop(self) -> None:
        """Stop the server."""
        self.running = False
