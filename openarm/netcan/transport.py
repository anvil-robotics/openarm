"""NetCAN transport implementation."""

import asyncio
import json
from time import time
from typing import Protocol

from can import Message


class Transport(Protocol):
    """Generic CAN transport protocol."""

    async def encode(self, msg: Message) -> None:
        """Encode CAN message and send."""
        ...

    async def decode(self) -> Message | None:
        """Decode and return a CAN message."""
        ...


class AsyncSocketTransport:
    """Async CAN transport for Socket using asyncio streams."""

    def __init__(
        self, reader: asyncio.StreamReader, writer: asyncio.StreamWriter
    ) -> None:
        """Initialize Async Socket Transport.

        Args:
            reader: Asyncio stream reader for incoming data
            writer: Asyncio stream writer for outgoing data

        """
        self.reader = reader
        self.writer = writer
        self.addr = writer.get_extra_info("peername")

    async def encode(self, message: Message) -> None:
        """Encode CAN message and send to client.

        Args:
            message: CAN message to encode and send

        """
        try:
            payload = {
                "arbitration_id": message.arbitration_id,
                "data": message.data.hex() if message.data else "",
                "timestamp": message.timestamp if message.timestamp else time(),
                "is_extended_id": message.is_extended_id,
            }

            data = json.dumps(payload) + "\n"
            self.writer.write(data.encode("utf-8"))
            await self.writer.drain()
        except (ConnectionError, OSError):
            # Connection closed, will be handled by caller
            pass

    async def decode(self) -> Message | None:
        """Decode and return a CAN message from client.

        Returns:
            Decoded CAN message or None if connection closed/error

        """
        try:
            line = await self.reader.readline()
            if not line:
                return None

            message_data = line.decode("utf-8").strip()
            if not message_data:
                return None

            payload = json.loads(message_data)

            return Message(
                arbitration_id=payload["arbitration_id"],
                data=bytes.fromhex(payload["data"]) if payload["data"] else b"",
                timestamp=payload.get("timestamp"),
                is_extended_id=payload.get("is_extended_id", False),
            )
        except (json.JSONDecodeError, KeyError, ValueError, UnicodeDecodeError):
            # Invalid message format
            return None
        except (ConnectionError, OSError):
            # Connection closed
            return None

    async def close(self) -> None:
        """Close the transport connection."""
        try:
            self.writer.close()
            await self.writer.wait_closed()
        except (ConnectionError, OSError):
            pass
