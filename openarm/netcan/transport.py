"""NetCAN transport implementation."""

import json
from socket import socket
from time import time
from typing import Generic, Protocol, TypeVar

from can import Message

T = TypeVar("T")


class Transport(Protocol, Generic[T]):
    """Generic CAN transport."""

    def encode(self, msg: Message) -> None:
        """Encode CAN message and send."""
        ...

    def decode(self) -> Message | None:
        """Decode and return a CAN message."""
        ...

    def fileno() -> int:
        """Return the integer file descriptor."""
        ...


class SocketTransport:
    """CAN transport for Socket."""

    def __init__(self, sock: socket) -> None:
        """Initiate Socket Transport."""
        self.sock = sock
        self.r = self.sock.makefile("r")

    def encode(self, message: Message) -> None:
        """Encode CAN message and send to stored writer."""
        payload = {
            "arbitration_id": message.arbitration_id,
            "data": message.data.hex() if message.data else "",
            "timestamp": message.timestamp if message.timestamp else time(),
            "is_extended_id": message.is_extended_id,
        }

        data = json.dumps(payload) + "\n"
        self.sock.sendall(data.encode("utf-8"))

    def decode(self) -> Message | None:
        """Decode and return a CAN message from stored reader."""
        line = self.r.readline()
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

    def fileno(self) -> int:
        """Return the integer file descriptor of the socket."""
        return self.r.fileno()
