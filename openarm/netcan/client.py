"""NetCAN Client implementation."""

import json
import socket
from collections.abc import Iterator
from time import time

from can import BusABC, Message


class Client(BusABC):
    """NetCAN client that implements the CAN BusABC interface."""

    def __init__(self, host: str, port: int = 11898) -> None:
        """Initialize NetCAN client.

        Args:
            host: Server hostname or IP address
            port: Server port number

        """
        super().__init__(channel=f"{host}:{port}")
        self.host = host
        self.port = port
        self.sock: socket.socket | None = None
        self.file_reader = None
        self._is_connected = False

    def connect(self) -> None:
        """Connect to NetCAN server."""
        if self._is_connected:
            return

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.connect((self.host, self.port))
        self.file_reader = self.sock.makefile("r")
        self._is_connected = True

    def disconnect(self) -> None:
        """Disconnect from NetCAN server."""
        if not self._is_connected:
            return

        if self.file_reader:
            self.file_reader.close()
            self.file_reader = None

        if self.sock:
            self.sock.close()
            self.sock = None

        self._is_connected = False

    def send(self, msg: Message, timeout: float | None = None) -> None:  # noqa: ARG002
        """Send a CAN message to the server.

        Args:
            msg: CAN message to send
            timeout: Send timeout (unused in socket implementation)

        """
        if not self._is_connected:
            self.connect()

        if not self.sock:
            msg = "Not connected to server"
            raise RuntimeError(msg)

        payload = {
            "arbitration_id": msg.arbitration_id,
            "data": msg.data.hex() if msg.data else "",
            "timestamp": msg.timestamp if msg.timestamp else time(),
            "is_extended_id": msg.is_extended_id,
        }

        data = json.dumps(payload) + "\n"
        self.sock.sendall(data.encode("utf-8"))

    def recv(self, timeout: float | None = None) -> Message | None:
        """Receive a CAN message from the server.

        Args:
            timeout: Receive timeout

        Returns:
            Received CAN message or None if timeout/error

        """
        if not self._is_connected:
            self.connect()

        if not self.file_reader:
            msg = "Not connected to server"
            raise RuntimeError(msg)

        if timeout is not None:
            self.sock.settimeout(timeout)

        try:
            line = self.file_reader.readline()
            if not line:
                return None

            message_data = line.strip()
            if not message_data:
                return None

            payload = json.loads(message_data)

            return Message(
                arbitration_id=payload["arbitration_id"],
                data=bytes.fromhex(payload["data"]) if payload["data"] else b"",
                timestamp=payload.get("timestamp"),
                is_extended_id=payload.get("is_extended_id", False),
            )

        except TimeoutError:
            return None
        except (json.JSONDecodeError, KeyError, ValueError):
            return None
        finally:
            if timeout is not None:
                self.sock.settimeout(None)

    def __iter__(self) -> Iterator[Message]:
        """Iterate over received messages."""
        while True:
            msg = self.recv()
            if msg is not None:
                yield msg

    def fileno(self) -> int:
        """Return socket file descriptor."""
        if not self._is_connected:
            self.connect()

        if not self.sock:
            msg = "Not connected to server"
            raise RuntimeError(msg)

        return self.sock.fileno()

    def shutdown(self) -> None:
        """Shutdown the client connection."""
        self.disconnect()

    @property
    def is_connected(self) -> bool:
        """Check if client is connected to server."""
        return self._is_connected

    def __enter__(self) -> "Client":
        """Context manager entry."""
        self.connect()
        return self

    def __exit__(
        self,
        exc_type: type[BaseException] | None,
        exc_val: BaseException | None,
        exc_tb: object,
    ) -> None:
        """Context manager exit."""
        self.disconnect()
