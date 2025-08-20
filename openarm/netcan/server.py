"""NetCAN Server implementation."""

from can import BusABC

from .transport import Transport


class Server:
    """NetCAN server that handles CAN bus communication with multiple clients."""

    def __init__(self, bus: BusABC) -> None:
        """Initialize NetCAN server.

        Args:
            bus: CAN bus interface to bridge with network clients

        """
        self.trans_map: dict[int, Transport] = {}
        self.bus = bus

    def attach(self, trans: Transport) -> None:
        """Attach a transport connection to the server.

        Args:
            trans: Transport connection to attach

        """
        fd = trans.fileno()
        self.trans_map[fd] = trans

    def run(self, fd: int) -> bool:
        """Process incoming data from a file descriptor.

        Handles bidirectional message routing between CAN bus and network clients.
        Messages from the bus are broadcast to all connected clients.
        Messages from clients are sent to the CAN bus.

        Args:
            fd: File descriptor with pending data

        Returns:
            True if connection remains active, False if connection closed

        """
        # Handle messages from CAN bus - broadcast to all clients
        if fd == self.bus.fileno():
            msg = self.bus.recv()
            for [_, trans] in self.trans_map.items():
                trans.encode(msg)
            return True

        # Handle messages from network client - forward to CAN bus
        trans = self.trans_map[fd]
        if trans:
            msg = trans.decode()
            if msg is None:
                # Client disconnected
                del self.trans_map[fd]
                return False

            # Forward message to CAN bus
            self.bus.send(msg)
            return True

        return True
