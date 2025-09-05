"""CAN bus message multiplexer for filtering messages by arbitration ID."""

from collections import defaultdict
from time import time

import can


class Bus:
    """CAN bus wrapper."""

    def __init__(self, bus: can.BusABC) -> None:
        """Initialize the bus multiplexer.

        Args:
            bus: The underlying CAN bus interface.

        """
        self.bus = bus
        self.lookup = defaultdict[int, list[can.Message]](list)

    def send(self, msg: can.Message, timeout: float | None = None) -> None:
        """Send a CAN message.

        Args:
            msg: The CAN message to send.
            timeout: Optional send timeout in seconds.

        """
        self.bus.send(msg, timeout)

    def recv(
        self, arbitration_id: int, timeout: float | None = None
    ) -> can.Message | None:
        """Receive a CAN message with the specified arbitration ID.

        Messages with other arbitration IDs are queued for later retrieval.

        Args:
            arbitration_id: The arbitration ID to filter for.
            timeout: Optional receive timeout in seconds. None means wait indefinitely.

        Returns:
            The received CAN message, or None if timeout occurred.

        """
        queue = self.lookup[arbitration_id]
        if len(queue) > 0:
            return queue.pop(0)

        if timeout is None:
            while True:
                msg = self.bus.recv()
                if msg.is_error_frame or not msg.is_rx:
                    continue
                if msg.arbitration_id == arbitration_id:
                    return msg
                queue.append(msg)
        else:
            end = time() + timeout
            while timeout > 0:
                msg = self.bus.recv(timeout)
                if msg is None:
                    return None
                if msg.is_error_frame or not msg.is_rx:
                    continue
                if msg.arbitration_id == arbitration_id:
                    return msg
                queue.append(msg)
                timeout = end - time()

        return None
