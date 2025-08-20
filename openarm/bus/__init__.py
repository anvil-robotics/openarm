"""CAN bus message multiplexer for filtering messages by arbitration ID."""

from asyncio import Queue, QueueFull, get_running_loop
from collections.abc import Coroutine

import can


class Bus:
    """CAN bus wrapper."""

    def __init__(self, bus: can.BusABC) -> None:
        """Initialize the bus multiplexer.

        Args:
            bus: The underlying CAN bus interface.

        """
        self.bus = bus
        self.queues: dict[int, Queue[can.Message]] = {}
        loop = get_running_loop()
        loop.add_reader(bus.fileno(), self._onreadable)

    def _onreadable(self) -> None:
        try:
            msg = self.bus.recv()
            if msg is None:
                # This shouldn't happen when called by the event loop
                err_msg = "Unexpected None from bus.recv() in _onreadable"
                raise RuntimeError(err_msg)

            if msg.arbitration_id in self.queues:
                self.queues[msg.arbitration_id].put_nowait(msg)
            else:
                queue = Queue[can.Message]()
                queue.put_nowait(msg)
                self.queues[msg.arbitration_id] = queue

        except can.CanOperationError:
            loop = get_running_loop()
            loop.remove_reader(self.bus.fileno())
        except QueueFull:
            # drop message
            pass

    def send(self, msg: can.Message, timeout: float | None = None) -> None:
        """Send a CAN message.

        Args:
            msg: The CAN message to send.
            timeout: Optional send timeout in seconds.

        """
        self.bus.send(msg, timeout)

    def recv(self, arbitration_id: int) -> Coroutine[any, any, can.Message]:
        """Receive a CAN message with the specified arbitration ID.

        Messages with other arbitration IDs are queued separately.

        Args:
            arbitration_id: The arbitration ID to filter for.

        Returns:
            The received CAN message.

        """
        if arbitration_id in self.queues:
            return self.queues[arbitration_id].get()

        queue = Queue[can.Message]()
        self.queues[arbitration_id] = queue
        return queue.get()
