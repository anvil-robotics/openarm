"""CAN bus message multiplexer for filtering messages by arbitration ID."""

from asyncio import Queue, QueueShutDown, get_running_loop

import can


class Bus:
    """CAN bus wrapper."""

    def __init__(self, bus: can.BusABC) -> None:
        """Initialize the bus multiplexer.

        Args:
            bus: The underlying CAN bus interface.

        """
        self.bus = bus
        self.lookup: dict[int, list[can.Message]] = {}
        self.queue = Queue[can.Message]()
        loop = get_running_loop()
        loop.add_reader(bus.fileno(), self._onreadable)

    def _onreadable(self) -> None:
        try:
            msg = self.bus.recv()
            if msg is None:
                loop = get_running_loop()
                loop.remove_reader(self.bus.fileno())
                self.queue.shutdown()
            else:
                self.queue.put(msg)
        except can.CanOperationError:
            loop = get_running_loop()
            loop.remove_reader(self.bus.fileno())
            self.queue.shutdown()

    def send(self, msg: can.Message, timeout: float | None = None) -> None:
        """Send a CAN message.

        Args:
            msg: The CAN message to send.
            timeout: Optional send timeout in seconds.

        """
        self.bus.send(msg, timeout)

    async def recv(self, arbitration_id: int) -> can.Message | None:
        """Receive a CAN message with the specified arbitration ID.

        Messages with other arbitration IDs are queued for later retrieval.

        Args:
            arbitration_id: The arbitration ID to filter for.

        Returns:
            The received CAN message, or None if bus closed.

        """
        queue = self.lookup[arbitration_id]
        if queue and len(queue) > 0:
            return queue.pop(0)

        try:
            while True:
                msg = await self.queue.get()
                if msg.arbitration_id == arbitration_id:
                    return msg
                queue = self.lookup[msg.arbitration_id]
                if queue:
                    queue.append(msg)
                else:
                    self.lookup[msg.arbitration_id] = [msg]
        except QueueShutDown:
            return None
