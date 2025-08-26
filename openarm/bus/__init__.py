"""CAN bus message multiplexer for filtering messages by arbitration ID."""

from asyncio import Queue, QueueFull, QueueShutDown, get_running_loop
from collections import defaultdict
from collections.abc import Coroutine
from typing import Protocol

import can


class AsyncBusProto(Protocol):
    """Protocol for async CAN bus."""

    def send(self, msg: can.Message, timeout: float | None = None) -> None:
        """Send a CAN message."""
        ...

    async def recv(self) -> can.Message:
        """Receive a CAN message."""
        ...

    def shutdown(self) -> None:
        """Shutdown the bus and clean up resources."""
        ...


class AsyncBusMuxProto(Protocol):
    """Protocol for async CAN bus with multiplexing by arbitration ID."""

    def send(self, msg: can.Message, timeout: float | None = None) -> None:
        """Send a CAN message."""
        ...

    def recv(self, arbitration_id: int) -> Coroutine[any, any, can.Message]:
        """Receive a CAN message with specific arbitration ID."""
        ...

    def shutdown(self) -> None:
        """Shutdown the bus and clean up resources."""
        ...


class AsyncBusMux:
    """CAN bus wrapper with multiplexing by arbitration ID."""

    def __init__(self, bus: can.BusABC) -> None:
        """Initialize the bus multiplexer.

        Args:
            bus: The underlying CAN bus interface.

        """
        self.bus = bus
        self._queues = defaultdict[int, Queue[can.Message]](Queue)
        self._shutdown = False
        loop = get_running_loop()
        loop.add_reader(bus.fileno(), self._onreadable)

    def _onreadable(self) -> None:
        try:
            msg = self.bus.recv()
            if msg is None:
                # This shouldn't happen when called by the event loop
                err_msg = "Unexpected None from bus.recv() in _onreadable"
                raise RuntimeError(err_msg)

            # defaultdict automatically creates queue if it doesn't exist
            self._queues[msg.arbitration_id].put_nowait(msg)

        except can.CanOperationError:
            for queue in self._queues.values():
                queue.shutdown()
            self._shutdown = True
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
        if self._shutdown:
            raise QueueShutDown

        # defaultdict automatically creates queue if it doesn't exist
        return self._queues[arbitration_id].get()

    def shutdown(self) -> None:
        """Shutdown the bus multiplexer and clean up resources."""
        for queue in self._queues.values():
            queue.shutdown()
        self._shutdown = True
        loop = get_running_loop()
        loop.remove_reader(self.bus.fileno())


class AsyncBus:
    """Async CAN bus wrapper that receives all messages."""

    def __init__(self, bus: can.BusABC) -> None:
        """Initialize the broadcast bus.

        Args:
            bus: The underlying CAN bus interface.

        """
        self.bus = bus
        self._queue: Queue[can.Message] = Queue()
        loop = get_running_loop()
        loop.add_reader(bus.fileno(), self._onreadable)

    def _onreadable(self) -> None:
        """Handle readable event from the CAN bus."""
        try:
            msg = self.bus.recv()
            if msg is None:
                # This shouldn't happen when called by the event loop
                err_msg = "Unexpected None from bus.recv() in _onreadable"
                raise RuntimeError(err_msg)

            # Put message in the single queue for all messages
            self._queue.put_nowait(msg)

        except can.CanOperationError:
            self._queue.shutdown()
            loop = get_running_loop()
            loop.remove_reader(self.bus.fileno())
        except QueueFull:
            # drop message if queue is full
            pass

    def send(self, msg: can.Message, timeout: float | None = None) -> None:
        """Send a CAN message.

        Args:
            msg: The CAN message to send.
            timeout: Optional send timeout in seconds.

        """
        self.bus.send(msg, timeout)

    async def recv(self) -> can.Message:
        """Receive the next CAN message regardless of arbitration ID.

        Returns:
            The next received CAN message.

        """
        return await self._queue.get()

    def shutdown(self) -> None:
        """Shutdown the async bus and clean up resources."""
        self._queue.shutdown()
        loop = get_running_loop()
        loop.remove_reader(self.bus.fileno())
