"""NetCAN Server implementation."""

from can import BusABC

from .transport import Transport


class Server:
    """CAN Server."""

    def __init__(self, bus: BusABC) -> None:
        """Initialize Server."""
        self.trans_map: dict[int, Transport] = {}
        self.bus = bus

    def attach(self, trans: Transport) -> None:
        """Attach transport."""
        fd = trans.fileno()
        self.trans_map[fd] = trans

    def run(self, fd: int) -> bool:
        """Server run."""
        # message from bus
        if fd == self.bus.fileno():
            msg = self.bus.recv()
            for [_, trans] in self.trans_map.items():
                trans.encode(msg)
            return True

        # get message from transport
        trans = self.trans_map[fd]
        if trans:
            msg = trans.decode()
            if msg is None:
                # connection closed
                del self.trans_map[fd]
                return False

            # send to bus
            self.bus.send(msg)
            return True

        return True
