"""NetCAN Server implementation."""

from select import select

from can import BusABC

from .transport import Transport


class Server:
    """CAN Server."""

    def __init__(self, bus: BusABC) -> None:
        """Initialize Server."""
        self.reads = [bus.fileno()]
        self.trans_map: dict[int, Transport] = {}
        self.trans_list: list[Transport] = []
        self.bus = bus

    def attach(self, trans: Transport) -> None:
        """Attach transport."""
        fd = trans.fileno()
        self.reads.append(fd)
        self.trans_map[fd] = trans
        self.trans_list.append(trans)

    async def loop(self) -> None:
        """Server loop."""
        while True:
            reads, _, _ = select(self.reads, [], [], None)
            for fd in reads:
                # message from bus
                if fd == self.bus.fileno():
                    msg = self.bus.recv()
                    for trans in self.trans_list:
                        trans.encode(msg)
                    continue

                # get message from transport
                trans = self.trans_map[fd]
                msg = trans.decode()
                if msg is None:
                    # connection closed
                    self.reads.remove(fd)
                    del self.trans_list[fd]
                    self.trans_list.remove(trans)
                    continue

                # send to bus
                self.bus.send(msg)
