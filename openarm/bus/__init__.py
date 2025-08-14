import can
from time import time

class Bus:
    def __init__(self, bus: can.BusABC):
        self.bus = bus
        self.lookup : dict[int, list[can.Message]] = {}

    def send(self, msg: can.Message, timeout: float | None):
        self.bus.send(msg, timeout)
    
    def recv(self, arbitration_id: int, timeout: float | None) -> can.Message:
        list = self.lookup[arbitration_id]
        if list and len(list) > 0:
            return list.pop(0)
        
        if timeout is None:
            while True:
                msg = self.bus.recv()
                if msg.arbitration_id == arbitration_id:
                    return msg
                list = self.lookup[msg.arbitration_id]
                if list:
                    list.append(msg)
                else:
                    self.lookup[msg.arbitration_id] = [msg]
        else:
            end = time() + timeout
            while timeout > 0:
                msg = self.bus.recv(timeout)
                if msg.arbitration_id == arbitration_id:
                    return msg
                list = self.lookup[msg.arbitration_id]
                if list:
                    list.append(msg)
                else:
                    self.lookup[msg.arbitration_id] = [msg]
                timeout = end - time()
