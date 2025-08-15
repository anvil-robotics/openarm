"""Mock implementation of Bus for testing."""

from typing import List
import can

from . import BusProtocol


class BusMock:
    """Mock CAN bus for testing that tracks expected send/recv operations."""

    def __init__(self) -> None:
        """Initialize the mock bus."""
        self.expected_sends: List[can.Message] = []
        self.expected_recvs: List[can.Message] = []
        self.send_index = 0
        self.recv_index = 0

    def expect_send(self, msg: can.Message) -> None:
        """Set expectation for a message to be sent.
        
        Args:
            msg: The CAN message expected to be sent.
        """
        self.expected_sends.append(msg)

    def expect_recv(self, msg: can.Message) -> None:
        """Set expectation for a message to be received.
        
        Args:
            msg: The CAN message that should be returned by recv().
        """
        self.expected_recvs.append(msg)

    def send(self, msg: can.Message, _timeout: float | None = None) -> None:
        """Mock send that verifies the message matches expectations.
        
        Args:
            msg: The CAN message to send.
            _timeout: Optional send timeout (ignored in mock).
            
        Raises:
            AssertionError: If message doesn't match expected.
        """
        if self.send_index >= len(self.expected_sends):
            raise AssertionError(f"Unexpected send call. Got: {msg}")
        
        expected = self.expected_sends[self.send_index]
        assert msg.arbitration_id == expected.arbitration_id, \
            f"Send arbitration_id mismatch. Expected: {expected.arbitration_id}, Got: {msg.arbitration_id}"
        assert msg.data == expected.data, \
            f"Send data mismatch. Expected: {expected.data}, Got: {msg.data}"
        
        self.send_index += 1

    def recv(self, arbitration_id: int, _timeout: float | None = None) -> can.Message | None:
        """Mock recv that returns pre-configured messages.
        
        Args:
            arbitration_id: The arbitration ID to filter for.
            _timeout: Optional receive timeout (ignored in mock).
            
        Returns:
            The next expected message with matching arbitration_id, or None.
        """
        if self.recv_index >= len(self.expected_recvs):
            return None
            
        expected = self.expected_recvs[self.recv_index]
        if expected.arbitration_id == arbitration_id:
            self.recv_index += 1
            return expected
            
        return None

    def verify_all_expectations(self) -> None:
        """Verify all expected send/recv operations were called.
        
        Raises:
            AssertionError: If not all expectations were met.
        """
        assert self.send_index == len(self.expected_sends), \
            f"Not all expected sends called. Expected: {len(self.expected_sends)}, Called: {self.send_index}"
        assert self.recv_index == len(self.expected_recvs), \
            f"Not all expected recvs called. Expected: {len(self.expected_recvs)}, Called: {self.recv_index}"