"""Motor detection utilities for Damiao motors.

This module provides functionality to scan and detect Damiao motors on the CAN bus.
"""

import struct
import time
from collections.abc import Generator, Iterable
from dataclasses import dataclass

import can

from .encoding import RegisterAddress, encode_read_register


@dataclass
class MotorInfo:
    """Information about a detected motor."""

    slave_id: int
    master_id: int


def detect_motors(
    bus: can.BusABC,
    slave_ids: Iterable[int],
    timeout: float = 0.05,
) -> Generator[MotorInfo, None, None]:
    """Detect motors by sending register read requests and collecting responses.

    This function sends register read requests to all specified slave IDs
    simultaneously, then collects responses to identify which motors are present.

    Args:
        bus: CAN bus instance for communication
        slave_ids: Iterable of slave IDs to scan
        timeout: Total timeout for detection process in seconds (default: 0.05)

    Yields:
        MotorInfo objects as they are detected

    Example:
        for motor_info in detect_motors(bus, range(1, 11)):
            print(f"Found motor: {motor_info}")

    """
    # Send register read request to all slave IDs at once
    # Read ESC_ID register (0x08) which contains the slave ID
    for slave_id in slave_ids:
        encode_read_register(bus, slave_id, RegisterAddress.ESC_ID)

    # Collect all responses until timeout
    end_time = time.time() + timeout

    while time.time() < end_time:
        remaining_time = end_time - time.time()
        if remaining_time <= 0:
            break

        # Read one message at a time
        message = bus.recv(timeout=remaining_time)

        if message is None:
            # Timeout, no more messages
            break

        if message.is_error_frame:
            continue

        # The arbitration_id IS the master_id!
        master_id = message.arbitration_id

        try:
            # Decode register response to get slave_id from the value
            # Format: '<HBBI' = slave_id(H) + cmd(B) + reg_id(B) + value(I)
            _, _, _, slave_id = struct.unpack("<HBBI", message.data[:8])

            yield MotorInfo(
                slave_id=slave_id,
                master_id=master_id,
            )
        except struct.error:
            # Not a valid register response, ignore
            continue
