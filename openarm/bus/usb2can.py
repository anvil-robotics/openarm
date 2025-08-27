"""USB2CAN adapter implementation for python-can."""

import can
import serial


class USB2CAN(can.BusABC):
    """USB2CAN adapter that implements the CAN BusABC interface."""

    FRAME_LENGTH = 30
    HEADER = 0xAA55  # Note: 0x55 0xAA in little-endian
    HEADER_BYTE_1 = 0x55
    HEADER_BYTE_2 = 0xAA
    TAIL = 0x55

    def __init__(
        self,
        channel: str,
        baudrate: int = 921600,
        timeout: float | None = 0.1,
        can_filters: can.typechecking.CanFilters | None = None,
        **kwargs: object,
    ) -> None:
        """Initialize USB2CAN adapter.

        Args:
            channel: Serial port device path (e.g., '/dev/ttyUSB0')
            baudrate: Serial baudrate (default: 921600)
            timeout: Default timeout for operations
            can_filters: See :meth:`~can.BusABC.set_filters` for details
            **kwargs: Additional arguments passed to parent class

        """
        self.channel_info = f"USB2CAN on {channel}"
        self.serial_port = serial.Serial(
            port=channel,
            baudrate=baudrate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=timeout,
        )

        self.data_buffer = b""

        # Call parent constructor last as per BusABC documentation
        super().__init__(channel=channel, can_filters=can_filters, **kwargs)

    def send(self, msg: can.Message, timeout: float | None = None) -> None:  # noqa: ARG002
        """Send a CAN message via USB2CAN.

        Args:
            msg: CAN message to send
            timeout: Send timeout (not used for serial)

        """
        if self._is_shutdown:
            msg_text = "Bus is shutdown"
            raise can.CanError(msg_text)

        frame = self._build_frame(msg)
        self.serial_port.write(frame)

    def _recv_internal(self, timeout: float | None) -> tuple[can.Message | None, bool]:
        """Read a message from the bus and tell whether it was filtered.

        This is the preferred method to override instead of recv() according to BusABC.

        Args:
            timeout: Receive timeout in seconds

        Returns:
            Tuple of (message, already_filtered) where already_filtered is False
            since we don't do hardware filtering

        """
        if self._is_shutdown:
            msg_text = "Bus is shutdown"
            raise can.CanError(msg_text)

        if timeout is not None:
            old_timeout = self.serial_port.timeout
            self.serial_port.timeout = timeout

        try:
            # Read available data and append to buffer
            new_data = self.serial_port.read(self.serial_port.in_waiting or 1)
            if not new_data:
                return None, False

            self.data_buffer += new_data

            # Try to extract a complete packet
            packet = self._extract_packet()
            if packet:
                msg = self._parse_packet(packet)
                return msg, False  # We don't do hardware filtering

            return None, False

        finally:
            if timeout is not None:
                self.serial_port.timeout = old_timeout

    def shutdown(self) -> None:
        """Shutdown the USB2CAN connection."""
        if not self._is_shutdown:
            if self.serial_port and self.serial_port.is_open:
                self.serial_port.close()
            # Call parent shutdown to handle _is_shutdown flag and periodic tasks
            super().shutdown()

    def fileno(self) -> int:
        """Return file descriptor for select() compatibility.

        Returns:
            File descriptor of the serial port

        """
        if self._is_shutdown:
            msg_text = "Bus is shutdown"
            raise can.CanError(msg_text)
        return self.serial_port.fileno()

    def _build_frame(self, msg: can.Message) -> bytes:
        """Build USB2CAN frame from CAN message.

        Args:
            msg: CAN message

        Returns:
            30-byte frame for USB2CAN

        """
        frame = bytearray(self.FRAME_LENGTH)

        # Header
        frame[0] = 0x55
        frame[1] = 0xAA

        # Frame type and command (based on original protocol)
        frame[2] = 0x1E  # Frame length indicator
        frame[3] = 0x03  # Command type
        frame[4] = 0x01  # Sub-command

        # CAN ID (little-endian, 4 bytes starting at offset 13)
        can_id = msg.arbitration_id
        if msg.is_extended_id:
            can_id |= 0x80000000  # Set extended ID flag

        frame[13] = can_id & 0xFF
        frame[14] = (can_id >> 8) & 0xFF
        frame[15] = (can_id >> 16) & 0xFF
        frame[16] = (can_id >> 24) & 0xFF

        # Data length
        data_len = len(msg.data) if msg.data else 0
        frame[19] = min(data_len, 8)  # CAN data is max 8 bytes

        # CAN data (starting at offset 21)
        if msg.data:
            for i, byte in enumerate(msg.data[:8]):
                frame[21 + i] = byte

        # Tail
        frame[29] = self.TAIL

        return bytes(frame)

    def _extract_packet(self) -> bytes | None:
        """Extract a complete packet from the buffer.

        Returns:
            Complete packet if found, None otherwise

        """
        while len(self.data_buffer) >= self.FRAME_LENGTH:
            # Look for header
            header_pos = self.data_buffer.find(b"\x55\xaa")

            if header_pos == -1:
                # No header found, clear buffer
                self.data_buffer = b""
                return None

            # Remove data before header
            if header_pos > 0:
                self.data_buffer = self.data_buffer[header_pos:]

            # Check if we have a complete frame
            if len(self.data_buffer) >= self.FRAME_LENGTH:
                # Verify tail
                if self.data_buffer[self.FRAME_LENGTH - 1] == self.TAIL:
                    # Extract packet
                    packet = self.data_buffer[: self.FRAME_LENGTH]
                    self.data_buffer = self.data_buffer[self.FRAME_LENGTH :]
                    return packet
                # Invalid frame, skip header and continue
                self.data_buffer = self.data_buffer[2:]
            else:
                # Not enough data yet
                break

        return None

    def _parse_packet(self, packet: bytes) -> can.Message | None:
        """Parse USB2CAN packet into CAN message.

        Args:
            packet: 30-byte USB2CAN packet

        Returns:
            CAN message or None if invalid packet

        """
        if len(packet) != self.FRAME_LENGTH:
            return None

        # Verify header and tail
        if (
            packet[0] != self.HEADER_BYTE_1
            or packet[1] != self.HEADER_BYTE_2
            or packet[29] != self.TAIL
        ):
            return None

        # Extract CAN ID (little-endian from offset 13)
        can_id = (
            packet[13] | (packet[14] << 8) | (packet[15] << 16) | (packet[16] << 24)
        )

        # Check for extended ID
        is_extended = bool(can_id & 0x80000000)
        if is_extended:
            can_id &= 0x1FFFFFFF  # Clear extended flag bit

        # Extract data length
        data_len = min(packet[19], 8)

        # Extract data
        data = bytes(packet[21 : 21 + data_len])

        return can.Message(arbitration_id=can_id, data=data, is_extended_id=is_extended)
