"""Non-blocking serial button reader for Arduino button box."""

from __future__ import annotations

import logging
import threading

import serial

logger = logging.getLogger(__name__)


class ButtonReader:
    """Reads button states from an Arduino over serial in a background thread."""

    def __init__(
        self,
        port: str = "/dev/ttyACM0",
        baudrate: int = 115200,
        n_buttons: int = 4,
    ) -> None:
        self._port = port
        self._baudrate = baudrate
        self._n = n_buttons
        self._lock = threading.Lock()
        self._state = [0] * n_buttons
        self._thread = threading.Thread(target=self._reader_loop, daemon=True)
        self._thread.start()

    def _reader_loop(self) -> None:
        try:
            ser = serial.Serial(self._port, self._baudrate, timeout=0.05)
            while True:
                line = ser.readline().decode("utf-8", errors="ignore").strip()
                if not line:
                    continue
                parts = line.split()
                if len(parts) == self._n:
                    try:
                        vals = [int(p) for p in parts]
                        with self._lock:
                            self._state[:] = vals
                    except ValueError:
                        pass
        except Exception as e:
            logger.warning("Serial reader failed: %s", e)

    @property
    def state(self) -> list[int]:
        """Return a snapshot of the current button states."""
        with self._lock:
            return self._state[:]
