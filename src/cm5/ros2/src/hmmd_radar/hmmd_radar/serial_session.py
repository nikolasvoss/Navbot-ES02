from dataclasses import dataclass
import math
import time

import serial

from .protocol import DEBUG_MODE_COMMAND, FrameParser


MAX_READ_BYTES = 4096
WRITE_TIMEOUT_SECONDS = 0.5
RECONNECT_DELAY_START = 0.25
RECONNECT_DELAY_MAX = 5.0


@dataclass(frozen=True)
class SessionStatus:
    connected: bool
    stale: bool
    last_frame_age_seconds: float | None


class SerialSession:
    def __init__(self, port: str, baud_rate: int, serial_factory, stale_after: float = 1.0):
        if not isinstance(port, str):
            raise TypeError("port must be a string")
        if not port:
            raise ValueError("port is required")
        if isinstance(baud_rate, bool) or not isinstance(baud_rate, int) or baud_rate <= 0:
            raise ValueError("baud_rate must be a positive integer")
        if (
            isinstance(stale_after, bool)
            or not isinstance(stale_after, (int, float))
            or not math.isfinite(stale_after)
            or stale_after <= 0
        ):
            raise ValueError("stale_after must be finite and positive")
        self.port = port
        self.baud_rate = baud_rate
        self._serial_factory = serial_factory
        self.stale_after = stale_after
        self.parser = FrameParser()
        self._serial = None
        self.last_frame_time: float | None = None
        self.frames_received = 0
        self.reconnects = 0
        self.input_backlog_overflows = 0
        self.last_io_error = ""
        self._next_open_time = 0.0
        self._retry_delay = RECONNECT_DELAY_START
        self._partial_started_at: float | None = None

    @property
    def connected(self) -> bool:
        return self._serial is not None and bool(self._serial.is_open)

    def status(self, now: float | None = None) -> SessionStatus:
        now = time.monotonic() if now is None else now
        age = None if self.last_frame_time is None else max(0.0, now - self.last_frame_time)
        return SessionStatus(self.connected, age is None or age > self.stale_after, age)

    def poll(self, now: float | None = None):
        now = time.monotonic() if now is None else now
        if self._partial_started_at is not None and now - self._partial_started_at >= self.stale_after:
            self.parser.reset()
            self._partial_started_at = None
        if self._serial is None:
            if now >= self._next_open_time:
                self._open(now)
        if self._serial is None:
            return []
        try:
            waiting = int(self._serial.in_waiting)
            if waiting > MAX_READ_BYTES:
                self._serial.reset_input_buffer()
                self.parser.reset()
                self._partial_started_at = None
                self.input_backlog_overflows += 1
                return []
            data = self._serial.read(min(waiting, MAX_READ_BYTES)) if waiting else b""
            frames = self.parser.feed(data)
            if self.parser.buffered_bytes:
                if self._partial_started_at is None or frames:
                    self._partial_started_at = now
            else:
                self._partial_started_at = None
            if frames:
                self.frames_received += len(frames)
                self.last_frame_time = now
            return frames
        except (serial.SerialException, OSError) as error:
            self._drop_connection(str(error), now)
            return []

    def close(self) -> None:
        if self._serial is not None:
            try:
                self._serial.close()
            finally:
                self._serial = None
        self.parser.reset()
        self.last_frame_time = None
        self._partial_started_at = None

    def _open(self, now: float) -> None:
        connection = None
        try:
            connection = self._serial_factory()
            connection.port = self.port
            connection.baudrate = self.baud_rate
            connection.timeout = 0
            connection.write_timeout = WRITE_TIMEOUT_SECONDS
            connection.dtr = False
            connection.rts = False
            connection.open()
            self._serial = connection
            written = connection.write(DEBUG_MODE_COMMAND)
            if written != len(DEBUG_MODE_COMMAND):
                raise OSError(f"short initialization write: {written} of {len(DEBUG_MODE_COMMAND)} bytes")
            self._retry_delay = RECONNECT_DELAY_START
            self._next_open_time = 0.0
            self.last_io_error = ""
        except (serial.SerialException, OSError) as error:
            if connection is not None:
                try:
                    connection.close()
                except (serial.SerialException, OSError):
                    pass
            self._serial = None
            self.reconnects += 1
            self.last_io_error = str(error)
            self._next_open_time = now + self._retry_delay
            self._retry_delay = min(RECONNECT_DELAY_MAX, self._retry_delay * 2)

    def _drop_connection(self, error: str, now: float) -> None:
        connection, self._serial = self._serial, None
        if connection is not None:
            try:
                connection.close()
            except (serial.SerialException, OSError):
                pass
        self.parser.reset()
        self._partial_started_at = None
        self.last_frame_time = None
        self.reconnects += 1
        self.last_io_error = error
        self._next_open_time = now + self._retry_delay
        self._retry_delay = min(RECONNECT_DELAY_MAX, self._retry_delay * 2)
