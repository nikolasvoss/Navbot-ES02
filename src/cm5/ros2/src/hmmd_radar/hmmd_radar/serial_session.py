from dataclasses import dataclass
from collections import deque
import math
import struct
import time

import serial

from .protocol import (
    DEBUG_MODE_COMMAND,
    ENTER_CONFIG_COMMAND,
    EXIT_CONFIG_COMMAND,
    HMMDStreamParser,
    MAXIMUM_DISTANCE_GATE_PARAMETER,
    POSITIVE_ACK_STATUS,
    READ_PARAMETER_COMMAND,
    TARGET_DISAPPEARANCE_DELAY_PARAMETER,
    WRITE_PARAMETER_COMMAND,
    RadarCommandReply,
    RadarFrame,
    encode_command,
    encode_read_parameter,
    encode_write_parameter,
)


MAX_READ_BYTES = 4096
WRITE_TIMEOUT_SECONDS = 0.5
COMMAND_TIMEOUT_SECONDS = 0.35
LATE_REPLY_DRAIN_SECONDS = 0.025
DEFERRED_MAP_LIMIT = 64
RECONNECT_DELAY_START = 0.25
RECONNECT_DELAY_MAX = 5.0


@dataclass(frozen=True)
class SessionStatus:
    connected: bool
    stale: bool
    last_frame_age_seconds: float | None


@dataclass(frozen=True)
class RadarConfigRead:
    success: bool
    maximum_distance_gate: int
    target_disappearance_delay_seconds: int
    outcome: str
    stage: str
    detail: str


@dataclass(frozen=True)
class RadarSettingWrite:
    success: bool
    write_acknowledged: bool
    has_observed_value: bool
    observed_value: int
    readback_matched: bool
    save_acknowledged: bool
    outcome: str
    stage: str
    detail: str


class SessionFailure(Exception):
    def __init__(self, outcome: str, stage: str, detail: str):
        super().__init__(detail)
        self.outcome = outcome
        self.stage = stage
        self.detail = detail


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
        self.parser = HMMDStreamParser()
        self._deferred_maps = deque(maxlen=DEFERRED_MAP_LIMIT)
        self.deferred_map_overflows = 0
        self.late_command_replies = 0
        self._write_needs_refresh = False
        self._timed_out_commands = {}
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
        frames = list(self._deferred_maps)
        self._deferred_maps.clear()
        if self._partial_started_at is not None and now - self._partial_started_at >= self.stale_after:
            self.parser.reset()
            self._partial_started_at = None
        if self._serial is None:
            if now >= self._next_open_time:
                self._open(now)
        if self._serial is None:
            return frames
        try:
            waiting = int(self._serial.in_waiting)
            if waiting > MAX_READ_BYTES:
                self._serial.reset_input_buffer()
                self.parser.reset()
                self._partial_started_at = None
                self.input_backlog_overflows += 1
                return frames
            data = self._serial.read(min(waiting, MAX_READ_BYTES)) if waiting else b""
            self._record_events(self.parser.feed(data), now)
            if self.parser.buffered_bytes:
                if self._partial_started_at is None or frames:
                    self._partial_started_at = now
            else:
                self._partial_started_at = None
            frames.extend(self._deferred_maps)
            self._deferred_maps.clear()
            return frames
        except (serial.SerialException, OSError) as error:
            self._drop_connection(str(error), now)
            return frames

    def _record_events(self, events, now=None):
        for event in events:
            if isinstance(event, RadarFrame):
                if len(self._deferred_maps) == self._deferred_maps.maxlen:
                    self.deferred_map_overflows += 1
                self._deferred_maps.append(event)
                self.frames_received += 1
                self.last_frame_time = time.monotonic() if now is None else now
            elif isinstance(event, RadarCommandReply):
                self.late_command_replies += 1

    def _transaction_events(self, events, expected_command=None):
        reply = None
        for event in events:
            if isinstance(event, RadarFrame):
                if len(self._deferred_maps) == self._deferred_maps.maxlen:
                    self.deferred_map_overflows += 1
                self._deferred_maps.append(event)
                self.frames_received += 1
                self.last_frame_time = time.monotonic()
            elif isinstance(event, RadarCommandReply):
                if event.command == expected_command and reply is None:
                    reply = event
                else:
                    self.late_command_replies += 1
        return reply

    def _read_serial_events(self, expected_command=None):
        waiting = int(self._serial.in_waiting)
        if not waiting:
            return None
        data = self._serial.read(min(waiting, MAX_READ_BYTES))
        return self._transaction_events(self.parser.feed(data), expected_command)

    def _drain_late_replies(self):
        deadline = time.monotonic() + LATE_REPLY_DRAIN_SECONDS
        while time.monotonic() < deadline and self.connected:
            try:
                self._read_serial_events()
            except (serial.SerialException, OSError) as error:
                self._drop_connection(str(error), time.monotonic())
                return
            time.sleep(0.002)

    def _exchange(self, request: bytes, command: int, stage: str, expected_payload: bytes | None) -> RadarCommandReply:
        if not self.connected:
            raise SessionFailure("not_connected", stage, "UART is not connected.")
        quarantine_until = self._timed_out_commands.pop(command, 0.0)
        while time.monotonic() < quarantine_until and self.connected:
            try:
                self._read_serial_events()
            except (serial.SerialException, OSError) as error:
                self._drop_connection(str(error), time.monotonic())
                break
            time.sleep(0.002)
        if not self.connected:
            raise SessionFailure("io_error", stage, self.last_io_error or "UART disconnected while draining a late reply.")
        self._drain_late_replies()
        if not self.connected:
            raise SessionFailure("io_error", stage, self.last_io_error or "UART disconnected before command write.")
        try:
            written = self._serial.write(request)
            if written != len(request):
                self._drop_connection(f"short command write: {written} of {len(request)} bytes", time.monotonic())
                raise SessionFailure("io_error", stage, self.last_io_error)
            deadline = time.monotonic() + COMMAND_TIMEOUT_SECONDS
            reply = None
            while time.monotonic() < deadline and self.connected:
                reply = self._read_serial_events(command)
                if reply is not None:
                    break
                time.sleep(0.002)
        except SessionFailure:
            raise
        except (serial.SerialException, OSError) as error:
            self._drop_connection(str(error), time.monotonic())
            raise SessionFailure("io_error", stage, str(error)) from error
        if reply is None:
            self._drain_late_replies()
            self._timed_out_commands[command] = time.monotonic() + COMMAND_TIMEOUT_SECONDS
            raise SessionFailure("timeout", stage, f"Timed out waiting for HMMD command 0x{command:04x}.")
        if not reply.acknowledged or reply.status != POSITIVE_ACK_STATUS:
            self._drain_late_replies()
            raise SessionFailure("device_rejected", stage, f"HMMD rejected command 0x{command:04x} with status {reply.status}.")
        if expected_payload is not None and reply.payload != expected_payload:
            self._drain_late_replies()
            raise SessionFailure("protocol_error", stage, f"Unexpected reply payload for HMMD command 0x{command:04x}.")
        self._drain_late_replies()
        return reply

    def _read_parameter(self, parameter: int, stage: str, maximum: int) -> int:
        reply = self._exchange(encode_read_parameter(parameter), READ_PARAMETER_COMMAND, stage, None)
        if len(reply.payload) != 4:
            raise SessionFailure("protocol_error", stage, "Parameter reply must contain one uint32 value.")
        value = struct.unpack("<I", reply.payload)[0]
        if value > maximum:
            raise SessionFailure("protocol_error", stage, f"Parameter readback {value} is outside 0 to {maximum}.")
        return value

    @staticmethod
    def _read_failure(outcome: str, stage: str, detail: str) -> RadarConfigRead:
        return RadarConfigRead(False, 0, 0, outcome, stage, detail)

    @staticmethod
    def _write_failure(
        outcome: str,
        stage: str,
        detail: str,
        write_acknowledged: bool = False,
        has_observed_value: bool = False,
        observed_value: int = 0,
        readback_matched: bool = False,
        save_acknowledged: bool = False,
    ) -> RadarSettingWrite:
        return RadarSettingWrite(
            False,
            write_acknowledged,
            has_observed_value,
            observed_value,
            readback_matched,
            save_acknowledged,
            outcome,
            stage,
            detail,
        )

    def read_radar_config(self) -> RadarConfigRead:
        if not self.connected and not self._open(time.monotonic()):
            return self._read_failure("not_connected", "enter_config", self.last_io_error or "UART is not connected.")
        entered = False
        try:
            entered = True
            self._exchange(encode_command(ENTER_CONFIG_COMMAND), ENTER_CONFIG_COMMAND, "enter_config", b"\x02\x00\x20\x00")
            maximum_distance_gate = self._read_parameter(MAXIMUM_DISTANCE_GATE_PARAMETER, "read_maximum_distance_gate", 15)
            target_disappearance_delay = self._read_parameter(TARGET_DISAPPEARANCE_DELAY_PARAMETER, "read_target_disappearance_delay", 65535)
        except SessionFailure as failure:
            original = failure
        else:
            original = None

        exit_failure = None
        if entered and self.connected:
            try:
                self._exchange(encode_command(EXIT_CONFIG_COMMAND), EXIT_CONFIG_COMMAND, "exit_config", b"")
            except SessionFailure as failure:
                exit_failure = failure
        if exit_failure:
            detail = exit_failure.detail if original is None else f"{original.detail}; config-mode exit failed: {exit_failure.detail}"
            return self._read_failure(exit_failure.outcome, "exit_config", detail)
        if original:
            return self._read_failure(original.outcome, original.stage, original.detail)
        self._write_needs_refresh = False
        return RadarConfigRead(
            True,
            maximum_distance_gate,
            target_disappearance_delay,
            "ok",
            "none",
            "Both radar settings were read and config mode was closed with an ACK.",
        )

    def set_radar_setting(self, setting: int, value: int) -> RadarSettingWrite:
        selection = {
            0: (MAXIMUM_DISTANCE_GATE_PARAMETER, 0, 15, "maximum distance gate"),
            1: (TARGET_DISAPPEARANCE_DELAY_PARAMETER, 0, 65535, "target disappearance delay"),
        }
        if isinstance(setting, bool) or not isinstance(setting, int) or setting not in selection:
            return self._write_failure("invalid_request", "none", "Unknown radar setting selector.")
        parameter, minimum, maximum, label = selection[setting]
        if isinstance(value, bool) or not isinstance(value, int) or not minimum <= value <= maximum:
            return self._write_failure("invalid_request", "none", f"{label} must be an integer from {minimum} to {maximum}.")
        if self._write_needs_refresh:
            return self._write_failure("refresh_required", "none", "Read both radar settings before another write.")
        if not self.connected and not self._open(time.monotonic()):
            return self._write_failure("not_connected", "enter_config", self.last_io_error or "UART is not connected.")

        write_acknowledged = False
        has_observed_value = False
        observed_value = 0
        readback_matched = False
        save_acknowledged = False
        entered = False
        failure = None
        stage = "enter_config"
        try:
            entered = True
            self._exchange(encode_command(ENTER_CONFIG_COMMAND), ENTER_CONFIG_COMMAND, stage, b"\x02\x00\x20\x00")
            stage = "write"
            self._write_needs_refresh = True
            self._exchange(encode_write_parameter(parameter, value), WRITE_PARAMETER_COMMAND, stage, b"")
            write_acknowledged = True
            stage = "readback"
            observed_value = self._read_parameter(parameter, stage, maximum)
            has_observed_value = True
            readback_matched = observed_value == value
            if not readback_matched:
                raise SessionFailure("readback_mismatch", stage, f"Readback was {observed_value}; requested {value}.")
            stage = "save"
            self._exchange(encode_command(EXIT_CONFIG_COMMAND), EXIT_CONFIG_COMMAND, stage, b"")
            save_acknowledged = True
            self._write_needs_refresh = False
        except SessionFailure as error:
            failure = error

        exit_failure = None
        if entered and self.connected and not save_acknowledged and not (failure and failure.stage == "save"):
            try:
                self._exchange(encode_command(EXIT_CONFIG_COMMAND), EXIT_CONFIG_COMMAND, "save", b"")
                save_acknowledged = True
            except SessionFailure as error:
                exit_failure = error
        if exit_failure:
            detail = exit_failure.detail if failure is None else f"{failure.detail}; config-mode exit failed: {exit_failure.detail}"
            return self._write_failure(exit_failure.outcome, "save", detail, write_acknowledged, has_observed_value, observed_value, readback_matched, save_acknowledged)
        if failure:
            return self._write_failure(failure.outcome, failure.stage, failure.detail, write_acknowledged, has_observed_value, observed_value, readback_matched, save_acknowledged)
        return RadarSettingWrite(
            True,
            write_acknowledged,
            has_observed_value,
            observed_value,
            readback_matched,
            save_acknowledged,
            "ok",
            "none",
            "Write ACK, matching readback, and config-mode exit ACK received.",
        )

    def close(self) -> None:
        if self._serial is not None:
            try:
                self._serial.close()
            finally:
                self._serial = None
        self.parser.reset()
        self.last_frame_time = None
        self._partial_started_at = None

    def _open(self, now: float) -> bool:
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
            return True
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
            return False

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
