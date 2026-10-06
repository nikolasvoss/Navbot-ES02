from dataclasses import dataclass
import struct


HEADER = b"\xAA\xBF\x10\x14"
FOOTER = b"\xFD\xFC\xFB\xFA"
DOPPLER_BINS = 20
RANGE_GATES = 16
VALUE_COUNT = DOPPLER_BINS * RANGE_GATES
PAYLOAD_SIZE = VALUE_COUNT * 4
FRAME_SIZE = len(HEADER) + PAYLOAD_SIZE + len(FOOTER)
MAX_BUFFER_SIZE = FRAME_SIZE * 2
DEBUG_MODE_COMMAND = bytes.fromhex("fd fc fb fa 08 00 12 00 00 00 00 00 00 00 04 03 02 01")
COMMAND_HEADER = b"\xFD\xFC\xFB\xFA"
COMMAND_FOOTER = b"\x04\x03\x02\x01"
MAX_COMMAND_DATA_SIZE = 64

ENTER_CONFIG_COMMAND = 0x00FF
READ_PARAMETER_COMMAND = 0x0008
WRITE_PARAMETER_COMMAND = 0x0007
EXIT_CONFIG_COMMAND = 0x00FE
MAXIMUM_DISTANCE_GATE_PARAMETER = 0x0001
TARGET_DISAPPEARANCE_DELAY_PARAMETER = 0x0004
RESPONSE_ACK_FLAG = 0x0100
POSITIVE_ACK_STATUS = 0x0000


@dataclass(frozen=True)
class RadarFrame:
    amplitude_squared: tuple[int, ...]


@dataclass(frozen=True)
class RadarCommandReply:
    command: int
    acknowledged: bool
    status: int
    payload: bytes


def encode_command(command: int, payload: bytes = b"") -> bytes:
    if isinstance(command, bool) or not isinstance(command, int) or not 0 <= command <= 0xFFFF:
        raise ValueError("command must be a uint16")
    if not isinstance(payload, bytes):
        raise TypeError("payload must be bytes")
    data_length = 2 + len(payload)
    if data_length > MAX_COMMAND_DATA_SIZE:
        raise ValueError("command data exceeds the protocol limit")
    return COMMAND_HEADER + struct.pack("<H", data_length) + struct.pack("<H", command) + payload + COMMAND_FOOTER


def encode_read_parameter(parameter: int) -> bytes:
    return encode_command(READ_PARAMETER_COMMAND, struct.pack("<H", parameter))


def encode_write_parameter(parameter: int, value: int) -> bytes:
    if isinstance(value, bool) or not isinstance(value, int) or not 0 <= value <= 0xFFFFFFFF:
        raise ValueError("parameter value must be a uint32")
    return encode_command(WRITE_PARAMETER_COMMAND, struct.pack("<HI", parameter, value))


def _decode_command_reply(frame: bytes) -> RadarCommandReply:
    data_length = struct.unpack_from("<H", frame, 4)[0]
    if data_length < 4:
        raise ValueError("command reply is shorter than its command and status")
    body = frame[6 : 6 + data_length]
    response_code, status = struct.unpack_from("<HH", body)
    return RadarCommandReply(
        response_code & 0x00FF,
        bool(response_code & RESPONSE_ACK_FLAG),
        status,
        body[4:],
    )


class HMMDStreamParser:
    """Routes complete radar maps and command replies from the shared UART stream."""

    def __init__(self):
        self._buffer = bytearray()
        self._map_parser = FrameParser()
        self.malformed_candidates = 0
        self.discarded_bytes = 0

    @property
    def buffered_bytes(self) -> int:
        return len(self._buffer) + self._map_parser.buffered_bytes

    def reset(self) -> None:
        self.discarded_bytes += len(self._buffer)
        self._buffer.clear()
        self._map_parser.reset()

    def feed(self, data: bytes) -> list[RadarFrame | RadarCommandReply]:
        if not isinstance(data, (bytes, bytearray, memoryview)):
            raise TypeError("data must be bytes-like")
        incoming = memoryview(data)
        events: list[RadarFrame | RadarCommandReply] = []
        offset = 0
        while offset < len(incoming):
            free = MAX_BUFFER_SIZE - len(self._buffer)
            if free == 0:
                self._extract(events)
                free = MAX_BUFFER_SIZE - len(self._buffer)
            count = min(free, len(incoming) - offset)
            self._buffer.extend(incoming[offset : offset + count])
            offset += count
            self._extract(events)
        return events

    def _discard(self, count: int) -> None:
        del self._buffer[:count]
        self.discarded_bytes += count

    def _extract(self, events: list[RadarFrame | RadarCommandReply]) -> None:
        while self._buffer:
            map_start = self._buffer.find(HEADER)
            command_start = self._buffer.find(COMMAND_HEADER)
            starts = [start for start in (map_start, command_start) if start >= 0]
            if not starts:
                keep = 0
                for header in (HEADER, COMMAND_HEADER):
                    for size in range(1, len(header)):
                        if self._buffer.endswith(header[:size]):
                            keep = max(keep, size)
                self._discard(len(self._buffer) - keep)
                return
            start = min(starts)
            if start:
                self._discard(start)
            if self._buffer.startswith(HEADER):
                if len(self._buffer) < FRAME_SIZE:
                    return
                footer_start = len(HEADER) + PAYLOAD_SIZE
                if self._buffer[footer_start : footer_start + len(FOOTER)] != FOOTER:
                    self.malformed_candidates += 1
                    self._discard(1)
                    continue
                map_bytes = bytes(self._buffer[:FRAME_SIZE])
                events.extend(self._map_parser.feed(map_bytes))
                self._discard(FRAME_SIZE)
                continue
            if len(self._buffer) < 6:
                return
            data_length = struct.unpack_from("<H", self._buffer, 4)[0]
            if data_length < 4 or data_length > MAX_COMMAND_DATA_SIZE:
                self.malformed_candidates += 1
                self._discard(1)
                continue
            command_frame_size = 4 + 2 + data_length + len(COMMAND_FOOTER)
            if len(self._buffer) < command_frame_size:
                return
            footer_start = command_frame_size - len(COMMAND_FOOTER)
            if self._buffer[footer_start:command_frame_size] != COMMAND_FOOTER:
                self.malformed_candidates += 1
                self._discard(1)
                continue
            frame = bytes(self._buffer[:command_frame_size])
            try:
                events.append(_decode_command_reply(frame))
            except ValueError:
                self.malformed_candidates += 1
            self._discard(command_frame_size)


class FrameParser:
    def __init__(self):
        self._buffer = bytearray()
        self.malformed_candidates = 0
        self.discarded_bytes = 0

    @property
    def buffered_bytes(self) -> int:
        return len(self._buffer)

    def reset(self) -> None:
        self.discarded_bytes += len(self._buffer)
        self._buffer.clear()

    def feed(self, data: bytes) -> list[RadarFrame]:
        if not isinstance(data, (bytes, bytearray, memoryview)):
            raise TypeError("data must be bytes-like")
        incoming = memoryview(data)
        frames = []
        offset = 0
        while offset < len(incoming):
            free = MAX_BUFFER_SIZE - len(self._buffer)
            if free == 0:
                self._extract(frames)
                free = MAX_BUFFER_SIZE - len(self._buffer)
            count = min(free, MAX_BUFFER_SIZE, len(incoming) - offset)
            self._buffer.extend(incoming[offset : offset + count])
            offset += count
            self._extract(frames)
        return frames

    def _discard(self, count: int) -> None:
        del self._buffer[:count]
        self.discarded_bytes += count

    def _extract(self, frames: list[RadarFrame]) -> None:
        while self._buffer:
            start = self._buffer.find(HEADER)
            if start < 0:
                keep = 0
                for size in range(1, len(HEADER)):
                    if self._buffer.endswith(HEADER[:size]):
                        keep = size
                self._discard(len(self._buffer) - keep)
                return
            if start:
                self._discard(start)
            if len(self._buffer) < FRAME_SIZE:
                return
            footer_start = len(HEADER) + PAYLOAD_SIZE
            if self._buffer[footer_start : footer_start + len(FOOTER)] != FOOTER:
                self.malformed_candidates += 1
                self._discard(1)
                continue
            payload = bytes(self._buffer[len(HEADER) : footer_start])
            values = tuple(value[0] for value in struct.iter_unpack("<I", payload))
            frames.append(RadarFrame(values))
            self._discard(FRAME_SIZE)
