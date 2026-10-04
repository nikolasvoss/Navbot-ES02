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


@dataclass(frozen=True)
class RadarFrame:
    amplitude_squared: tuple[int, ...]


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
