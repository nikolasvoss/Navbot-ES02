"""NavBot link framing: COBS + CRC-16/CCITT-FALSE, 0x00-delimited.

Frame on the wire:  COBS(payload || crc16_le(payload)) 0x00
`payload` is a serialized navbot.link.v1.Message.
"""

from dataclasses import dataclass

CRC_INIT = 0xFFFF
MAX_PAYLOAD = 252  # keeps COBS output single-block-friendly and RAM bounded


def crc16(data: bytes) -> int:
    crc = CRC_INIT
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def cobs_encode(data: bytes) -> bytes:
    out = bytearray()
    block = bytearray()
    for i, b in enumerate(data):
        if b == 0:
            out.append(len(block) + 1)
            out += block
            block.clear()
            continue
        block.append(b)
        if len(block) == 254:
            out.append(0xFF)
            out += block
            block.clear()
            if i == len(data) - 1:
                return bytes(out)  # exact fit: no trailing block
    out.append(len(block) + 1)
    out += block
    return bytes(out)


def cobs_decode(data: bytes) -> bytes:
    out = bytearray()
    i = 0
    while i < len(data):
        code = data[i]
        end = i + code
        if code == 0 or end > len(data):
            raise ValueError("bad COBS block")
        out += data[i + 1:end]
        i = end
        if code < 0xFF and i < len(data):
            out.append(0)
    return bytes(out)


def encode_frame(payload: bytes) -> bytes:
    if len(payload) > MAX_PAYLOAD:
        raise ValueError("payload too large")
    return cobs_encode(payload + crc16(payload).to_bytes(2, "little")) + b"\x00"


@dataclass(frozen=True)
class Text:
    """Printable bytes between delimiters that are not a valid frame: ROM boot
    output, panic dumps, stray prints."""

    text: str


_PRINTABLE = frozenset(range(0x20, 0x7F)) | {0x09, 0x0A, 0x0D}


class FrameDecoder:
    """Feed arbitrary byte chunks; returns payloads (bytes) and Text in stream
    order, and counts chunks that are neither."""

    def __init__(self):
        self._buf = bytearray()
        self.bad_frames = 0

    def feed(self, chunk: bytes) -> list:
        out = []
        for b in chunk:
            if b != 0:
                self._buf.append(b)
                if len(self._buf) > MAX_PAYLOAD + 8:  # longer than any frame
                    self._flush_unframed(out)
                continue
            raw, self._buf = bytes(self._buf), bytearray()
            if raw:
                self._decode(raw, out)
        return out

    def flush(self) -> list:
        """Call when the line has been idle longer than any gap inside a frame:
        the robot writes whole frames, so pending bytes are then text without
        its delimiter (a boot message, a crash dump)."""
        out = []
        if self._buf:
            self._flush_unframed(out)
        return out

    def _flush_unframed(self, out: list) -> None:
        raw, self._buf = bytes(self._buf), bytearray()
        if _PRINTABLE.issuperset(raw):
            out.append(Text(raw.decode("ascii")))
        else:
            self.bad_frames += 1

    def _decode(self, raw: bytes, out: list) -> None:
        try:
            body = cobs_decode(raw)
        except ValueError:
            body = b""
        if 2 <= len(body) <= MAX_PAYLOAD + 2 and crc16(body[:-2]) == int.from_bytes(body[-2:], "little"):
            out.append(body[:-2])
        elif _PRINTABLE.issuperset(raw):
            out.append(Text(raw.decode("ascii")))
        else:
            self.bad_frames += 1
