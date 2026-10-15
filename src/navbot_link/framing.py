"""NavBot link framing: COBS + CRC-16/CCITT-FALSE, 0x00-delimited.

Frame on the wire:  COBS(payload || crc16_le(payload)) 0x00
`payload` is a serialized navbot.link.v1.Message.
"""

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


class FrameDecoder:
    """Feed arbitrary byte chunks; yields valid payloads, counts bad frames."""

    def __init__(self):
        self._buf = bytearray()
        self.bad_frames = 0

    def feed(self, chunk: bytes):
        for b in chunk:
            if b != 0:
                self._buf.append(b)
                if len(self._buf) > MAX_PAYLOAD + 8:  # runaway: resync at next 0x00
                    self._buf.clear()
                    self.bad_frames += 1
                continue
            raw, self._buf = bytes(self._buf), bytearray()
            if not raw:
                continue
            try:
                body = cobs_decode(raw)
            except ValueError:
                self.bad_frames += 1
                continue
            if len(body) < 2 or crc16(body[:-2]) != int.from_bytes(body[-2:], "little"):
                self.bad_frames += 1
                continue
            yield body[:-2]
