import time


class WireTransport:
    """Delivers fixed bytes one at a time; the robot ignores everything the PC writes."""

    def __init__(self, data: bytes = b"", start_after_s: float = 0.0):
        self._data = bytearray(data)
        self._start = time.monotonic() + start_after_s

    def read(self) -> bytes:
        time.sleep(0.001)
        if time.monotonic() < self._start:
            return b""
        chunk, self._data = bytes(self._data[:1]), self._data[1:]
        return chunk

    def write(self, data: bytes) -> None:
        pass

    def close(self) -> None:
        pass


def poll_until(bridge, count: int, timeout_s: float = 2.0) -> list:
    out, deadline = [], time.monotonic() + timeout_s
    while len(out) < count and time.monotonic() < deadline:
        out += bridge.poll()
        time.sleep(0.002)
    return out
