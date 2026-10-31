import time

from fake_firmware import E, FakeEntry
from navbot_link.health import HEALTH_SIGNALS

_HEALTH_VALUES = {"balance_state": 2, "voltage_filtered_v": 7.4, "voltage_min_raw_v": 7.1, "uptime_s": 12}
_HEALTH_BOOLS = {"overspeed_tripped", "motor_test_active", "rc_valid"}


def with_health(entries):
    """The entries plus a constant signal for each HEALTH_SIGNALS name the table lacks."""
    have = {e.name for e in entries}
    extra = []
    for name in HEALTH_SIGNALS:
        if name in have:
            continue
        type_ = E.F32 if name.startswith("voltage_") else E.BOOL if name in _HEALTH_BOOLS else E.U32
        extra.append(FakeEntry(name, E.SIGNAL, type_, "", _HEALTH_VALUES.get(name, 0)))
    return list(entries) + extra


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
