import math
import time
from numbers import Integral

from .protocol import DOPPLER_BINS, RANGE_GATES, VALUE_COUNT


UINT32_MAX = (1 << 32) - 1


class RangeDopplerViewModel:
    def __init__(self, stale_after: float = 1.0):
        if not math.isfinite(stale_after) or stale_after <= 0:
            raise ValueError("stale_after must be finite and positive")
        self.stale_after = stale_after
        self.matrix = tuple(tuple(0 for _ in range(RANGE_GATES)) for _ in range(DOPPLER_BINS))
        self.received_at: float | None = None

    def update(self, doppler_bins: int, range_gates: int, amplitude_squared, now: float | None = None) -> None:
        if doppler_bins != DOPPLER_BINS or range_gates != RANGE_GATES:
            raise ValueError("message dimensions must be 20 by 16")
        values = tuple(amplitude_squared)
        if len(values) != VALUE_COUNT:
            raise ValueError("message must contain exactly 320 amplitudes")
        if any(isinstance(v, bool) or not isinstance(v, Integral) or v < 0 or v > UINT32_MAX for v in values):
            raise ValueError("amplitudes must be uint32 values")
        values = tuple(int(value) for value in values)
        self.matrix = tuple(
            tuple(values[doppler * RANGE_GATES : (doppler + 1) * RANGE_GATES])
            for doppler in range(DOPPLER_BINS)
        )
        self.received_at = time.monotonic() if now is None else now

    def display_matrix(self, logarithmic: bool = False) -> tuple[tuple[float, ...], ...]:
        if not logarithmic:
            return self.matrix
        return tuple(tuple(math.log1p(value) for value in row) for row in self.matrix)

    def is_stale(self, now: float | None = None) -> bool:
        if self.received_at is None:
            return True
        now = time.monotonic() if now is None else now
        return max(0.0, now - self.received_at) > self.stale_after
