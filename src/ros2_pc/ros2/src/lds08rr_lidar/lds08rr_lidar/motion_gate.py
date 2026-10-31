"""Per-scan motion gating for SLAM: drop scans taken while the robot turns or drives too fast.

ROS-free. A LiDAR scan sweeps for about 0.2 s and is not deskewed, so the robot's motion during the sweep
smears it: a turn at yaw rate w smears it by about w * 0.2 s, a drive at speed v by about v * 0.2 s.
Scans from fast motion are dropped before slam_toolbox merges them.

Time base: the odometry header stamp, the same clock as the scan header stamp.
"""

from __future__ import annotations

from bisect import bisect_right
from collections import deque
from threading import Lock

# Provisional: not measured. Tune from a logged drive.
MAX_YAW_RATE_RAD_S = 0.8
MAX_SPEED_M_S = 0.3
HOLDOFF_SEC = 0.4

ODOM_BUFFER_SEC = 2.0
ODOM_STALE_SEC = 0.1

REASON_TURN = "turning too fast"
REASON_SPEED = "driving too fast"


class MotionGate:
    """Odometry buffer plus gating state; fed by the ROS callbacks, read by the web status."""

    def __init__(
        self,
        max_yaw_rate: float = MAX_YAW_RATE_RAD_S,
        max_speed: float = MAX_SPEED_M_S,
        holdoff_sec: float = HOLDOFF_SEC,
        buffer_sec: float = ODOM_BUFFER_SEC,
        stale_sec: float = ODOM_STALE_SEC,
    ):
        self._max_yaw_rate = max_yaw_rate
        self._max_speed = max_speed
        self._holdoff = holdoff_sec
        self._buffer_sec = buffer_sec
        self._stale_sec = stale_sec
        self._lock = Lock()
        self._samples: deque[tuple[float, float, float]] = deque()  # (stamp, linear_x, angular_z)
        self._last_violation: tuple[float, str] | None = None  # (stamp, reason) of the latest over-limit sample
        self._reason: str | None = None
        self.state = "no_odom"
        self.scans_dropped = 0

    def add_odom(self, stamp_sec: float, linear_x: float, angular_z: float) -> None:
        with self._lock:
            if self._samples and stamp_sec < self._samples[-1][0]:
                return
            self._samples.append((stamp_sec, linear_x, angular_z))
            while self._samples[0][0] < stamp_sec - self._buffer_sec:
                self._samples.popleft()

    def apply(self, start_sec: float, scan_time: float) -> bool:
        """True to keep the scan for SLAM, False to drop it. Without odometry covering the window: kept."""
        end = start_sec + scan_time
        with self._lock:
            samples = list(self._samples)
        times = [sample[0] for sample in samples]
        # The sample just before the window counts too, so a limit crossed at the window start is seen.
        first = max(bisect_right(times, start_sec) - 1, 0)
        last = bisect_right(times, end)
        if last <= first or not self._covers(times, first, last, start_sec, end):
            self.state = "no_odom"
            self._reason = None
            return True

        for stamp, linear_x, angular_z in samples[first:last]:
            reason = self._reason_for(linear_x, angular_z)
            if reason is not None and (self._last_violation is None or stamp >= self._last_violation[0]):
                self._last_violation = (stamp, reason)

        if self._last_violation is None or start_sec >= self._last_violation[0] + self._holdoff:
            self.state = "ok"
            self._reason = None
            return True
        self.state = "limiting"
        self._reason = self._last_violation[1]
        self.scans_dropped += 1
        return False

    def snapshot(self) -> dict:
        return {
            "state": self.state,
            "scans_dropped": self.scans_dropped,
            "reason": self._reason,
            "max_yaw_rate": self._max_yaw_rate,
            "max_speed": self._max_speed,
        }

    def _reason_for(self, linear_x: float, angular_z: float) -> str | None:
        if abs(angular_z) > self._max_yaw_rate:
            return REASON_TURN
        if abs(linear_x) > self._max_speed:
            return REASON_SPEED
        return None

    def _covers(self, times, first: int, last: int, start_sec: float, end: float) -> bool:
        """Some sample within stale_sec of the window start and of its end."""
        if times[first] < start_sec - self._stale_sec or times[first] > start_sec + self._stale_sec:
            return False
        return times[last - 1] >= end - self._stale_sec
