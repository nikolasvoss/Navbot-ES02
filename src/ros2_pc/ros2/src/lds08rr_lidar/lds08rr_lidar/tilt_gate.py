"""Per-scan tilt gating for SLAM: drop scans that tilt too far and beams that would reach the floor.

ROS-free. The balancing robot pitches about its wheel axis; the scan plane pitches with it. A beam at
base_link bearing phi (CCW from forward) then has elevation -pitch * cos(phi), and its endpoint sits at
height h + r * sin(elevation) above the floor. Pitch is positive nose-down (REP-103).
"""

from __future__ import annotations

import math
from bisect import bisect_right
from collections import deque
from dataclasses import dataclass
from threading import Lock

from . import mount

IMU_BUFFER_SEC = 2.0
IMU_STALE_SEC = 0.1


@dataclass(frozen=True)
class TiltConfig:
    scan_height_m: float
    roll_to_pitch_sign: float
    level_roll_rad: float
    max_pitch_rad: float
    floor_margin_m: float
    mount_yaw_rad: float

    def pitch(self, imu_roll_rad: float) -> float:
        return self.roll_to_pitch_sign * (imu_roll_rad - self.level_roll_rad)


def config_from_mount() -> TiltConfig | None:
    """None while any of the three mount measurements is missing."""
    measured = (mount.SCAN_PLANE_HEIGHT_M, mount.IMU_ROLL_TO_PITCH_SIGN, mount.IMU_ROLL_LEVEL_RAD)
    if any(value is None for value in measured):
        return None
    if not 0.0 < mount.SCAN_PLANE_HEIGHT_M < 1.0:
        raise ValueError(f"SCAN_PLANE_HEIGHT_M must be in metres (0..1), got {mount.SCAN_PLANE_HEIGHT_M}")
    if mount.IMU_ROLL_TO_PITCH_SIGN not in (1, -1):
        raise ValueError(f"IMU_ROLL_TO_PITCH_SIGN must be 1 or -1, got {mount.IMU_ROLL_TO_PITCH_SIGN}")
    if not abs(mount.IMU_ROLL_LEVEL_RAD) < math.pi:
        raise ValueError(f"IMU_ROLL_LEVEL_RAD must be in radians, got {mount.IMU_ROLL_LEVEL_RAD}")
    return TiltConfig(
        scan_height_m=mount.SCAN_PLANE_HEIGHT_M,
        roll_to_pitch_sign=mount.IMU_ROLL_TO_PITCH_SIGN,
        level_roll_rad=mount.IMU_ROLL_LEVEL_RAD,
        max_pitch_rad=mount.MAX_SCAN_PITCH_RAD,
        floor_margin_m=mount.FLOOR_MARGIN_M,
        mount_yaw_rad=mount.SENSOR_MOUNT_YAW_RAD,
    )


def roll_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    """Roll of a REP-103 ZYX (yaw-pitch-roll) quaternion."""
    return math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))


def window_pitch_range(times, pitches, start: float, end: float, stale_sec: float = IMU_STALE_SEC):
    """(min, max) pitch over [start, end] from time-ordered samples, including the one just before start.

    None when the samples do not cover the window: nothing within stale_sec of its start or of its end.
    """
    first = max(bisect_right(times, start) - 1, 0)
    last = bisect_right(times, end)
    if last <= first:
        return None
    if times[first] < start - stale_sec or times[first] > start + stale_sec:
        return None
    if times[last - 1] < end - stale_sec:
        return None
    window = pitches[first:last]
    return min(window), max(window)


def gate_ranges(ranges, angle_min: float, angle_increment: float, pitch_range, config: TiltConfig):
    """Gated ranges and the number of beams set to +inf; (None, 0) when the whole scan is too tilted."""
    low, high = pitch_range
    if max(abs(low), abs(high)) > config.max_pitch_rad:
        return None, 0
    gated, dropped = list(ranges), 0
    for index, distance in enumerate(gated):
        if not math.isfinite(distance):
            continue
        bearing = angle_min + index * angle_increment + config.mount_yaw_rad
        for pitch in (low, high):
            height = config.scan_height_m + distance * math.sin(-pitch * math.cos(bearing))
            if height < config.floor_margin_m:
                gated[index] = math.inf
                dropped += 1
                break
    return gated, dropped


class TiltGate:
    """IMU buffer plus gating state; fed by the ROS callbacks, read by the web status."""

    def __init__(self, config: TiltConfig | None, buffer_sec: float = IMU_BUFFER_SEC, stale_sec: float = IMU_STALE_SEC):
        self._config = config
        self._buffer_sec = buffer_sec
        self._stale_sec = stale_sec
        self._lock = Lock()
        self._samples: deque[tuple[float, float]] = deque()
        self.state = "unconfigured" if config is None else "no_imu"
        self.scans_dropped = 0
        self.beams_dropped = 0

    def add_imu(self, stamp_sec: float, roll_rad: float) -> None:
        with self._lock:
            if self._samples and stamp_sec < self._samples[-1][0]:
                return
            self._samples.append((stamp_sec, roll_rad))
            while self._samples[0][0] < stamp_sec - self._buffer_sec:
                self._samples.popleft()

    def apply(self, ranges, angle_min: float, angle_increment: float, start_sec: float, scan_time: float):
        """Ranges to publish for SLAM, or None to drop the scan. Unconfigured or without IMU: unchanged."""
        if self._config is None:
            return list(ranges)
        with self._lock:
            times = [t for t, _ in self._samples]
            pitches = [self._config.pitch(roll) for _, roll in self._samples]
        pitch_range = window_pitch_range(times, pitches, start_sec, start_sec + scan_time, self._stale_sec)
        if pitch_range is None:
            self.state = "no_imu"
            return list(ranges)
        self.state = "active"
        gated, dropped = gate_ranges(ranges, angle_min, angle_increment, pitch_range, self._config)
        if gated is None:
            self.scans_dropped += 1
        self.beams_dropped += dropped
        return gated

    def snapshot(self) -> dict:
        return {"state": self.state, "scans_dropped": self.scans_dropped, "beams_dropped": self.beams_dropped}
