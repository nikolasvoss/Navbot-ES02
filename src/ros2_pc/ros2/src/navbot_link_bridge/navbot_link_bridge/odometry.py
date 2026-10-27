"""Differential-drive pose integration from cumulative wheel angles, optionally with gyro heading.

Convention: a positive wheel angle means forward travel, the left wheel is on
the left. Sign checks and open items: docs/integration/software/navbot-link-bridge.md.
"""

import math
from dataclasses import dataclass

_MICROS_MASK = 0xFFFFFFFF
# The IMU arrives every 20 ms. A longer interval means lost samples; the yaw
# across it is unknown, so it is not integrated.
GYRO_MAX_GAP_US = 60_000

HEADING_SOURCES = ("wheels", "gyro")


@dataclass(frozen=True)
class OdometryState:
    x_m: float
    y_m: float
    yaw_rad: float
    linear_mps: float
    angular_rad_s: float


def _positive_finite(value: float) -> bool:
    return math.isfinite(value) and value > 0.0


def gyro_yaw_sign_for(heading_source: str, gyro_yaw_sign: int) -> int | None:
    """Validates the heading parameters. Returns the gyro sign, or None for wheel heading."""
    if heading_source == "wheels":
        return None
    if heading_source != "gyro":
        raise ValueError(f"heading_source must be one of {HEADING_SOURCES}, got {heading_source!r}")
    if gyro_yaw_sign not in (1, -1):
        raise ValueError("heading_source 'gyro' requires gyro_yaw_sign 1 or -1 (IMU gz to ROS +yaw, CCW from above)")
    return gyro_yaw_sign


class GyroYaw:
    """Integrates sign * gz between IMU samples; take() hands out the yaw collected since the last take()."""

    def __init__(self, yaw_sign: int):
        self._sign = yaw_sign
        self.rebase()

    def add(self, t_us: int, gz_rad_s: float) -> None:
        if not math.isfinite(gz_rad_s):
            self._gap = True
            return
        rate = self._sign * gz_rad_s
        if self._prev is not None:
            prev_t_us, prev_rate = self._prev
            dt_us = (t_us - prev_t_us) & _MICROS_MASK  # micros() wraps at 2**32
            if dt_us == 0:
                return
            if dt_us > GYRO_MAX_GAP_US:
                self._gap = True
            else:
                self._d_yaw += (prev_rate + rate) / 2.0 * dt_us * 1e-6
                self._covered = True
        self._prev = (t_us, rate)

    def take(self) -> tuple[float, float] | None:
        """(yaw change, latest rate) since the last take, or None if no gap-free IMU interval covers it."""
        result = (self._d_yaw, self._prev[1]) if self._covered and not self._gap else None
        self._d_yaw = 0.0
        self._covered = False
        self._gap = False
        return result

    def rebase(self) -> None:
        self._prev = None  # (t_us, signed rate) of the last IMU sample
        self._d_yaw = 0.0
        self._covered = False
        self._gap = False


class DiffDriveOdometry:
    def __init__(self, wheel_radius_m: float, wheel_separation_m: float, gyro_yaw_sign: int | None = None):
        """gyro_yaw_sign None: heading from the wheels. +1 or -1: heading from IMU gz times that sign."""
        if not (_positive_finite(wheel_radius_m) and _positive_finite(wheel_separation_m)):
            raise ValueError("wheel radius and separation must be finite and > 0")
        if gyro_yaw_sign not in (None, 1, -1):
            raise ValueError("gyro_yaw_sign must be None, 1 or -1")
        self._radius = wheel_radius_m
        self._separation = wheel_separation_m
        self._gyro = None if gyro_yaw_sign is None else GyroYaw(gyro_yaw_sign)
        self._x = 0.0
        self._y = 0.0
        self._yaw = 0.0
        self._baseline = None  # (t_us, left_rad, right_rad) of the last sample
        self.gyro_fallbacks = 0  # gyro-mode steps that used the wheel yaw

    def add_gyro(self, t_us: int, gz_rad_s: float) -> None:
        """Feeds one IMU sample (gz in the sensor frame). Ignored in wheel-heading mode."""
        if self._gyro is not None:
            self._gyro.add(t_us, gz_rad_s)

    def update(self, t_us: int, left_angle_rad: float, right_angle_rad: float) -> OdometryState | None:
        """Integrates one wheel sample. Returns None for the baseline and for a zero dt."""
        if self._baseline is None:
            self._baseline = (t_us, left_angle_rad, right_angle_rad)
            if self._gyro is not None:
                self._gyro.take()  # yaw from before the baseline belongs to no step
            return None
        prev_t_us, prev_left, prev_right = self._baseline
        dt_us = (t_us - prev_t_us) & _MICROS_MASK  # micros() wraps at 2**32
        if dt_us == 0:
            return None
        dt_s = dt_us * 1e-6

        d_left = self._radius * (left_angle_rad - prev_left)
        d_right = self._radius * (right_angle_rad - prev_right)
        distance = (d_left + d_right) / 2.0
        d_yaw = (d_right - d_left) / self._separation
        angular = d_yaw / dt_s
        if self._gyro is not None:
            gyro = self._gyro.take()
            if gyro is None:
                self.gyro_fallbacks += 1
            else:
                d_yaw, angular = gyro

        mid_yaw = self._yaw + d_yaw / 2.0
        self._x += distance * math.cos(mid_yaw)
        self._y += distance * math.sin(mid_yaw)
        # atan2 keeps yaw in [-pi, pi] without accumulating across wraps.
        self._yaw = math.atan2(math.sin(self._yaw + d_yaw), math.cos(self._yaw + d_yaw))

        self._baseline = (t_us, left_angle_rad, right_angle_rad)
        return OdometryState(
            x_m=self._x,
            y_m=self._y,
            yaw_rad=self._yaw,
            linear_mps=distance / dt_s,
            angular_rad_s=angular,
        )

    def rebase(self) -> None:
        """Keeps the pose and drops the wheel and gyro baselines, so the next samples only re-baseline."""
        self._baseline = None
        if self._gyro is not None:
            self._gyro.rebase()
