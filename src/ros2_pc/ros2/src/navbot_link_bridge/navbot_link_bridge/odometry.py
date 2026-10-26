"""Differential-drive pose integration from cumulative wheel angles.

Convention: a positive wheel angle means forward travel, the left wheel is on
the left. This is not verified on hardware; see docs/ros2_pc/software/README.md.
"""

import math
from dataclasses import dataclass

_MICROS_MASK = 0xFFFFFFFF


@dataclass(frozen=True)
class OdometryState:
    x_m: float
    y_m: float
    yaw_rad: float
    linear_mps: float
    angular_rad_s: float


def _positive_finite(value: float) -> bool:
    return math.isfinite(value) and value > 0.0


class DiffDriveOdometry:
    def __init__(self, wheel_radius_m: float, wheel_separation_m: float):
        if not (_positive_finite(wheel_radius_m) and _positive_finite(wheel_separation_m)):
            raise ValueError("wheel radius and separation must be finite and > 0")
        self._radius = wheel_radius_m
        self._separation = wheel_separation_m
        self._x = 0.0
        self._y = 0.0
        self._yaw = 0.0
        self._baseline = None  # (t_us, left_rad, right_rad) of the last sample

    def update(self, t_us: int, left_angle_rad: float, right_angle_rad: float) -> OdometryState | None:
        """Integrates one wheel sample. Returns None for the baseline and for a zero dt."""
        if self._baseline is None:
            self._baseline = (t_us, left_angle_rad, right_angle_rad)
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
            angular_rad_s=d_yaw / dt_s,
        )

    def rebase(self) -> None:
        """Keeps the pose and drops the baseline, so the next sample only re-baselines."""
        self._baseline = None
