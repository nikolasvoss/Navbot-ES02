from __future__ import annotations

import base64
import time
from threading import Lock

STALE_AFTER_SEC = 10.0


def grid_to_dict(width: int, height: int, resolution: float, origin_x: float, origin_y: float, data) -> dict:
    """Pack an occupancy grid for the browser: int8 cells, row-major, -1 unknown, base64."""
    return {
        "width": width,
        "height": height,
        "resolution": resolution,
        "origin_x": origin_x,
        "origin_y": origin_y,
        "data": base64.b64encode(bytes(v & 0xFF for v in data)).decode("ascii"),
    }


class SlamState:
    """Latest map and pose, written by the ROS node and read by the web app."""

    def __init__(self, clock=time.monotonic) -> None:
        self._clock = clock
        self._lock = Lock()
        self._grid: dict | None = None
        self._grid_at: float | None = None
        self._pose: dict | None = None
        self._pose_at: float | None = None

    def set_grid(self, grid: dict) -> None:
        with self._lock:
            self._grid, self._grid_at = grid, self._clock()

    def set_pose(self, x: float, y: float, yaw: float) -> None:
        with self._lock:
            self._pose, self._pose_at = {"x": x, "y": y, "yaw": yaw}, self._clock()

    def snapshot(self) -> dict:
        now = self._clock()
        with self._lock:
            grid, grid_at, pose, pose_at = self._grid, self._grid_at, self._pose, self._pose_at
        map_age = None if grid_at is None else now - grid_at
        pose_age = None if pose_at is None else now - pose_at
        if grid is None:
            state = "waiting"
        elif pose is None or pose_age > STALE_AFTER_SEC:
            state = "no_pose"
        else:
            state = "mapping"
        return {
            "state": state,
            "map": grid,
            "pose": pose if pose_age is not None and pose_age <= STALE_AFTER_SEC else None,
            "map_age_sec": map_age,
            "pose_age_sec": pose_age,
        }
