"""Navigation supervisor: decides whether a goal may start and when a running goal must be cancelled. ROS-free.

The firmware is the fast stop. This class makes Nav2 give up the goal, so the robot does not drive on by itself
when a failed check clears again. Plan: docs/integration/plans/autonomous-driving.md (Supervisor).
"""

from __future__ import annotations

import time
from typing import Callable, Mapping

VOLTAGE_MIN_V = 6.8  # placeholder until U6 measures the discharge curve
HEALTH_MAX_AGE_SEC = 3.0
SCAN_MAX_AGE_SEC = 0.5
CHECKS = ("balance", "ch5", "remote", "health", "localization", "scan", "battery", "nav2")

NOT_READY = "not_ready"
READY = "ready"
DRIVING = "driving"
STOPPED = "stopped"


class NavSupervisor:
    """States not_ready, ready, driving and stopped, fed by the ROS node and called by SlamControl."""

    def __init__(self, clock: Callable[[], float] = time.monotonic) -> None:
        raise NotImplementedError

    def set_health(self, values: Mapping[str, str]) -> None:
        """The KeyValues of the bridge's navbot_firmware diagnostic status."""
        raise NotImplementedError

    def set_voltage(self, volts: float) -> None:
        raise NotImplementedError

    def set_localization(self, state: str) -> None:
        """The LocalizationTracker state: idle, searching, localized or lost."""
        raise NotImplementedError

    def scan_seen(self) -> None:
        raise NotImplementedError

    def set_nav_ready(self, ready: bool) -> None:
        raise NotImplementedError

    def request_goal(self) -> str | None:
        """None when a goal may start (the state becomes driving), else the refusal."""
        raise NotImplementedError

    def goal_ended(self) -> None:
        raise NotImplementedError

    def stop(self) -> None:
        raise NotImplementedError

    def resume(self) -> str | None:
        raise NotImplementedError

    def check(self) -> bool:
        """True exactly once when a running goal must be cancelled now."""
        raise NotImplementedError

    def snapshot(self) -> dict:
        """{"state", "reason", "checks": [{"name", "ok", "detail"}]} with the checks in CHECKS order."""
        raise NotImplementedError
