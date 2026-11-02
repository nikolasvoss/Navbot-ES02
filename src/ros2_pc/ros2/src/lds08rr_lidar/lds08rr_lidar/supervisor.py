"""Navigation supervisor: decides whether a goal may start and when a running goal must be cancelled. ROS-free.

The firmware is the fast stop. This class makes Nav2 give up the goal, so the robot does not drive on by itself
when a failed check clears again. Plan: docs/integration/plans/autonomous-driving.md (Supervisor).
"""

from __future__ import annotations

import time
from typing import Callable, Mapping

VOLTAGE_MIN_V = 6.8  # placeholder until U6 measures the discharge curve
HEALTH_MAX_AGE_SEC = 3.0
SCAN_MAX_AGE_SEC = 1.0  # five missed scans at 5 Hz; the collision monitor stops the robot after two
CHECKS = ("balance", "ch5", "remote", "health", "localization", "scan", "battery", "nav2")

NOT_READY = "not_ready"
READY = "ready"
DRIVING = "driving"
STOPPED = "stopped"

USER_STOP = "stopped by the user"


class NavSupervisor:
    """States not_ready, ready, driving and stopped, fed by the ROS node and called by SlamControl."""

    def __init__(self, clock: Callable[[], float] = time.monotonic) -> None:
        self._clock = clock
        self._health: Mapping[str, str] | None = None
        self._health_at = 0.0
        self._volts: float | None = None
        self._localization: str | None = None
        self._scan_at: float | None = None
        self._nav_ready = False
        self._state = NOT_READY
        self._reason: str | None = None
        self._cancel_pending = False

    def set_health(self, values: Mapping[str, str]) -> None:
        """The KeyValues of the bridge's navbot_firmware diagnostic status."""
        self._health = dict(values)
        self._health_at = self._clock()
        self._settle()

    def set_voltage(self, volts: float) -> None:
        self._volts = volts
        self._settle()

    def set_localization(self, state: str) -> None:
        """The LocalizationTracker state: idle, searching, localized or lost."""
        self._localization = state
        self._settle()

    def scan_seen(self) -> None:
        self._scan_at = self._clock()
        self._settle()

    def set_nav_ready(self, ready: bool) -> None:
        self._nav_ready = ready
        self._settle()

    def request_goal(self) -> str | None:
        """None when a goal may start (the state becomes driving), else the refusal."""
        self._settle()
        if self._state == STOPPED:
            return f"stopped: {self._reason}; press Resume"
        if self._state == NOT_READY:
            return self._reason
        if self._state == READY:
            self._state = DRIVING
        return None

    def goal_ended(self) -> None:
        if self._state == DRIVING:
            self._state = READY
            self._settle()

    def stop(self) -> None:
        if self._state == STOPPED:
            return
        if self._state == DRIVING:
            self._cancel_pending = True
        self._state = STOPPED
        self._reason = USER_STOP

    def resume(self) -> str | None:
        if self._state != STOPPED:
            return None
        failing = self._first_failure()
        if failing is not None:
            return f"cannot resume: {failing}"
        self._state = READY
        self._reason = None
        return None

    def check(self) -> bool:
        """True exactly once when a running goal must be cancelled now."""
        if self._cancel_pending:
            self._cancel_pending = False
            return True
        if self._state != DRIVING:
            return False
        failing = self._first_failure()
        if failing is None:
            return False
        self._state = STOPPED
        self._reason = failing
        return True

    def snapshot(self) -> dict:
        """{"state", "reason", "checks": [{"name", "ok", "detail"}]} with the checks in CHECKS order."""
        self._settle()
        return {
            "state": self._state,
            "reason": self._reason,
            "checks": [{"name": name, "ok": ok, "detail": detail} for name, ok, detail in self._results()],
        }

    def _settle(self) -> None:
        """Move between not_ready and ready from the current checks. Driving and stopped only change by their own calls."""
        if self._state not in (NOT_READY, READY):
            return
        failing = self._first_failure()
        self._state = READY if failing is None else NOT_READY
        self._reason = failing

    def _first_failure(self) -> str | None:
        for _name, ok, detail in self._results():
            if not ok:
                return detail
        return None

    def _results(self) -> list[tuple[str, bool, str]]:
        """(name, ok, detail) for each check, in CHECKS order."""
        now = self._clock()
        health = self._health
        if health is None:
            no_health = "no robot health yet"
            balance = (False, no_health)
            ch5 = (False, no_health)
            remote = (False, no_health)
        else:
            state = str(health.get("balance_state", "unknown"))
            ok = state == "Run"
            balance = (ok, state if ok else f"balance is {state}")
            position = str(health.get("ch5_position", "unknown"))
            ok = position == "1"
            ch5 = (ok, position if ok else f"CH5 is in position {position}, not 1")
            rc_valid = str(health.get("rc_valid", "0"))
            ok = rc_valid in ("1", "True")
            remote = (ok, rc_valid if ok else "no remote signal")

        if health is None:
            health_check = (False, "no robot health yet")
        else:
            age = now - self._health_at
            if age <= HEALTH_MAX_AGE_SEC:
                health_check = (True, f"{age:.1f} s")
            else:
                health_check = (False, f"no robot health for {age:.1f} s")

        if self._localization is None:
            localization = (False, "localization is unknown")
        elif self._localization == "localized":
            localization = (True, "localized")
        else:
            localization = (False, f"localization is {self._localization}")

        if self._scan_at is None:
            scan = (False, f"no LiDAR scan for {SCAN_MAX_AGE_SEC:g} s")
        else:
            age = now - self._scan_at
            if age <= SCAN_MAX_AGE_SEC:
                scan = (True, f"{age:.1f} s")
            else:
                scan = (False, f"no LiDAR scan for {SCAN_MAX_AGE_SEC:g} s")

        if self._volts is None:
            battery = (False, "no battery voltage yet")
        elif self._volts >= VOLTAGE_MIN_V:
            battery = (True, f"{self._volts:.2f} V")
        else:
            battery = (False, f"battery at {self._volts:.2f} V, below {VOLTAGE_MIN_V:g} V")

        nav2 = (True, "ready") if self._nav_ready else (False, "Nav2 is not ready")

        ordered = (balance, ch5, remote, health_check, localization, scan, battery, nav2)
        return [(name, ok, detail) for name, (ok, detail) in zip(CHECKS, ordered)]
