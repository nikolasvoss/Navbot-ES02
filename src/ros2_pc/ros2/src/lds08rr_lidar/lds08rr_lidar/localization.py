from __future__ import annotations

import math
import time
from pathlib import Path
from threading import Event, Lock, Thread, current_thread
from typing import Callable

LOCALIZATION_PARAMS = Path(__file__).with_name("localization.yaml")
XY_STD_MAX_M = 0.15
YAW_STD_MAX_RAD = math.radians(10.0)
STABLE_UPDATES = 5  # consecutive /amcl_pose messages under both limits
SEARCH_TIMEOUT_SEC = 30.0  # then state "lost" with hint "drive or turn a little"
LOST_FACTOR = 2.0  # localized -> lost when a std exceeds LOST_FACTOR x its limit for STABLE_UPDATES
REINITIALIZE_SERVICE = "/reinitialize_global_localization"
NOMOTION_SERVICE = "/request_nomotion_update"
SERVICE_CALL_TIMEOUT_SEC = 2.0
SERVICE_RETRY_SEC = 0.5
DRIVE_HINT = "Drive or turn the robot a little; AMCL keeps updating from motion."


def localization_command(map_yaml: Path) -> list[str]:
    return [
        "ros2", "launch", "nav2_bringup", "localization_launch.py", f"map:={map_yaml}",
        f"params_file:={LOCALIZATION_PARAMS}", "autostart:=true", "use_sim_time:=false",
        "use_composition:=False",
    ]


class LocalizationTracker:
    """Latest AMCL pose and the localization state, written by the ROS node and read by the web app."""

    def __init__(self, clock=time.monotonic) -> None:
        self._clock = clock
        self._lock = Lock()
        self._state = "idle"
        self._pose: dict | None = None
        self._std: dict | None = None
        self._search_started_at: float | None = None
        self._good = 0
        self._bad = 0
        self._error: str | None = None
        self._hint: str | None = None

    def start_search(self) -> None:
        with self._lock:
            self._state = "searching"
            self._pose = self._std = None
            self._search_started_at = self._clock()
            self._good = self._bad = 0
            self._error = self._hint = None

    def add_pose(self, x: float, y: float, yaw: float, var_x: float, var_y: float, var_yaw: float) -> None:
        with self._lock:
            self._expire_locked(self._clock())
            if self._state == "idle":
                return
            std_x, std_y, std_yaw = math.sqrt(max(var_x, 0.0)), math.sqrt(max(var_y, 0.0)), math.sqrt(max(var_yaw, 0.0))
            self._pose = {"x": x, "y": y, "yaw": yaw}
            self._std = {"x": std_x, "y": std_y, "yaw": std_yaw}
            within = std_x <= XY_STD_MAX_M and std_y <= XY_STD_MAX_M and std_yaw <= YAW_STD_MAX_RAD
            far_off = (
                std_x > LOST_FACTOR * XY_STD_MAX_M
                or std_y > LOST_FACTOR * XY_STD_MAX_M
                or std_yaw > LOST_FACTOR * YAW_STD_MAX_RAD
            )
            if self._state in ("searching", "lost"):
                self._good = self._good + 1 if within else 0
                if self._good >= STABLE_UPDATES:
                    self._set_localized_locked()
            elif self._state == "localized":
                self._bad = self._bad + 1 if far_off else 0
                if self._bad >= STABLE_UPDATES:
                    self._state, self._hint = "lost", DRIVE_HINT
                    self._good = self._bad = 0

    def fail(self, error: str) -> None:
        with self._lock:
            self._state, self._error, self._hint = "lost", error, None
            self._search_started_at = None
            self._good = self._bad = 0

    def reset(self) -> None:
        with self._lock:
            self._state = "idle"
            self._pose = self._std = self._search_started_at = None
            self._good = self._bad = 0
            self._error = self._hint = None

    def searching(self) -> bool:
        with self._lock:
            self._expire_locked(self._clock())
            return self._state == "searching"

    def snapshot(self) -> dict:
        with self._lock:
            self._expire_locked(self._clock())
            search_age = None
            if self._state == "searching" and self._search_started_at is not None:
                search_age = self._clock() - self._search_started_at
            return {
                "state": self._state,
                "pose": None if self._pose is None else dict(self._pose),
                "std": None if self._std is None else dict(self._std),
                "search_age_sec": search_age,
                "error": self._error,
                "hint": self._hint,
            }

    def _set_localized_locked(self) -> None:
        self._state, self._error, self._hint = "localized", None, None
        self._search_started_at = None
        self._good = self._bad = 0

    def _expire_locked(self, now: float) -> None:
        if (
            self._state == "searching"
            and self._search_started_at is not None
            and now - self._search_started_at >= SEARCH_TIMEOUT_SEC
        ):
            self._state, self._hint = "lost", DRIVE_HINT
            self._search_started_at = None
            self._good = 0


class GlobalLocalizer:
    """Global search on each localization start: reinitialize AMCL, then force scan updates until localized."""

    def __init__(
        self,
        call_service: Callable[[str, float], str | None],
        tracker: LocalizationTracker,
        service_timeout: float = 60.0,
        nomotion_period: float = 0.2,
    ) -> None:
        self._call_service = call_service
        self._tracker = tracker
        self._service_timeout = service_timeout
        self._nomotion_period = nomotion_period
        self._lock = Lock()
        self._cancel: Event | None = None
        self._thread: Thread | None = None
        self._runs = 0

    def start(self) -> None:
        with self._lock:
            if self._cancel is not None:
                self._cancel.set()
            cancel = Event()
            self._runs += 1
            self._cancel = cancel
            self._thread = Thread(
                target=self._run, args=(cancel,), name=f"global-localization-{self._runs}", daemon=True,
            )
            self._thread.start()

    def stop(self) -> None:
        with self._lock:
            cancel, thread = self._cancel, self._thread
            self._cancel = self._thread = None
            if cancel is not None:
                cancel.set()
        if thread is not None and thread is not current_thread():
            thread.join()

    def _run(self, cancel: Event) -> None:
        deadline = time.monotonic() + self._service_timeout
        while True:
            error = self._call_service(REINITIALIZE_SERVICE, SERVICE_CALL_TIMEOUT_SEC)
            if error is None:
                break
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                self._fail(cancel, f"{REINITIALIZE_SERVICE}: {error}")
                return
            if cancel.wait(min(SERVICE_RETRY_SEC, remaining)):
                return
        with self._lock:
            if cancel.is_set():
                return
            self._tracker.start_search()
        deadline = time.monotonic() + self._service_timeout
        while not cancel.is_set() and self._tracker.searching():
            error = self._call_service(NOMOTION_SERVICE, SERVICE_CALL_TIMEOUT_SEC)
            if error is None:
                cancel.wait(self._nomotion_period)
                continue
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                self._fail(cancel, f"{NOMOTION_SERVICE}: {error}")
                return
            if cancel.wait(min(SERVICE_RETRY_SEC, remaining)):
                return

    def _fail(self, cancel: Event, error: str) -> None:
        with self._lock:
            if not cancel.is_set():
                self._tracker.fail(error)
