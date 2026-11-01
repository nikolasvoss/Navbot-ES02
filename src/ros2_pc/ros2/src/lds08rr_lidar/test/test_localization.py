import math
import time
from pathlib import Path
from threading import Event, Lock

import pytest

from lds08rr_lidar import localization
from lds08rr_lidar.localization import (
    DRIVE_HINT,
    LOCALIZATION_PARAMS,
    REINITIALIZE_SERVICE,
    GlobalLocalizer,
    LocalizationTracker,
    localization_command,
)

GOOD_VAR_XY = 0.05**2  # std 0.05 m, under the 0.15 m limit
GOOD_VAR_YAW = math.radians(3.0) ** 2  # under the 10 deg limit
BAD_VAR_XY = 1.0**2  # std 1.0 m, more than 2 x the limit


class FakeClock:
    def __init__(self, now=0.0):
        self.now = now

    def __call__(self):
        return self.now


def good_pose(tracker, x=1.0, y=2.0, yaw=0.5):
    tracker.add_pose(x, y, yaw, GOOD_VAR_XY, GOOD_VAR_XY, GOOD_VAR_YAW)


def bad_pose(tracker):
    tracker.add_pose(0.0, 0.0, 0.0, BAD_VAR_XY, BAD_VAR_XY, GOOD_VAR_YAW)


def wait_until(predicate, timeout=5.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(0.01)
    raise AssertionError("condition was not reached in time")


def test_localization_command_runs_our_launch_wrapper_with_our_params(monkeypatch):
    monkeypatch.setattr(localization, "LOCALIZATION_PARAMS", Path("/cfg/localization.yaml"))
    monkeypatch.setattr(localization, "LOCALIZATION_LAUNCH", Path("/cfg/localization_launch.py"))

    assert localization_command(Path("/data/map/map.yaml")) == [
        "ros2", "launch", "/cfg/localization_launch.py", "map:=/data/map/map.yaml",
        "params_file:=/cfg/localization.yaml",
    ]


def test_launch_wrapper_sits_next_to_the_module():
    assert localization.LOCALIZATION_LAUNCH == Path(localization.__file__).with_name("localization_launch.py")
    assert localization.LOCALIZATION_LAUNCH.is_file()


def test_params_file_sits_next_to_the_module():
    assert LOCALIZATION_PARAMS == Path(localization.__file__).with_name("localization.yaml")


def test_params_file_sets_amcl_and_map_server_keys():
    yaml = pytest.importorskip("yaml")
    params = yaml.safe_load(LOCALIZATION_PARAMS.read_text(encoding="utf-8"))

    amcl = params["amcl"]["ros__parameters"]
    assert amcl["base_frame_id"] == "base_link"
    assert amcl["odom_frame_id"] == "odom"
    assert amcl["global_frame_id"] == "map"
    assert amcl["scan_topic"] == "/scan_slam"
    assert amcl["robot_model_type"] == "nav2_amcl::DifferentialMotionModel"
    assert amcl["laser_model_type"] == "likelihood_field"
    assert amcl["laser_max_range"] == 4.0
    assert amcl["max_beams"] == 60
    assert amcl["min_particles"] == 500
    assert amcl["max_particles"] == 8000
    assert amcl["set_initial_pose"] is False
    assert amcl["tf_broadcast"] is True
    assert amcl["transform_tolerance"] == 1.0
    assert amcl["update_min_d"] == 0.1
    assert amcl["update_min_a"] == 0.1
    assert "map_server" not in params


def test_tracker_starts_idle_and_ignores_poses_until_a_search_starts():
    tracker = LocalizationTracker(clock=FakeClock())
    good_pose(tracker)

    assert tracker.snapshot() == {
        "state": "idle", "pose": None, "std": None, "search_age_sec": None, "error": None, "hint": None,
    }
    assert tracker.searching() is False


def test_five_good_poses_localize_and_report_pose_and_std():
    tracker = LocalizationTracker(clock=FakeClock())
    tracker.start_search()
    for _ in range(4):
        good_pose(tracker, x=1.0, y=2.0, yaw=0.5)
    assert tracker.snapshot()["state"] == "searching"

    good_pose(tracker, x=1.0, y=2.0, yaw=0.5)
    snap = tracker.snapshot()
    assert snap["state"] == "localized"
    assert snap["pose"] == {"x": 1.0, "y": 2.0, "yaw": 0.5}
    assert snap["std"]["x"] == pytest.approx(0.05)
    assert snap["std"]["y"] == pytest.approx(0.05)
    assert snap["std"]["yaw"] == pytest.approx(math.radians(3.0))
    assert snap["search_age_sec"] is None
    assert snap["error"] is None and snap["hint"] is None
    assert tracker.searching() is False


def test_one_bad_pose_restarts_the_good_count():
    tracker = LocalizationTracker(clock=FakeClock())
    tracker.start_search()
    for _ in range(4):
        good_pose(tracker)
    bad_pose(tracker)
    for _ in range(4):
        good_pose(tracker)
    assert tracker.snapshot()["state"] == "searching"

    good_pose(tracker)
    assert tracker.snapshot()["state"] == "localized"


def test_yaw_std_above_limit_counts_as_not_converged():
    tracker = LocalizationTracker(clock=FakeClock())
    tracker.start_search()
    yaw_too_loose = math.radians(11.0) ** 2
    for _ in range(10):
        tracker.add_pose(0.0, 0.0, 0.0, GOOD_VAR_XY, GOOD_VAR_XY, yaw_too_loose)
    assert tracker.snapshot()["state"] == "searching"


def test_search_times_out_to_lost_with_drive_hint():
    clock = FakeClock()
    tracker = LocalizationTracker(clock=clock)
    tracker.start_search()
    clock.now = localization.SEARCH_TIMEOUT_SEC - 0.1
    assert tracker.searching() is True
    assert tracker.snapshot()["search_age_sec"] == pytest.approx(localization.SEARCH_TIMEOUT_SEC - 0.1)

    clock.now = localization.SEARCH_TIMEOUT_SEC
    snap = tracker.snapshot()
    assert snap["state"] == "lost"
    assert snap["hint"] == DRIVE_HINT
    assert tracker.searching() is False


def test_timeout_is_seen_without_any_pose_arriving():
    clock = FakeClock()
    tracker = LocalizationTracker(clock=clock)
    tracker.start_search()
    clock.now = 100.0
    assert tracker.snapshot()["state"] == "lost"


def test_localized_goes_lost_after_five_far_off_poses():
    tracker = LocalizationTracker(clock=FakeClock())
    tracker.start_search()
    for _ in range(5):
        good_pose(tracker)
    assert tracker.snapshot()["state"] == "localized"

    for _ in range(4):
        bad_pose(tracker)
    assert tracker.snapshot()["state"] == "localized"
    good_pose(tracker)  # a good pose in between resets the bad count
    for _ in range(4):
        bad_pose(tracker)
    assert tracker.snapshot()["state"] == "localized"

    bad_pose(tracker)
    snap = tracker.snapshot()
    assert snap["state"] == "lost"
    assert snap["hint"] == DRIVE_HINT


def test_lost_recovers_to_localized_after_five_good_poses():
    tracker = LocalizationTracker(clock=FakeClock())
    tracker.start_search()
    tracker.fail("boom")
    for _ in range(5):
        good_pose(tracker)

    snap = tracker.snapshot()
    assert snap["state"] == "localized"
    assert snap["error"] is None


def test_fail_sets_lost_with_error_text():
    tracker = LocalizationTracker(clock=FakeClock())
    tracker.start_search()
    tracker.fail("service missing")

    snap = tracker.snapshot()
    assert snap["state"] == "lost"
    assert snap["error"] == "service missing"
    assert snap["hint"] is None
    assert tracker.searching() is False


def test_reset_returns_to_idle_and_drops_pose():
    tracker = LocalizationTracker(clock=FakeClock())
    tracker.start_search()
    for _ in range(5):
        good_pose(tracker)
    tracker.reset()

    snap = tracker.snapshot()
    assert snap["state"] == "idle"
    assert snap["pose"] is None and snap["std"] is None


class FakeService:
    """Records every service call; `missing` fails the first N reinitialize calls, `nomotion_error` fails all nomotion calls."""

    def __init__(self, missing=0, nomotion_error=None):
        self.missing = missing
        self.nomotion_error = nomotion_error
        self.calls = []  # (name, thread name)
        self._lock = Lock()

    def __call__(self, name, timeout):
        with self._lock:
            self.calls.append((name, _thread_name()))
            if name == REINITIALIZE_SERVICE and self.missing > 0:
                self.missing -= 1
                return "service not available"
            if name != REINITIALIZE_SERVICE and self.nomotion_error:
                return self.nomotion_error
            return None

    def names(self, name):
        with self._lock:
            return [thread for called, thread in self.calls if called == name]


def _thread_name():
    import threading

    return threading.current_thread().name


@pytest.fixture
def fast_retry(monkeypatch):
    monkeypatch.setattr(localization, "SERVICE_RETRY_SEC", 0.01)


def test_global_localizer_retries_missing_service_then_reinitializes_once(fast_retry):
    service = FakeService(missing=2)
    tracker = LocalizationTracker()
    localizer = GlobalLocalizer(service, tracker, service_timeout=5.0, nomotion_period=0.01)

    localizer.start()
    try:
        wait_until(lambda: tracker.searching())
        names = [name for name, _ in service.calls]
        assert names.count(REINITIALIZE_SERVICE) == 3
        assert names[0] == REINITIALIZE_SERVICE
    finally:
        localizer.stop()


def test_global_localizer_forces_nomotion_updates_until_localized(fast_retry):
    service = FakeService()
    tracker = LocalizationTracker()
    localizer = GlobalLocalizer(service, tracker, service_timeout=5.0, nomotion_period=0.01)

    localizer.start()
    try:
        wait_until(lambda: len(service.names("/request_nomotion_update")) >= 3)
        for _ in range(localization.STABLE_UPDATES):
            good_pose(tracker)
        assert tracker.snapshot()["state"] == "localized"

        settled = len(service.names("/request_nomotion_update"))
        time.sleep(0.2)
        assert len(service.names("/request_nomotion_update")) == settled
    finally:
        localizer.stop()


def test_global_localizer_service_timeout_fails_the_tracker(fast_retry):
    service = FakeService(missing=10_000)
    tracker = LocalizationTracker()
    localizer = GlobalLocalizer(service, tracker, service_timeout=0.1, nomotion_period=0.01)

    localizer.start()
    try:
        wait_until(lambda: tracker.snapshot()["state"] == "lost")
        snap = tracker.snapshot()
        assert REINITIALIZE_SERVICE in snap["error"]
        assert "service not available" in snap["error"]
        assert service.names("/request_nomotion_update") == []
    finally:
        localizer.stop()


def test_global_localizer_nomotion_error_retries_then_fails_the_tracker(fast_retry):
    service = FakeService(nomotion_error="amcl gone")
    tracker = LocalizationTracker()
    localizer = GlobalLocalizer(service, tracker, service_timeout=0.1, nomotion_period=0.01)

    localizer.start()
    try:
        wait_until(lambda: tracker.snapshot()["state"] == "lost")
        assert tracker.snapshot()["error"] == "/request_nomotion_update: amcl gone"
        assert len(service.names("/request_nomotion_update")) > 1
    finally:
        localizer.stop()


def test_global_localizer_survives_a_transient_nomotion_error(fast_retry):
    class FlakyNomotion(FakeService):
        """The first two nomotion calls fail, later ones succeed."""

        def __call__(self, name, timeout):
            error = super().__call__(name, timeout)
            if name == "/request_nomotion_update" and len(self.names(name)) <= 2:
                return "service busy"
            return error

    service = FlakyNomotion()
    tracker = LocalizationTracker()
    localizer = GlobalLocalizer(service, tracker, service_timeout=5.0, nomotion_period=0.01)

    localizer.start()
    try:
        wait_until(lambda: len(service.names("/request_nomotion_update")) >= 5)
        assert tracker.snapshot()["state"] == "searching"
        assert tracker.snapshot()["error"] is None
    finally:
        localizer.stop()


def test_second_start_cancels_the_first_run(fast_retry):
    gate = Event()
    first_reinit_seen = Event()

    class GatedService(FakeService):
        def __call__(self, name, timeout):
            if name == REINITIALIZE_SERVICE and not first_reinit_seen.is_set():
                first_reinit_seen.set()
                gate.wait(5.0)  # the first run is stuck inside its reinitialize call
            return super().__call__(name, timeout)

    service = GatedService()
    tracker = LocalizationTracker()
    localizer = GlobalLocalizer(service, tracker, service_timeout=5.0, nomotion_period=0.01)

    localizer.start()
    wait_until(first_reinit_seen.is_set)
    localizer.start()
    wait_until(lambda: len(service.names("/request_nomotion_update")) >= 2)
    gate.set()
    try:
        wait_until(lambda: len(service.names(REINITIALIZE_SERVICE)) >= 2)
        time.sleep(0.1)
        nomotion_threads = set(service.names("/request_nomotion_update"))
        assert nomotion_threads == {"global-localization-2"}
        assert tracker.snapshot()["state"] == "searching"
    finally:
        localizer.stop()


def test_stop_ends_the_search_without_touching_the_tracker(fast_retry):
    service = FakeService()
    tracker = LocalizationTracker()
    localizer = GlobalLocalizer(service, tracker, service_timeout=5.0, nomotion_period=0.01)

    localizer.start()
    wait_until(lambda: len(service.names("/request_nomotion_update")) >= 2)
    localizer.stop()
    calls = len(service.calls)
    time.sleep(0.1)
    assert len(service.calls) == calls
