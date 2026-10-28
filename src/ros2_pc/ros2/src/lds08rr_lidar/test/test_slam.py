import base64
import os
import sys
import time
from pathlib import Path

import pytest

pytest.importorskip("flask")

from lds08rr_lidar.localization import LocalizationTracker, localization_command
from lds08rr_lidar.runtime import LidarRuntime
from lds08rr_lidar.slam import MapStore, SlamControl, SlamProcess, SlamState, export_occupancy, grid_to_dict
from lds08rr_lidar.source import SourceConfig
from lds08rr_lidar.web import make_lidar_app


class FakeClock:
    def __init__(self, now=0.0):
        self.now = now

    def __call__(self):
        return self.now


def wait_until(predicate, timeout=5.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(0.02)
    raise AssertionError("condition was not reached in time")


def test_map_and_pose_go_stale_independently():
    clock = FakeClock()
    slam = SlamState(clock=clock)
    slam.set_grid(grid_to_dict(1, 1, 0.05, 0.0, 0.0, [0]))
    slam.set_pose(0.0, 0.0, 0.0)
    assert slam.snapshot()["state"] == "mapping"

    clock.now = 11.0
    shown = slam.snapshot()
    assert shown["state"] == "stale"
    assert shown["map_age_sec"] == 11.0

    clock = FakeClock(now=20.0)
    slam = SlamState(clock=clock)
    slam.set_grid(grid_to_dict(1, 1, 0.05, 0.0, 0.0, [0]))
    slam.set_pose(1.0, 2.0, 0.5, age_sec=11.0)
    shown = slam.snapshot()
    assert shown["state"] == "no_pose"
    assert shown["pose"] is None
    assert shown["pose_age_sec"] == 11.0


def write_map_set(directory, content):
    for suffix in (".data", ".posegraph", ".pgm", ".yaml"):
        Path(directory, f"map{suffix}").write_text(content, encoding="utf-8")


def test_map_store_commits_all_four_files_and_loads_them_on_every_start(tmp_path):
    store = MapStore(tmp_path / "slam")
    base_yaml = "slam_toolbox:\n  ros__parameters:\n    mode: mapping\n"
    assert "map_file_name" not in store.params_file(base_yaml).read_text(encoding="utf-8")
    assert not store.has_occupancy()

    tmp_dir = store.directory / "map.saving"
    tmp_dir.mkdir()
    write_map_set(tmp_dir, "new")
    store.commit(tmp_dir)
    assert store.exists()
    assert store.has_occupancy()
    assert store.occupancy_yaml() == tmp_path / "slam" / "map.yaml"
    assert store.saved_at() is not None
    assert (store.directory / "map.pgm").read_text(encoding="utf-8") == "new"

    params = store.params_file(base_yaml).read_text(encoding="utf-8")
    assert params == (
        "slam_toolbox:\n  ros__parameters:\n    mode: mapping\n"
        f'    map_file_name: "{tmp_path / "slam" / "map"}"\n'
        "    map_start_at_dock: true\n"
    )

    store.delete()
    store.delete()
    assert not store.exists()
    assert not store.has_occupancy()
    assert store.saved_at() is None
    assert not any((store.directory / f"map{suffix}").exists() for suffix in (".data", ".posegraph", ".pgm", ".yaml"))


def test_map_store_has_no_occupancy_map_without_the_yaml(tmp_path):
    store = MapStore(tmp_path)
    tmp_dir = tmp_path / "map.saving"
    tmp_dir.mkdir()
    write_map_set(tmp_dir, "new")
    Path(tmp_dir, "map.yaml").unlink()
    with pytest.raises(FileNotFoundError):
        store.commit(tmp_dir)
    assert not store.has_occupancy()


def test_map_store_refuses_an_incomplete_serialized_map(tmp_path):
    store = MapStore(tmp_path)
    tmp_dir = tmp_path / "map.saving"
    tmp_dir.mkdir()
    for suffix in (".data", ".posegraph", ".pgm"):
        Path(tmp_dir, f"map{suffix}").write_text("new", encoding="utf-8")
    with pytest.raises(FileNotFoundError):
        store.commit(tmp_dir)
    assert not store.exists()
    assert not store.has_occupancy()
    assert (tmp_dir / "map.data").exists()
    assert not (tmp_path / "map.data").exists()


def test_map_store_keeps_the_old_set_when_the_commit_fails(tmp_path, monkeypatch):
    store = MapStore(tmp_path)
    write_map_set(tmp_path, "old")
    tmp_dir = tmp_path / "map.saving"
    tmp_dir.mkdir()
    write_map_set(tmp_dir, "new")

    real_replace = os.replace
    def failing_replace(src, dst):
        if Path(src) == tmp_dir / "map.posegraph":
            raise OSError("disk full")
        real_replace(src, dst)
    monkeypatch.setattr(os, "replace", failing_replace)
    with pytest.raises(OSError):
        store.commit(tmp_dir)
    monkeypatch.undo()

    for suffix in (".data", ".posegraph", ".pgm", ".yaml"):
        assert Path(f"{store.base}{suffix}").read_text(encoding="utf-8") == "old"
    assert not list(tmp_path.glob("*.old"))


def test_map_store_removes_new_files_when_the_first_save_fails(tmp_path, monkeypatch):
    store = MapStore(tmp_path)
    tmp_dir = tmp_path / "map.saving"
    tmp_dir.mkdir()
    write_map_set(tmp_dir, "new")

    real_replace = os.replace
    def failing_replace(src, dst):
        if Path(src) == tmp_dir / "map.pgm":
            raise OSError("disk full")
        real_replace(src, dst)
    monkeypatch.setattr(os, "replace", failing_replace)
    with pytest.raises(OSError):
        store.commit(tmp_dir)
    monkeypatch.undo()

    assert not any((tmp_path / f"map{suffix}").exists() for suffix in (".data", ".posegraph", ".pgm"))


def test_slam_process_survives_a_command_that_raises(tmp_path):
    process = SlamProcess(lambda: 1 / 0, tmp_path / "slam.log", restart_delay=0.05)
    process.start()
    wait_until(lambda: process.status()["restarts"] >= 2)
    assert process.status()["error"] == "ZeroDivisionError: division by zero"
    process.stop()


def test_slam_process_runs_stops_and_starts_again(tmp_path):
    process = SlamProcess(lambda: [sys.executable, "-c", "import time; time.sleep(60)"], tmp_path / "slam.log")
    for _ in range(2):
        process.start()
        wait_until(lambda: process.status()["pid"] is not None)
        status = process.status()
        assert status["state"] == "running"
        pid = status["pid"]
        process.stop()
        assert process.status()["state"] == "stopped"
        with pytest.raises(ProcessLookupError):
            os.kill(pid, 0)
    process.stop()


def test_slam_process_restarts_a_child_that_exits(tmp_path):
    process = SlamProcess(lambda: [sys.executable, "-c", "raise SystemExit(3)"], tmp_path / "slam.log", restart_delay=0.05)
    process.start()
    wait_until(lambda: process.status()["restarts"] >= 2)
    status = process.status()
    assert status["exit_code"] == 3
    process.stop()
    assert process.status()["state"] == "stopped"


class FakeProcess:
    def __init__(self):
        self.calls = []

    def start(self):
        self.calls.append("start")

    def stop(self, timeout=10.0):
        self.calls.append("stop")

    def status(self):
        return {"state": "running", "pid": 1, "exit_code": None, "restarts": 0, "error": None}


def write_serialized_map(base):
    Path(f"{base}.posegraph").write_text("graph", encoding="utf-8")
    Path(f"{base}.data").write_text("data", encoding="utf-8")
    return None


def write_occupancy(base):
    Path(f"{base}.pgm").write_text("pgm", encoding="utf-8")
    Path(f"{base}.yaml").write_text(f"image: {Path(base).name}.pgm\n", encoding="utf-8")
    return None


def test_slam_control_saves_then_deletes_and_restarts_slam(tmp_path):
    store = MapStore(tmp_path)
    process = FakeProcess()
    control = SlamControl(store, process, write_serialized_map, write_occupancy)

    assert control.save() is None
    assert control.status()["saved_map"]["exists"] is True
    assert (tmp_path / "map.posegraph").read_text(encoding="utf-8") == "graph"
    assert (tmp_path / "map.pgm").read_text(encoding="utf-8") == "pgm"
    assert (tmp_path / "map.yaml").read_text(encoding="utf-8") == "image: map.pgm\n"
    assert not list(tmp_path.glob("map.saving.*"))

    control.delete()
    assert control.status()["saved_map"]["exists"] is False
    assert not (tmp_path / "map.yaml").exists()
    assert process.calls == ["stop", "start"]


def test_slam_control_exports_the_occupancy_map_from_the_serialized_base(tmp_path):
    seen = {}
    def export(base):
        seen["base"] = base
        return write_occupancy(base)
    control = SlamControl(MapStore(tmp_path), FakeProcess(), write_serialized_map, export)

    assert control.save() is None
    assert seen["base"].name == "map"
    assert seen["base"].parent.name.startswith("map.saving.")
    assert not seen["base"].parent.exists()


def test_slam_control_reload_restarts_slam_only_with_a_saved_map(tmp_path):
    process = FakeProcess()
    control = SlamControl(MapStore(tmp_path), process, write_serialized_map, write_occupancy)

    assert control.reload() == "No saved map to load"
    assert process.calls == []

    control.save()
    assert control.reload() is None
    assert process.calls == ["stop", "start"]
    assert control.status()["saved_map"]["exists"] is True


def test_slam_control_restarts_slam_when_the_delete_fails(tmp_path):
    class BrokenStore(MapStore):
        def delete(self):
            raise PermissionError("read-only")

    process = FakeProcess()
    control = SlamControl(BrokenStore(tmp_path), process, write_serialized_map, write_occupancy)
    with pytest.raises(PermissionError):
        control.delete()
    assert process.calls == ["stop", "start"]


def test_slam_control_reports_a_failed_serialize(tmp_path):
    control = SlamControl(MapStore(tmp_path), FakeProcess(), lambda base: "boom", write_occupancy)
    assert control.save() == "boom"
    assert control.status()["saved_map"]["exists"] is False
    assert not list(tmp_path.glob("map.saving.*"))


def test_slam_control_keeps_the_old_map_when_the_export_fails(tmp_path):
    store = MapStore(tmp_path)
    assert SlamControl(store, FakeProcess(), write_serialized_map, write_occupancy).save() is None

    control = SlamControl(store, FakeProcess(), write_serialized_map, lambda base: "map export timed out")
    assert control.save() == "map export timed out"
    assert (tmp_path / "map.posegraph").read_text(encoding="utf-8") == "graph"
    assert (tmp_path / "map.pgm").read_text(encoding="utf-8") == "pgm"
    assert (tmp_path / "map.yaml").read_text(encoding="utf-8") == "image: map.pgm\n"
    assert not list(tmp_path.glob("map.saving.*"))


class RecordingProcess:
    """Records each start with the command the control builds for it, as the supervisor does."""

    def __init__(self, events):
        self.events = events
        self.command = None

    def start(self):
        command = self.command()
        self.events.append(("process start", command))

    def stop(self, timeout=10.0):
        self.events.append(("process stop",))

    def status(self):
        return {"state": "running", "pid": 1, "exit_code": None, "restarts": 0, "error": None}


class RecordingLocalizer:
    def __init__(self, events):
        self.events = events

    def start(self):
        self.events.append(("localizer start",))

    def stop(self):
        self.events.append(("localizer stop",))


class RecordingSlamState:
    def __init__(self):
        self.calls = []

    def clear(self):
        self.calls.append(("clear",))

    def set_map_latched(self, latched):
        self.calls.append(("latched", latched))


def mode_control(tmp_path, events, tracker=None, slam_state=None):
    store = MapStore(tmp_path)
    process = RecordingProcess(events)
    control = SlamControl(
        store, process, write_serialized_map, write_occupancy,
        localizer=RecordingLocalizer(events), tracker=tracker, slam_state=slam_state,
        mapping_command=lambda: ["mapping-launch"],
    )
    process.command = control.command
    return control, store


def test_boot_mode_is_localization_only_when_the_occupancy_image_exists(tmp_path):
    control, _ = mode_control(tmp_path, [])
    assert control.initial_mode() == "mapping"
    Path(tmp_path, "map.posegraph").write_text("graph", encoding="utf-8")
    assert control.initial_mode() == "mapping"
    Path(tmp_path, "map.yaml").write_text("image: map.pgm\n", encoding="utf-8")
    assert control.initial_mode() == "localization"


def test_start_without_a_map_runs_mapping_and_no_localizer(tmp_path):
    events = []
    slam_state = RecordingSlamState()
    control, _ = mode_control(tmp_path, events, tracker=LocalizationTracker(), slam_state=slam_state)
    control.start()
    assert control.mode == "mapping"
    assert events == [("process start", ["mapping-launch"])]
    assert slam_state.calls == [("clear",), ("latched", False)]


def test_start_boots_into_localization_on_the_saved_occupancy_map(tmp_path):
    events = []
    slam_state = RecordingSlamState()
    tracker = LocalizationTracker()
    control, store = mode_control(tmp_path, events, tracker=tracker, slam_state=slam_state)
    assert control.save() is None
    control.start()
    assert control.mode == "localization"
    assert events == [
        ("localizer start",),
        ("process start", localization_command(store.occupancy_yaml())),
    ]
    assert slam_state.calls == [("clear",), ("latched", True)]
    assert tracker.snapshot()["state"] == "idle"


def test_set_mode_stops_the_running_launch_and_starts_the_other(tmp_path):
    events = []
    control, store = mode_control(tmp_path, events)
    control.save()
    control.start()
    events.clear()

    assert control.set_mode("mapping") is None
    assert control.mode == "mapping"
    assert events == [
        ("process stop",),
        ("localizer stop",),
        ("process start", ["mapping-launch"]),
    ]

    events.clear()
    assert control.set_mode("localization") is None
    assert control.mode == "localization"
    assert events == [
        ("process stop",),
        ("localizer stop",),
        ("localizer start",),
        ("process start", localization_command(store.occupancy_yaml())),
    ]

    events.clear()
    assert control.set_mode("localization") is None
    assert events == []


def test_switching_mode_clears_the_view_and_the_pose(tmp_path):
    slam_state = RecordingSlamState()
    tracker = LocalizationTracker()
    control, _ = mode_control(tmp_path, [], tracker=tracker, slam_state=slam_state)
    control.save()
    control.start()
    assert slam_state.calls == [("clear",), ("latched", True)]
    tracker.start_search()
    assert tracker.snapshot()["state"] == "searching"

    slam_state.calls.clear()
    assert control.set_mode("mapping") is None
    assert slam_state.calls == [("clear",), ("latched", False)]
    assert tracker.snapshot()["state"] == "idle"


def test_set_mode_refuses_localization_without_an_occupancy_map_and_unknown_modes(tmp_path):
    events = []
    control, _ = mode_control(tmp_path, events)
    control.start()
    events.clear()

    assert control.set_mode("localization") == "No occupancy map to localize on; save the map once in mapping mode first"
    assert control.set_mode("nonsense") == "unknown mode 'nonsense'; use mapping or localization"
    assert control.mode == "mapping"
    assert events == []


def test_save_and_locate_are_refused_in_the_wrong_mode(tmp_path):
    events = []
    control, store = mode_control(tmp_path, events, tracker=LocalizationTracker())
    assert control.save() is None
    control.start()
    before = store.occupancy_yaml().read_text(encoding="utf-8")
    events.clear()

    assert control.save() == "Save needs mapping mode; switch to mapping first"
    assert store.occupancy_yaml().read_text(encoding="utf-8") == before
    assert events == []

    assert control.locate() is None
    assert events == [("localizer start",)]

    control.set_mode("mapping")
    events.clear()
    assert control.locate() == "Locate needs localization mode; switch to localization first"
    assert events == []
    assert control.save() is None


def test_every_localization_launch_starts_the_global_search(tmp_path):
    events = []
    control, store = mode_control(tmp_path, events)
    control.save()
    control.start()
    events.clear()

    assert control.command() == localization_command(store.occupancy_yaml())
    assert events == [("localizer start",)]


def test_delete_in_localization_ends_in_mapping(tmp_path):
    events = []
    control, store = mode_control(tmp_path, events)
    control.save()
    control.start()
    events.clear()

    control.delete()
    assert not store.exists() and not store.has_occupancy()
    assert control.mode == "mapping"
    assert events == [
        ("process stop",),
        ("localizer stop",),
        ("process start", ["mapping-launch"]),
    ]


def test_stop_stops_the_launch_and_the_localizer(tmp_path):
    events = []
    control, _ = mode_control(tmp_path, events)
    control.stop()
    assert events == [("process stop",), ("localizer stop",)]


def test_status_reports_the_mode_and_the_localization_snapshot(tmp_path):
    tracker = LocalizationTracker()
    control, _ = mode_control(tmp_path, [], tracker=tracker)
    control.start()
    shown = control.status()
    assert shown["mode"] == "mapping"
    assert shown["localization"] == tracker.snapshot()


FAKE_ROS2_HEAD = (
    "#!/bin/sh\n"
    'printf \'%s\\n\' "$@" > "$(dirname "$0")/args.txt"\n'
    'while [ "$1" != "-f" ]; do shift; done\n'
    'base="$2"\n'
)


def use_fake_ros2(monkeypatch, tmp_path, body):
    bin_dir = tmp_path / "bin"
    bin_dir.mkdir(exist_ok=True)
    script = bin_dir / "ros2"
    script.write_text(FAKE_ROS2_HEAD + body, encoding="utf-8")
    script.chmod(0o755)
    monkeypatch.setenv("PATH", f"{bin_dir}{os.pathsep}{os.environ['PATH']}")
    return bin_dir


def test_export_occupancy_runs_map_saver_on_the_transient_local_map(tmp_path, monkeypatch):
    bin_dir = use_fake_ros2(monkeypatch, tmp_path, (
        'printf \'image: %s.pgm\\n\' "$(basename "$base")" > "$base.yaml"\n'
        "printf 'P5' > \"$base.pgm\"\n"
    ))
    assert export_occupancy(tmp_path / "map") is None
    assert (tmp_path / "map.pgm").read_bytes() == b"P5"
    assert (tmp_path / "map.yaml").read_text(encoding="utf-8") == "image: map.pgm\n"
    assert (bin_dir / "args.txt").read_text(encoding="utf-8").split("\n") == [
        "run", "nav2_map_server", "map_saver_cli", "-f", str(tmp_path / "map"),
        "--ros-args", "-p", "map_subscribe_transient_local:=true", "",
    ]


def test_export_occupancy_reports_a_failed_saver_and_a_missing_file(tmp_path, monkeypatch):
    use_fake_ros2(monkeypatch, tmp_path, "echo 'no map received' >&2\nexit 3\n")
    error = export_occupancy(tmp_path / "map")
    assert error.startswith("map export failed with exit code 3")
    assert "no map received" in error

    use_fake_ros2(monkeypatch, tmp_path, "printf 'P5' > \"$base.pgm\"\n")
    assert export_occupancy(tmp_path / "map") == "map export wrote no map.yaml"


def test_export_occupancy_kills_a_saver_that_hangs(tmp_path, monkeypatch):
    use_fake_ros2(monkeypatch, tmp_path, "sleep 30\n")
    started = time.monotonic()
    assert export_occupancy(tmp_path / "map", timeout=0.5) == "map export timed out"
    assert time.monotonic() - started < 10


class FakeControl:
    def __init__(self, error=None):
        self.error = error
        self.deleted = 0
        self.reloaded = 0
        self.reload_error = None

    def status(self):
        return {"process": {"state": "running"}, "saved_map": {"exists": False, "saved_at": None}}

    def save(self):
        return self.error

    def delete(self):
        self.deleted += 1

    def reload(self):
        self.reloaded += 1
        return self.reload_error


def make_client(tmp_path, slam, control):
    runtime = LidarRuntime(SourceConfig(mode="demo"), driver_factory=lambda *_: None)
    return make_lidar_app(runtime, slam=slam, control=control, radar_web=tmp_path / "missing").test_client()


def test_slam_actions_are_refused_without_a_running_control(tmp_path):
    client = make_client(tmp_path, SlamState(), None)
    refused = {"ok": False, "error": "SLAM is not running on this service"}
    for path in ("/slam/save", "/slam/delete", "/slam/reset"):
        response = client.post(path)
        assert response.status_code == 409
        assert response.get_json() == refused
    assert client.get("/map.json").get_json()["control"] is None


def test_web_save_and_delete_drive_the_control(tmp_path):
    slam = SlamState()
    control = FakeControl()
    client = make_client(tmp_path, slam, control)

    response = client.post("/slam/save")
    assert response.status_code == 409
    assert response.get_json() == {"ok": False, "error": "No map to save yet"}

    slam.set_grid(grid_to_dict(2, 1, 0.05, 0.0, 0.0, [-1, 100]))
    assert client.get("/map.json").get_json()["control"] == control.status()
    assert client.post("/slam/save").get_json() == {"ok": True}

    control.error = "boom"
    response = client.post("/slam/save")
    assert response.status_code == 500
    assert response.get_json() == {"ok": False, "error": "boom"}

    assert base64.b64decode(client.get("/map.json").get_json()["map"]["data"]) == b"\xff\x64"
    assert client.post("/slam/delete").get_json() == {"ok": True}
    assert control.deleted == 1
    cleared = client.get("/map.json").get_json()
    assert cleared["state"] == "waiting" and cleared["map"] is None


def test_web_refuses_cross_site_posts(tmp_path):
    control = FakeControl()
    client = make_client(tmp_path, SlamState(), control)
    for headers in ({"Origin": "http://evil.example"}, {"Sec-Fetch-Site": "cross-site"}):
        response = client.post("/slam/delete", headers=headers)
        assert response.status_code == 403
    assert control.deleted == 0
    same = client.post("/slam/delete", headers={"Origin": "http://localhost"})
    assert same.get_json() == {"ok": True}
    assert control.deleted == 1


def test_web_reset_reloads_the_saved_map_and_clears_the_view(tmp_path):
    slam = SlamState()
    control = FakeControl()
    client = make_client(tmp_path, slam, control)
    slam.set_grid(grid_to_dict(1, 1, 0.05, 0.0, 0.0, [0]))

    control.reload_error = "No saved map to load"
    response = client.post("/slam/reset")
    assert response.status_code == 409
    assert response.get_json() == {"ok": False, "error": "No saved map to load"}
    assert client.get("/map.json").get_json()["map"] is not None

    control.reload_error = None
    assert client.post("/slam/reset").get_json() == {"ok": True}
    assert client.get("/map.json").get_json()["map"] is None


def test_odom_accumulates_path_and_goes_stale():
    clock = FakeClock()
    slam = SlamState(clock=clock)
    assert slam.snapshot()["odom"] is None
    for i in range(5):
        clock.now = i * 0.1
        slam.set_odom(0.3 * i, 0.4 * i, 0.1, 0.2, 0.0)
    shown = slam.snapshot()["odom"]
    assert shown["path_m"] == pytest.approx(2.0)
    assert shown["rate_hz"] == pytest.approx(10.0)
    assert shown["fresh"]

    clock.now = 10.0
    shown = slam.snapshot()["odom"]
    assert not shown["fresh"] and shown["rate_hz"] is None
