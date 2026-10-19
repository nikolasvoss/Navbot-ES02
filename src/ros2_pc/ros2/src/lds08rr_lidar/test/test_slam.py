import base64
import os
import sys
import time
from pathlib import Path

import pytest

from lds08rr_lidar.runtime import LidarRuntime
from lds08rr_lidar.slam import MapStore, SlamControl, SlamProcess, SlamState, grid_to_dict
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


def test_map_store_commits_both_files_and_loads_them_on_every_start(tmp_path):
    store = MapStore(tmp_path / "slam")
    base_yaml = "slam_toolbox:\n  ros__parameters:\n    mode: mapping\n"
    assert "map_file_name" not in store.params_file(base_yaml).read_text(encoding="utf-8")

    tmp_base = store.directory / "map.saving"
    Path(f"{tmp_base}.posegraph").write_text("graph", encoding="utf-8")
    Path(f"{tmp_base}.data").write_text("data", encoding="utf-8")
    store.commit(tmp_base)
    assert store.exists()
    assert store.saved_at() is not None

    params = store.params_file(base_yaml).read_text(encoding="utf-8")
    assert params == (
        "slam_toolbox:\n  ros__parameters:\n    mode: mapping\n"
        f'    map_file_name: "{tmp_path / "slam" / "map"}"\n'
        "    map_start_at_dock: true\n"
    )

    store.delete()
    store.delete()
    assert not store.exists()
    assert store.saved_at() is None
    assert not (store.directory / "map.data").exists()


def test_map_store_refuses_a_partial_serialized_map(tmp_path):
    store = MapStore(tmp_path)
    tmp_base = tmp_path / "map.saving"
    Path(f"{tmp_base}.data").write_text("data", encoding="utf-8")
    with pytest.raises(FileNotFoundError):
        store.commit(tmp_base)
    assert not store.exists()
    assert Path(f"{tmp_base}.data").exists()


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


def write_serialized_map(filename):
    Path(f"{filename}.posegraph").write_text("graph", encoding="utf-8")
    Path(f"{filename}.data").write_text("data", encoding="utf-8")
    return None


def test_slam_control_saves_then_deletes_and_restarts_slam(tmp_path):
    store = MapStore(tmp_path)
    process = FakeProcess()
    control = SlamControl(store, process, write_serialized_map)

    assert control.save() is None
    assert control.status()["saved_map"]["exists"] is True
    assert (tmp_path / "map.posegraph").read_text(encoding="utf-8") == "graph"
    assert not (tmp_path / "map.saving.posegraph").exists()

    control.delete()
    assert control.status()["saved_map"]["exists"] is False
    assert process.calls == ["stop", "start"]


def test_slam_control_reports_a_failed_save(tmp_path):
    control = SlamControl(MapStore(tmp_path), FakeProcess(), lambda filename: "boom")
    assert control.save() == "boom"
    assert control.status()["saved_map"]["exists"] is False


class FakeControl:
    def __init__(self, error=None):
        self.error = error
        self.deleted = 0

    def status(self):
        return {"process": {"state": "running"}, "saved_map": {"exists": False, "saved_at": None}}

    def save(self):
        return self.error

    def delete(self):
        self.deleted += 1


def make_client(tmp_path, slam, control):
    runtime = LidarRuntime(SourceConfig(mode="demo"), driver_factory=lambda *_: None)
    return make_lidar_app(runtime, slam=slam, control=control, radar_web=tmp_path / "missing").test_client()


def test_slam_actions_are_refused_without_a_running_control(tmp_path):
    client = make_client(tmp_path, SlamState(), None)
    refused = {"ok": False, "error": "SLAM is not running on this service"}
    for path in ("/slam/save", "/slam/delete"):
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
