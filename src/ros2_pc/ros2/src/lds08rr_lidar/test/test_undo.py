import math
import re
import time
from pathlib import Path

import pytest

pytest.importorskip("flask")

from lds08rr_lidar.localization import LocalizationTracker
from lds08rr_lidar.slam import (
    MapStore,
    SlamControl,
    SlamState,
    UndoRing,
    grid_to_dict,
    restored_pose,
    slam_command,
)

BASE_YAML = "slam_toolbox:\n  ros__parameters:\n    mode: mapping\n"


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
        time.sleep(0.01)
    raise AssertionError("condition was not reached in time")


def write_serialized(base):
    Path(f"{base}.posegraph").write_text("graph", encoding="utf-8")
    Path(f"{base}.data").write_text("data", encoding="utf-8")
    return None


def write_occupancy(base):
    Path(f"{base}.pgm").write_text("pgm", encoding="utf-8")
    Path(f"{base}.yaml").write_text(f"image: {Path(base).name}.pgm\n", encoding="utf-8")
    return None


def fresh_slam(clock):
    slam = SlamState(clock=clock)
    slam.set_grid(grid_to_dict(1, 1, 0.025, 0.0, 0.0, [0]))
    return slam


def feed(slam, pose, odom):
    # Also the map: a control start or mode switch clears SlamState, so the grid is set after it.
    slam.set_grid(grid_to_dict(1, 1, 0.025, 0.0, 0.0, [0]))
    slam.set_pose(*pose)
    slam.set_odom(odom[0], odom[1], odom[2], 0.0, 0.0)


def snapshot_count(control):
    return len(control.status()["undo"]["snapshots"])


class RecordingProcess:
    """Records each start with the command the control builds for it, as the supervisor does."""

    def __init__(self, events):
        self.events = events
        self.command = None

    def start(self):
        self.events.append(("process start", self.command()))

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


def mapping_control(tmp_path, events, clock, slam, period=0.01):
    store = MapStore(tmp_path)
    process = RecordingProcess(events)
    control = SlamControl(
        store, process, write_serialized, write_occupancy,
        localizer=RecordingLocalizer(events), tracker=LocalizationTracker(), slam_state=slam,
        mapping_command=lambda restore: slam_command(store.params_file(BASE_YAML, restore)),
        snapshot_period=period, clock=clock,
    )
    process.command = control.command
    return control, store


def params_of(command):
    params = next(arg for arg in command if arg.startswith("slam_params_file:=")).split(":=", 1)[1]
    return Path(params).read_text(encoding="utf-8")


def test_ring_keeps_the_three_newest(tmp_path):
    clock = FakeClock(10.0)
    slam = fresh_slam(clock)
    slam.set_pose(0.0, 0.0, 0.0)
    ring = UndoRing(tmp_path / "undo", write_serialized, slam, clock=clock, period=60)
    for step in range(4):
        slam.set_odom(float(step), 0.0, 0.0, 0.0, 0.0)  # one metre per snapshot
        clock.now += 1.0
        assert ring.take() is None

    names = sorted(path.name for path in (tmp_path / "undo").glob("snap-*.json"))
    assert names == ["snap-2.json", "snap-3.json", "snap-4.json"]
    assert [item["age_sec"] for item in ring.describe()] == [0.0, 1.0, 2.0]
    assert ring.get(1)["n"] == 4 and ring.get(3)["n"] == 2 and ring.get(4) is None


def test_ring_skips_a_snapshot_while_the_robot_hardly_moved(tmp_path):
    clock = FakeClock(0.0)
    slam = fresh_slam(clock)
    feed(slam, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0))
    ring = UndoRing(tmp_path / "undo", write_serialized, slam, clock=clock, period=60)

    assert ring.take() is None
    assert len(ring.describe()) == 1
    slam.set_odom(0.04, 0.0, 0.09, 0.0, 0.0)  # 4 cm and 0.09 rad: both below the limits
    assert ring.take() is None
    assert len(ring.describe()) == 1
    slam.set_odom(0.04, 0.0, 0.2, 0.0, 0.0)  # turned 0.2 rad since the last snapshot
    assert ring.take() is None
    assert len(ring.describe()) == 2
    slam.set_odom(0.06, 0.0, 0.2, 0.0, 0.0)  # 2 cm from the newest snapshot
    assert ring.take() is None
    assert len(ring.describe()) == 2


def test_ring_takes_no_snapshot_without_a_map_or_a_pose(tmp_path):
    clock = FakeClock(0.0)
    slam = SlamState(clock=clock)
    ring = UndoRing(tmp_path / "undo", write_serialized, slam, clock=clock, period=60)
    slam.set_odom(0.0, 0.0, 0.0, 0.0, 0.0)
    assert ring.take() is None
    slam.set_grid(grid_to_dict(1, 1, 0.025, 0.0, 0.0, [0]))
    assert ring.take() is None
    assert ring.describe() == []
    slam.set_pose(0.0, 0.0, 0.0)
    assert ring.take() is None
    assert len(ring.describe()) == 1


def test_failed_snapshot_keeps_the_good_one(tmp_path):
    clock = FakeClock(0.0)
    slam = fresh_slam(clock)
    feed(slam, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0))
    failing = {"on": False}

    def serialize(base):
        Path(f"{base}.posegraph").write_text("half", encoding="utf-8")  # a partial write before the failure
        if failing["on"]:
            return "serialize_map timed out"
        return write_serialized(base)

    ring = UndoRing(tmp_path / "undo", serialize, slam, clock=clock, period=60)
    assert ring.take() is None
    failing["on"] = True
    slam.set_odom(1.0, 0.0, 0.0, 0.0, 0.0)
    assert ring.take() == "serialize_map timed out"

    assert len(ring.describe()) == 1
    assert (tmp_path / "undo" / "snap-1.posegraph").read_text(encoding="utf-8") == "graph"
    assert not (tmp_path / "undo" / "tmp").exists()


def test_timer_takes_snapshots_until_it_is_stopped(tmp_path):
    slam = fresh_slam(FakeClock(0.0))
    feed(slam, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0))
    ring = UndoRing(tmp_path / "undo", write_serialized, slam, period=0.01)
    ring.start_timer()
    wait_until(lambda: len(ring.describe()) == 1)
    ring.stop_timer()
    assert ring._timer is None
    count = len(ring.describe())
    time.sleep(0.05)
    assert len(ring.describe()) == count


def test_pose_maths_against_literal_numbers():
    quarter = math.pi / 2
    # Robot at map (1, 0, 0 deg); odometry moved from (0, 0, 0) to (0, 1, 90 deg): one metre left-ahead, turned 90 deg.
    pose = restored_pose({"x": 1.0, "y": 0.0, "yaw": 0.0}, {"x": 0.0, "y": 0.0, "yaw": 0.0}, {"x": 0.0, "y": 1.0, "yaw": quarter})
    assert pose == pytest.approx({"x": 1.0, "y": 1.0, "yaw": quarter})
    # Map heading 90 deg: a metre forward in odometry is a metre along +y in the map.
    pose = restored_pose({"x": 0.0, "y": 0.0, "yaw": quarter}, {"x": 0.0, "y": 0.0, "yaw": 0.0}, {"x": 1.0, "y": 0.0, "yaw": 0.0})
    assert pose == pytest.approx({"x": 0.0, "y": 1.0, "yaw": quarter})
    # Heading wraps into (-pi, pi]: 3.0 + 0.5 = 3.5 -> 3.5 - 2 pi.
    pose = restored_pose({"x": 0.0, "y": 0.0, "yaw": 3.0}, {"x": 0.0, "y": 0.0, "yaw": 0.0}, {"x": 0.0, "y": 0.0, "yaw": 0.5})
    assert pose["yaw"] == pytest.approx(3.5 - 2 * math.pi)


def test_timer_snapshots_while_mapping_and_status_reports_ages(tmp_path):
    clock = FakeClock(100.0)
    events = []
    slam = fresh_slam(clock)
    control, _ = mapping_control(tmp_path, events, clock, slam)
    control.start()
    feed(slam, (1.0, 0.0, 0.0), (0.0, 0.0, 0.0))
    wait_until(lambda: snapshot_count(control) == 1)

    clock.now = 112.5  # odometry last seen at 100 is now stale: no further snapshots
    assert control.status()["undo"] == {"snapshots": [{"index": 1, "age_sec": 12.5}]}
    control.stop()
    assert control.status()["undo"] == {"snapshots": []}


def test_undo_is_refused_in_localization_for_a_bad_index_and_without_odometry(tmp_path):
    clock = FakeClock(100.0)
    events = []
    slam = fresh_slam(clock)
    control, _ = mapping_control(tmp_path, events, clock, slam)
    control.start()
    feed(slam, (1.0, 0.0, 0.0), (0.0, 0.0, 0.0))
    wait_until(lambda: snapshot_count(control) == 1)

    assert control.undo(2) == "No undo snapshot 2"
    clock.now = 103.0  # odometry is older than its freshness limit
    assert control.undo(1) == "No current odometry; cannot undo"

    assert control.save() is None
    assert control.set_mode("localization") is None
    assert control.undo(1) == "Undo needs mapping mode"
    assert control.status()["undo"] == {"snapshots": []}


def test_undo_restarts_from_the_snapshot_pose_and_drops_newer_snapshots(tmp_path):
    clock = FakeClock(0.0)
    events = []
    slam = fresh_slam(clock)
    control, store = mapping_control(tmp_path, events, clock, slam)
    control.start()
    feed(slam, (1.0, 0.0, 0.0), (0.0, 0.0, 0.0))
    wait_until(lambda: snapshot_count(control) == 1)  # snapshot 1 (older)
    slam.set_odom(0.0, 1.0, math.pi / 2, 0.0, 0.0)
    wait_until(lambda: snapshot_count(control) == 2)  # snapshot 2 (newer)
    events.clear()

    assert control.undo(2) is None

    assert events[0] == ("process stop",)
    assert events[1] == ("localizer stop",)
    assert events[-1][0] == "process start"
    text = params_of(events[-1][1])
    assert f'map_file_name: "{tmp_path / "undo" / "snap-1"}"' in text
    assert "map_start_at_dock" not in text
    match = re.search(r"map_start_pose: \[(.+), (.+), (.+)\]", text)
    assert [float(value) for value in match.groups()] == pytest.approx([1.0, 1.0, math.pi / 2])
    assert [item["index"] for item in control.status()["undo"]["snapshots"]] == [1]
    assert not (tmp_path / "undo" / "snap-2.json").exists()

    # One-shot: the next command (a crash restart) starts the saved map as before, and here there is none.
    text = params_of(control.command())
    assert "map_file_name" not in text and "map_start_pose" not in text
    control.stop()


def test_saving_the_map_deletes_the_snapshots_and_keeps_the_saved_files(tmp_path):
    clock = FakeClock(100.0)
    slam = fresh_slam(clock)
    control, _ = mapping_control(tmp_path, [], clock, slam)
    control.start()
    feed(slam, (1.0, 0.0, 0.0), (0.0, 0.0, 0.0))
    wait_until(lambda: snapshot_count(control) == 1)
    clock.now = 103.0  # stale odometry: the timer takes no new snapshot during the save

    assert control.save() is None
    assert not (tmp_path / "undo").exists()
    assert (tmp_path / "map.posegraph").exists() and (tmp_path / "map.yaml").exists()
    control.stop()


def test_switching_to_localization_deletes_the_snapshots(tmp_path):
    clock = FakeClock(100.0)
    slam = fresh_slam(clock)
    control, store = mapping_control(tmp_path, [], clock, slam)
    assert control.save() is None
    control.start()
    assert control.mode == "localization"
    assert control.set_mode("mapping") is None
    feed(slam, (1.0, 0.0, 0.0), (0.0, 0.0, 0.0))
    wait_until(lambda: snapshot_count(control) == 1)
    clock.now = 103.0

    assert control.set_mode("localization") is None
    assert not (tmp_path / "undo").exists()
    control.stop()


def test_delete_stop_reload_and_start_clear_the_snapshots(tmp_path):
    clock = FakeClock(100.0)
    slam = fresh_slam(clock)
    control, _ = mapping_control(tmp_path, [], clock, slam)
    control.start()
    feed(slam, (1.0, 0.0, 0.0), (0.0, 0.0, 0.0))
    wait_until(lambda: snapshot_count(control) == 1)
    clock.now = 103.0
    control.delete()
    assert not (tmp_path / "undo").exists()

    assert control.save() is None
    control.set_mode("mapping")
    feed(slam, (1.0, 0.0, 0.0), (0.0, 0.0, 0.0))
    wait_until(lambda: snapshot_count(control) == 1)
    clock.now = 106.0
    assert control.reload() is None
    assert not (tmp_path / "undo").exists()

    control.set_mode("mapping")
    feed(slam, (1.0, 0.0, 0.0), (0.0, 0.0, 0.0))
    wait_until(lambda: snapshot_count(control) == 1)
    clock.now = 109.0
    control.stop()
    assert not (tmp_path / "undo").exists()

    leftover = tmp_path / "undo"
    leftover.mkdir()
    (leftover / "snap-1.json").write_text("{}", encoding="utf-8")
    control.start()
    assert not leftover.exists()
    control.stop()

