from __future__ import annotations

import base64
import json
import math
import os
import shutil
import signal
import subprocess
import sys
import time
from array import array
from collections import deque
from pathlib import Path
from threading import Event, Lock, Thread
from typing import Callable

from .localization import localization_command

MAPPING = "mapping"
LOCALIZATION = "localization"
NAVIGATION = "navigation"
MODES = (MAPPING, LOCALIZATION, NAVIGATION)

SNAPSHOT_PERIOD_SEC = 60.0
UNDO_SLOTS = 3
UNDO_DIR_NAME = "undo"
SNAPSHOT_MIN_TRAVEL_M = 0.05  # a snapshot is skipped while odometry moved less than this and turned less than the next
SNAPSHOT_MIN_TURN_RAD = 0.1

MAP_STALE_AFTER_SEC = 10.0  # slam_toolbox publishes /map every 2 s
POSE_STALE_AFTER_SEC = 10.0
ODOM_STALE_AFTER_SEC = 2.0  # the bridge publishes /odom at about 50 Hz
ODOM_RATE_WINDOW_SEC = 2.0
DEFAULT_SLAM_DIR = Path.home() / ".local/share/navbot/slam"
BASE_PARAMS = Path(__file__).with_name("slam_toolbox.yaml")
# slam_toolbox writes .posegraph + .data; map_saver_cli writes .pgm + .yaml for the AMCL localizer.
MAP_SUFFIXES = (".data", ".posegraph", ".pgm", ".yaml")


def grid_to_dict(width: int, height: int, resolution: float, origin_x: float, origin_y: float, data) -> dict:
    """Pack an occupancy grid for the browser: int8 cells, row-major, -1 unknown, base64."""
    return {
        "width": width,
        "height": height,
        "resolution": resolution,
        "origin_x": origin_x,
        "origin_y": origin_y,
        "data": base64.b64encode(array("b", data).tobytes()).decode("ascii"),
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
        self._odom: dict | None = None
        self._odom_at: float | None = None
        self._odom_times: deque[float] = deque()
        self._odom_path_m = 0.0
        self._correction: dict | None = None
        self._latched = False

    def set_grid(self, grid: dict) -> None:
        with self._lock:
            self._grid, self._grid_at = grid, self._clock()

    def set_pose(self, x: float, y: float, yaw: float, age_sec: float = 0.0) -> None:
        with self._lock:
            self._pose, self._pose_at = {"x": x, "y": y, "yaw": yaw}, self._clock() - age_sec

    def set_odom(self, x: float, y: float, yaw: float, linear: float, angular: float) -> None:
        """Latest wheel odometry in the odom frame; also accumulates path length and the message rate."""
        with self._lock:
            now = self._clock()
            if self._odom is not None:
                self._odom_path_m += math.hypot(x - self._odom["x"], y - self._odom["y"])
            self._odom, self._odom_at = {"x": x, "y": y, "yaw": yaw, "linear": linear, "angular": angular}, now
            self._odom_times.append(now)
            while self._odom_times[0] < now - ODOM_RATE_WINDOW_SEC:
                self._odom_times.popleft()

    def set_map_latched(self, latched: bool) -> None:
        """True in localization: the map is fixed, so its age never makes the state "stale"."""
        with self._lock:
            self._latched = latched

    def set_correction(self, x: float, y: float, yaw: float) -> None:
        """map -> odom transform: how far SLAM has moved the robot away from pure wheel odometry."""
        with self._lock:
            self._correction = {"x": x, "y": y, "yaw": yaw}

    def clear(self) -> None:
        with self._lock:
            self._grid = self._grid_at = self._pose = self._pose_at = None
            self._correction = None

    def snapshot(self) -> dict:
        now = self._clock()
        with self._lock:
            grid, grid_at, pose, pose_at = self._grid, self._grid_at, self._pose, self._pose_at
            odom, odom_at, correction = self._odom, self._odom_at, self._correction
            odom_path_m, odom_times, latched = self._odom_path_m, list(self._odom_times), self._latched
        map_age = None if grid_at is None else now - grid_at
        pose_age = None if pose_at is None else now - pose_at
        pose_fresh = pose is not None and pose_age <= POSE_STALE_AFTER_SEC
        if grid is None:
            state = "waiting"
        elif latched:
            state = "localization" if pose_fresh else "no_pose"
        elif map_age > MAP_STALE_AFTER_SEC:
            state = "stale"
        elif not pose_fresh:
            state = "no_pose"
        else:
            state = "mapping"
        odom_age = None if odom_at is None else now - odom_at
        odom_fresh = odom is not None and odom_age <= ODOM_STALE_AFTER_SEC
        odom_rate = None
        if odom_fresh and len(odom_times) > 1 and odom_times[-1] > odom_times[0]:
            odom_rate = (len(odom_times) - 1) / (odom_times[-1] - odom_times[0])
        return {
            "state": state,
            "odom": None if odom is None else {
                **odom, "fresh": odom_fresh, "age_sec": odom_age, "rate_hz": odom_rate, "path_m": odom_path_m,
            },
            "correction": correction if pose_fresh else None,
            "map": grid,
            "pose": pose if pose_fresh else None,
            "map_age_sec": map_age,
            "pose_age_sec": pose_age,
        }


class MapStore:
    """The one saved map: <base>.posegraph + <base>.data (slam_toolbox) and <base>.pgm + <base>.yaml (AMCL)."""

    def __init__(self, directory: Path) -> None:
        self.directory = directory
        self.base = directory / "map"

    def _file(self, suffix: str) -> Path:
        return Path(f"{self.base}{suffix}")

    def exists(self) -> bool:
        return self._file(".posegraph").exists()

    def has_occupancy(self) -> bool:
        return self._file(".posegraph").exists() and self._file(".yaml").exists()

    def occupancy_yaml(self) -> Path:
        return self._file(".yaml")

    def keepout_yaml(self) -> Path | None:
        """The keepout mask next to the map, or None when there is none."""
        path = self.directory / "keepout.yaml"
        return path if path.exists() else None

    def saved_at(self) -> float | None:
        try:
            return self._file(".posegraph").stat().st_mtime
        except FileNotFoundError:
            return None

    def commit(self, tmp_dir: Path) -> None:
        pairs = []
        for suffix in MAP_SUFFIXES:
            tmp = tmp_dir / f"map{suffix}"
            if not tmp.exists():
                raise FileNotFoundError(f"serialized map missing: {tmp}")
            pairs.append((tmp, self._file(suffix)))
        # Keep the old set until all new files are in place so a failure never leaves a mixed set.
        moved: list[tuple[Path, Path | None]] = []  # (final, backup of the old file or None)
        try:
            for tmp, final in pairs:
                backup = None
                if final.exists():
                    backup = Path(f"{final}.old")
                    os.replace(final, backup)
                moved.append((final, backup))
                os.replace(tmp, final)
        except OSError:
            for final, backup in reversed(moved):
                if backup is None:
                    final.unlink(missing_ok=True)
                else:
                    os.replace(backup, final)
            raise
        for _, backup in moved:
            if backup is not None:
                backup.unlink(missing_ok=True)

    def delete(self) -> None:
        # The two files that define exists() and has_occupancy() go first, so a failure part way
        # never leaves them present with their data files already gone.
        for suffix in (".posegraph", ".yaml", ".data", ".pgm"):
            self._file(suffix).unlink(missing_ok=True)

    def params_file(self, base_yaml: str, restore: tuple[Path, dict] | None = None) -> Path:
        """Params for the next mapping start. restore: (snapshot base, map pose) to start from an undo snapshot."""
        self.directory.mkdir(parents=True, exist_ok=True)
        text = base_yaml if base_yaml.endswith("\n") else base_yaml + "\n"
        if restore is not None:
            snapshot_base, pose = restore
            text += (
                f'    map_file_name: {json.dumps(str(snapshot_base))}\n'
                f'    map_start_pose: [{pose["x"]!r}, {pose["y"]!r}, {pose["yaw"]!r}]\n'
            )
        elif self.exists():
            # map_start_at_dock: the robot is assumed to start where the saved map began.
            text += f'    map_file_name: {json.dumps(str(self.base))}\n    map_start_at_dock: true\n'
        path = self.directory / "slam_params.yaml"
        path.write_text(text, encoding="utf-8")
        return path


def wrap_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def restored_pose(map_pose: dict, snapshot_odom: dict, odom: dict) -> dict:
    """Robot pose in the snapshot's map frame: the map pose advanced by the odometry travelled since the snapshot."""
    yaw0 = snapshot_odom["yaw"]
    dx, dy = odom["x"] - snapshot_odom["x"], odom["y"] - snapshot_odom["y"]
    # The travel in the snapshot's body frame: rotate the world offset by -yaw0.
    forward = math.cos(yaw0) * dx + math.sin(yaw0) * dy
    left = -math.sin(yaw0) * dx + math.cos(yaw0) * dy
    cos_map, sin_map = math.cos(map_pose["yaw"]), math.sin(map_pose["yaw"])
    return {
        "x": map_pose["x"] + cos_map * forward - sin_map * left,
        "y": map_pose["y"] + sin_map * forward + cos_map * left,
        "yaw": wrap_angle(map_pose["yaw"] + wrap_angle(odom["yaw"] - yaw0)),
    }


def _moved_since(previous_odom: dict, odom: dict) -> bool:
    travel = math.hypot(odom["x"] - previous_odom["x"], odom["y"] - previous_odom["y"])
    turn = abs(wrap_angle(odom["yaw"] - previous_odom["yaw"]))
    return travel >= SNAPSHOT_MIN_TRAVEL_M or turn >= SNAPSHOT_MIN_TURN_RAD


class UndoRing:
    """The newest UNDO_SLOTS posegraph snapshots taken while mapping, in <directory>/snap-<n>.*.

    A snapshot is complete when its .json exists; it is written last, so a half-written one is never listed.
    """

    def __init__(
        self,
        directory: Path,
        serialize: Callable[[str], str | None],
        slam_state,
        clock: Callable[[], float] = time.time,
        period: float = SNAPSHOT_PERIOD_SEC,
    ) -> None:
        self.directory = directory
        self._serialize = serialize
        self._slam_state = slam_state
        self._clock = clock
        self._period = period
        self._lock = Lock()
        self._timer: Thread | None = None
        self._timer_stop: Event | None = None

    def start_timer(self) -> None:
        if self._timer is not None:
            return
        self._timer_stop = Event()
        self._timer = Thread(target=self._run, args=(self._timer_stop,), name="undo-snapshots", daemon=True)
        self._timer.start()

    def stop_timer(self) -> None:
        if self._timer is None:
            return
        self._timer_stop.set()
        self._timer.join()
        self._timer, self._timer_stop = None, None

    def take(self) -> str | None:
        """Saves a snapshot unless the map, the pose or the odometry is missing or the robot hardly moved.

        Returns the serialize error text when a snapshot failed; the older snapshots stay in place.
        """
        with self._lock:
            poses = self._current_poses()
            if poses is None:
                return None
            map_pose, odom = poses
            newest = self._list()[:1]
            if newest and not _moved_since(newest[0]["odom_pose"], odom):
                return None
            index = newest[0]["n"] + 1 if newest else 1
            tmp = self.directory / "tmp"
            shutil.rmtree(tmp, ignore_errors=True)
            tmp.mkdir(parents=True)
            try:
                error = self._serialize(str(tmp / "snap"))
                if error is not None:
                    return error
                for suffix in (".posegraph", ".data"):
                    if not (tmp / f"snap{suffix}").exists():
                        return f"serialized snapshot missing snap{suffix}"
                meta = tmp / "meta.json"
                meta.write_text(
                    json.dumps({"saved_at": self._clock(), "map_pose": map_pose, "odom_pose": odom}),
                    encoding="utf-8",
                )
                base = self.directory / f"snap-{index}"
                os.replace(tmp / "snap.posegraph", f"{base}.posegraph")
                os.replace(tmp / "snap.data", f"{base}.data")
                os.replace(meta, f"{base}.json")
            except OSError as exc:
                return str(exc)
            finally:
                shutil.rmtree(tmp, ignore_errors=True)
            for item in self._list()[UNDO_SLOTS:]:
                self._remove(item["n"])
            return None

    def get(self, index: int) -> dict | None:
        """The snapshot at a 1-based index, newest first."""
        with self._lock:
            items = self._list()
            return items[index - 1] if 1 <= index <= len(items) else None

    def describe(self) -> list[dict]:
        with self._lock:
            now = self._clock()
            return [{"index": i, "age_sec": now - item["saved_at"]} for i, item in enumerate(self._list(), start=1)]

    def drop_newer_than(self, n: int) -> None:
        with self._lock:
            for item in self._list():
                if item["n"] > n:
                    self._remove(item["n"])

    def clear(self) -> None:
        with self._lock:
            shutil.rmtree(self.directory, ignore_errors=True)

    def _run(self, stop: Event) -> None:
        while not stop.wait(self._period):
            try:
                error = self.take()
            except Exception as exc:  # the timer must outlive a failed snapshot
                error = f"{type(exc).__name__}: {exc}"
            if error is not None:
                print(f"undo snapshot failed: {error}", file=sys.stderr, flush=True)

    def _current_poses(self) -> tuple[dict, dict] | None:
        if self._slam_state is None:
            return None
        shown = self._slam_state.snapshot()
        pose, odom = shown["pose"], shown["odom"]
        if shown["map"] is None or pose is None or odom is None or not odom["fresh"]:
            return None
        return {k: pose[k] for k in ("x", "y", "yaw")}, {k: odom[k] for k in ("x", "y", "yaw")}

    def _list(self) -> list[dict]:
        items = []
        for path in self.directory.glob("snap-*.json"):
            try:
                n = int(path.stem.removeprefix("snap-"))
                meta = json.loads(path.read_text(encoding="utf-8"))
                items.append({
                    "n": n,
                    "base": self.directory / f"snap-{n}",
                    "saved_at": meta["saved_at"],
                    "map_pose": meta["map_pose"],
                    "odom_pose": meta["odom_pose"],
                })
            except (ValueError, OSError, KeyError, TypeError):
                continue
        return sorted(items, key=lambda item: item["n"], reverse=True)

    def _remove(self, n: int) -> None:
        base = self.directory / f"snap-{n}"
        for suffix in (".json", ".posegraph", ".data"):  # the .json goes first, so a partial delete is never listed
            Path(f"{base}{suffix}").unlink(missing_ok=True)


class SlamProcess:
    """Supervises one slam_toolbox launch; restarts it after it exits until stop()."""

    def __init__(self, command: Callable[[], list[str]], log_path: Path, restart_delay: float = 5.0) -> None:
        self._command = command
        self._log_path = log_path
        self._restart_delay = restart_delay
        self._lock = Lock()
        self._state = "stopped"
        self._pid: int | None = None
        self._exit_code: int | None = None
        self._restarts = 0
        self._error: str | None = None
        self._stop_event: Event | None = None
        self._child: subprocess.Popen | None = None
        self._thread: Thread | None = None

    def start(self) -> None:
        with self._lock:
            if self._thread is not None:
                return
            self._stop_event = Event()
            self._state = "running"
            self._thread = Thread(target=self._supervise, args=(self._stop_event,), name="slam-supervisor", daemon=True)
            self._thread.start()

    def stop(self, timeout: float = 10.0) -> None:
        with self._lock:
            stop_event, thread, child = self._stop_event, self._thread, self._child
            if stop_event is None:
                return
            stop_event.set()
        if child is not None:
            _signal_group(child, signal.SIGINT)
        if thread is not None:
            thread.join(timeout)
            if thread.is_alive() and child is not None:
                _signal_group(child, signal.SIGKILL)
            thread.join()
        with self._lock:
            self._thread, self._stop_event, self._state = None, None, "stopped"

    def status(self) -> dict:
        with self._lock:
            return {
                "state": self._state,
                "pid": self._pid,
                "exit_code": self._exit_code,
                "restarts": self._restarts,
                "error": self._error,
            }

    def _supervise(self, stop_event: Event) -> None:
        while not stop_event.is_set():
            child = None
            try:
                child = self._spawn(stop_event)
            except Exception as exc:  # the supervisor must outlive any failure to build or start the command
                with self._lock:
                    self._error = f"{type(exc).__name__}: {exc}"
            if child is not None:
                exit_code = child.wait()
                with self._lock:
                    self._exit_code, self._child, self._pid = exit_code, None, None
            with self._lock:
                if stop_event.is_set():
                    break
                self._state = "restarting"
                self._restarts += 1
            stop_event.wait(self._restart_delay)

    def _spawn(self, stop_event: Event) -> subprocess.Popen | None:
        command = self._command()
        self._log_path.parent.mkdir(parents=True, exist_ok=True)
        with self._log_path.open("ab") as log, self._lock:
            if stop_event.is_set():
                return None
            child = subprocess.Popen(
                command,
                stdout=log,
                stderr=subprocess.STDOUT,
                stdin=subprocess.DEVNULL,
                start_new_session=True,
            )
            self._child, self._pid, self._error, self._state = child, child.pid, None, "running"
            return child


def _signal_group(child: subprocess.Popen, sig: int) -> None:
    try:
        os.killpg(child.pid, sig)
    except ProcessLookupError:
        pass


def export_occupancy(base: Path, timeout: float = 20.0) -> str | None:
    """Writes <base>.pgm and <base>.yaml from the live /map; the YAML names the image by its bare file name."""
    command = [
        "ros2", "run", "nav2_map_server", "map_saver_cli", "-f", str(base),
        # slam_toolbox publishes /map every 2 s and delivery of a big map takes up to 3 s; the saver waits only 2 s by default.
        "--ros-args", "-p", "map_subscribe_transient_local:=true", "-p", "save_map_timeout:=10.0",
    ]
    try:
        child = subprocess.Popen(
            command,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            stdin=subprocess.DEVNULL,
            start_new_session=True,
            text=True,
        )
    except OSError as exc:
        return f"map export could not start: {exc}"
    try:
        output, _ = child.communicate(timeout=timeout)
    except subprocess.TimeoutExpired:
        _signal_group(child, signal.SIGKILL)
        child.communicate()
        return "map export timed out"
    if child.returncode != 0:
        return f"map export failed with exit code {child.returncode}: {output.strip()[-500:]}"
    for suffix in (".pgm", ".yaml"):
        if not Path(f"{base}{suffix}").exists():
            return f"map export wrote no {base.name}{suffix}"
    return None


class SlamControl:
    """Mode, save, delete and status for the single map; one lock serializes them.

    Mapping runs slam_toolbox, localization runs map_server + AMCL on the saved occupancy image.
    Only one of them runs at a time, because both publish map -> odom.
    """

    def __init__(
        self,
        store: MapStore,
        process: SlamProcess,
        serialize: Callable[[str], str | None],
        export: Callable[[Path], str | None] = export_occupancy,
        localizer=None,
        tracker=None,
        slam_state=None,
        mapping_command: Callable[[tuple[Path, dict] | None], list[str]] | None = None,
        snapshot_period: float = SNAPSHOT_PERIOD_SEC,
        clock: Callable[[], float] = time.time,
        supervisor=None,
        nav=None,
        nav_process: SlamProcess | None = None,
        localization_command: Callable[[Path], list[str]] = localization_command,
    ) -> None:
        self._store = store
        self._process = process
        self._serialize = serialize
        self._export = export
        self._localizer = localizer
        self._tracker = tracker
        self._slam_state = slam_state
        self._mapping_command = mapping_command
        self._undo = UndoRing(store.directory / UNDO_DIR_NAME, serialize, slam_state, clock=clock, period=snapshot_period)
        self._pending_restore: tuple[Path, dict] | None = None  # consumed by the next mapping command()
        self._mode = MAPPING  # start() picks the boot mode
        self._supervisor = supervisor
        self._nav = nav
        self._nav_process = nav_process
        self._localization_command = localization_command
        self._goal_count = 0
        self._goal: dict | None = None  # the newest goal; its Nav2 callbacks write into this dict
        self._lock = Lock()

    @property
    def mode(self) -> str:
        return self._mode

    def initial_mode(self) -> str:
        return LOCALIZATION if self._store.has_occupancy() else MAPPING

    def command(self) -> list[str]:
        # The supervisor calls this before every spawn, so each AMCL start, also a restart after a crash, searches again.
        if self._mode in (LOCALIZATION, NAVIGATION):
            if self._localizer is not None:
                self._localizer.start()
            return self._localization_command(self._store.occupancy_yaml())
        restore, self._pending_restore = self._pending_restore, None  # one-shot: a crash restart starts the saved map
        return self._mapping_command(restore)

    def start(self) -> None:
        with self._lock:
            self._undo.clear()  # leftovers of an earlier run
            self._enter(self.initial_mode())

    def stop(self) -> None:
        with self._lock:
            self._halt()
            self._undo.clear()

    def set_mode(self, mode: str) -> str | None:
        with self._lock:
            if mode not in MODES:
                return f"unknown mode {mode!r}; use mapping, localization or navigation"
            if mode == self._mode:
                return None
            if mode == NAVIGATION:
                return self._enter_navigation()
            if mode == LOCALIZATION and not self._store.has_occupancy():
                return "No occupancy map to localize on; save the map once in mapping mode first"
            if mode == LOCALIZATION and self._mode == NAVIGATION:
                self._leave_navigation()  # the localization launch keeps running
                self._mode = LOCALIZATION
                return None
            restore = None
            if mode == MAPPING and self._store.exists():
                # Mapping continues the saved map from the localized pose; a wrong start pose would break the map.
                pose = self._localized_pose()
                if pose is None:
                    return "Not localized on the saved map; wait for 'localized' or press Locate again"
                restore = (self._store.base, pose)
            self._halt()
            self._undo.clear()
            self._pending_restore = restore
            self._enter(mode)
            return None

    def undo(self, index: int) -> str | None:
        """Restarts mapping from undo snapshot `index` (1 = newest); the map after it is discarded."""
        with self._lock:
            if self._mode != MAPPING:
                return "Undo needs mapping mode"
            snapshot = self._undo.get(index)
            if snapshot is None:
                return f"No undo snapshot {index}"
            odom = self._current_odom()
            if odom is None:
                return "No current odometry; cannot undo"
            pose = restored_pose(snapshot["map_pose"], snapshot["odom_pose"], odom)
            self._halt()
            self._pending_restore = (snapshot["base"], pose)
            self._undo.drop_newer_than(snapshot["n"])
            self._enter(MAPPING)
            return None

    def locate(self) -> str | None:
        with self._lock:
            if self._mode != LOCALIZATION:
                return "Locate needs localization mode; switch to localization first"
            if self._localizer is None:
                return "localization is not configured"
            self._localizer.start()
            return None

    def save_refusal(self) -> str | None:
        """Why save() would refuse right now, or None when it may run."""
        if self._mode != MAPPING:
            return "Save needs mapping mode; switch to mapping first"
        return None

    def save(self) -> str | None:
        with self._lock:
            refusal = self.save_refusal()
            if refusal is not None:
                return refusal
            tmp = self._store.directory / f"map.saving.{time.time_ns()}"
            try:
                tmp.mkdir(parents=True)
                error = self._serialize(str(tmp / "map"))
                if error is None:
                    error = self._export(tmp / "map")
                if error is not None:
                    return error
                try:
                    self._store.commit(tmp)
                except OSError as exc:
                    return str(exc)
                self._undo.clear()
                return None
            except OSError as exc:
                return str(exc)
            finally:
                shutil.rmtree(tmp, ignore_errors=True)

    def delete(self) -> str | None:
        with self._lock:
            self._halt()
            self._undo.clear()
            try:
                self._store.delete()
            finally:
                self._enter(MAPPING)
            return None

    def reload(self) -> str | None:
        """Drops the current map and restarts the active launch on the saved one; navigation falls back to localization."""
        with self._lock:
            if not self._store.exists():
                return "No saved map to load"
            mode = LOCALIZATION if self._mode == NAVIGATION else self._mode
            self._halt()
            self._undo.clear()
            self._enter(mode)
            return None

    def nav_goal(self, x: float, y: float, yaw: float) -> str | None:
        """Starts a Nav2 goal in the map frame; None or the refusal."""
        with self._lock:
            if self._mode != NAVIGATION:
                return "Goals need navigation mode"
            if not all(math.isfinite(value) for value in (x, y, yaw)):
                return "x, y and yaw must be finite numbers"
            refusal = self._supervisor.request_goal()
            if refusal is not None:
                return refusal
            goal = {"x": x, "y": y, "yaw": yaw, "distance_remaining_m": None, "outcome": None, "error": ""}
            error = self._nav.send_nav_goal(
                x, y, yaw,
                on_feedback=lambda distance: goal.update(distance_remaining_m=distance),
                on_done=lambda outcome, text: self._goal_done(goal, outcome, text),
            )
            if error is not None:
                self._supervisor.goal_ended()
                return error
            self._goal_count += 1
            self._goal = goal
            return None

    def nav_cancel(self) -> str | None:
        with self._lock:
            if self._goal is None or self._goal["outcome"] is not None:
                return "No goal is running"
            self._nav.cancel_nav_goal()
            return None

    def nav_stop(self) -> None:
        with self._lock:
            if self._mode == NAVIGATION:
                self._supervisor.stop()
                self._nav.cancel_nav_goal()

    def nav_resume(self) -> str | None:
        with self._lock:
            if self._mode != NAVIGATION:
                return "Resume needs navigation mode"
            return self._supervisor.resume()

    def status(self) -> dict:
        return {
            "mode": self._mode,
            "process": self._process.status(),
            "saved_map": {"exists": self._store.exists(), "saved_at": self._store.saved_at()},
            "directory": str(self._store.directory),
            "localization": None if self._tracker is None else self._tracker.snapshot(),
            "undo": {"snapshots": self._undo.describe() if self._mode == MAPPING else []},
            "nav": self._nav_status(),
        }

    def _nav_status(self) -> dict | None:
        if self._mode != NAVIGATION:
            return None
        goal = self._goal or {}
        return {
            "supervisor": self._supervisor.snapshot(),
            "goal_id": self._goal_count,
            "goal": {key: goal[key] for key in ("x", "y", "yaw")} if goal else None,
            "distance_remaining_m": goal.get("distance_remaining_m"),
            "outcome": goal.get("outcome"),
            "error": goal.get("error", ""),
            "plan": self._nav.nav_plan(),
            "process": self._nav_process.status(),
        }

    def _goal_done(self, goal: dict, outcome: str, error: str) -> None:
        goal["outcome"], goal["error"] = outcome, error
        # Nav2 reports a replaced goal too; only the current goal may move the supervisor.
        if goal is self._goal:
            self._supervisor.goal_ended()

    def _enter_navigation(self) -> str | None:
        if self._supervisor is None or self._nav is None or self._nav_process is None:
            return "Navigation is not available on this service"
        if self._mode == MAPPING:
            return "Switch to localization first, then to navigation"
        if self._tracker is None or self._tracker.snapshot()["state"] != "localized":
            return "Navigation needs a localized pose; wait or press Locate"
        self._mode = NAVIGATION
        self._nav_process.start()
        return None

    def _leave_navigation(self) -> None:
        goal, self._goal = self._goal, None
        if goal is not None and goal["outcome"] is None:
            self._supervisor.stop()
            self._nav.cancel_nav_goal()
        self._nav_process.stop()

    def _current_odom(self) -> dict | None:
        if self._slam_state is None:
            return None
        odom = self._slam_state.snapshot()["odom"]
        if odom is None or not odom["fresh"]:
            return None
        return {k: odom[k] for k in ("x", "y", "yaw")}

    def _localized_pose(self) -> dict | None:
        """The current map -> base_link pose while AMCL reports 'localized', else None."""
        if self._tracker is None or self._slam_state is None:
            return None
        if self._tracker.snapshot()["state"] != "localized":
            return None
        pose = self._slam_state.snapshot()["pose"]
        return None if pose is None else {k: pose[k] for k in ("x", "y", "yaw")}

    def _halt(self) -> None:
        self._undo.stop_timer()
        self._pending_restore = None  # a restore not yet consumed by a start is dropped with its launch
        if self._mode == NAVIGATION:
            self._leave_navigation()
        self._process.stop()
        if self._localizer is not None:
            self._localizer.stop()

    def _enter(self, mode: str) -> None:
        self._mode = mode
        if self._slam_state is not None:
            self._slam_state.clear()
            self._slam_state.set_map_latched(mode == LOCALIZATION)
        if self._tracker is not None:
            self._tracker.reset()
        self._process.start()
        if mode == MAPPING:
            self._undo.start_timer()


def slam_command(params_file: Path) -> list[str]:
    return [
        "ros2", "launch", "slam_toolbox", "online_async_launch.py",
        f"slam_params_file:={params_file}", "use_sim_time:=false",
    ]
