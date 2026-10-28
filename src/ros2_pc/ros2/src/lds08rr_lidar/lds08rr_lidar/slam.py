from __future__ import annotations

import base64
import json
import math
import os
import shutil
import signal
import subprocess
import time
from array import array
from collections import deque
from pathlib import Path
from threading import Event, Lock, Thread
from typing import Callable

from .localization import localization_command

MAPPING = "mapping"
LOCALIZATION = "localization"
MODES = (MAPPING, LOCALIZATION)

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

    def params_file(self, base_yaml: str) -> Path:
        self.directory.mkdir(parents=True, exist_ok=True)
        text = base_yaml if base_yaml.endswith("\n") else base_yaml + "\n"
        if self.exists():
            # map_start_at_dock: the robot is assumed to start where the saved map began.
            text += f'    map_file_name: {json.dumps(str(self.base))}\n    map_start_at_dock: true\n'
        path = self.directory / "slam_params.yaml"
        path.write_text(text, encoding="utf-8")
        return path


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
        "--ros-args", "-p", "map_subscribe_transient_local:=true",
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
        mapping_command: Callable[[], list[str]] | None = None,
    ) -> None:
        self._store = store
        self._process = process
        self._serialize = serialize
        self._export = export
        self._localizer = localizer
        self._tracker = tracker
        self._slam_state = slam_state
        self._mapping_command = mapping_command
        self._mode = MAPPING  # start() picks the boot mode
        self._lock = Lock()

    @property
    def mode(self) -> str:
        return self._mode

    def initial_mode(self) -> str:
        return LOCALIZATION if self._store.has_occupancy() else MAPPING

    def command(self) -> list[str]:
        # The supervisor calls this before every spawn, so each AMCL start, also a restart after a crash, searches again.
        if self._mode == LOCALIZATION:
            if self._localizer is not None:
                self._localizer.start()
            return localization_command(self._store.occupancy_yaml())
        return self._mapping_command()

    def start(self) -> None:
        with self._lock:
            self._enter(self.initial_mode())

    def stop(self) -> None:
        with self._lock:
            self._halt()

    def set_mode(self, mode: str) -> str | None:
        with self._lock:
            if mode not in MODES:
                return f"unknown mode {mode!r}; use mapping or localization"
            if mode == self._mode:
                return None
            if mode == LOCALIZATION and not self._store.has_occupancy():
                return "No occupancy map to localize on; save the map once in mapping mode first"
            self._halt()
            self._enter(mode)
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
                return None
            except OSError as exc:
                return str(exc)
            finally:
                shutil.rmtree(tmp, ignore_errors=True)

    def delete(self) -> None:
        with self._lock:
            self._halt()
            try:
                self._store.delete()
            finally:
                self._enter(MAPPING)

    def reload(self) -> str | None:
        """Drops the current map and restarts the active launch on the saved one."""
        with self._lock:
            if not self._store.exists():
                return "No saved map to load"
            self._halt()
            self._enter(self._mode)
            return None

    def status(self) -> dict:
        return {
            "mode": self._mode,
            "process": self._process.status(),
            "saved_map": {"exists": self._store.exists(), "saved_at": self._store.saved_at()},
            "directory": str(self._store.directory),
            "localization": None if self._tracker is None else self._tracker.snapshot(),
        }

    def _halt(self) -> None:
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


def slam_command(params_file: Path) -> list[str]:
    return [
        "ros2", "launch", "slam_toolbox", "online_async_launch.py",
        f"slam_params_file:={params_file}", "use_sim_time:=false",
    ]
