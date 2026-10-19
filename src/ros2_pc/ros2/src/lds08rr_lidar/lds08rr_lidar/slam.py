from __future__ import annotations

import base64
import json
import os
import signal
import subprocess
import time
from array import array
from pathlib import Path
from threading import Event, Lock, Thread
from typing import Callable

MAP_STALE_AFTER_SEC = 10.0  # slam_toolbox publishes /map every 2 s
POSE_STALE_AFTER_SEC = 10.0
DEFAULT_SLAM_DIR = Path.home() / ".local/share/navbot/slam"
BASE_PARAMS = Path(__file__).with_name("slam_toolbox.yaml")


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

    def set_grid(self, grid: dict) -> None:
        with self._lock:
            self._grid, self._grid_at = grid, self._clock()

    def set_pose(self, x: float, y: float, yaw: float, age_sec: float = 0.0) -> None:
        with self._lock:
            self._pose, self._pose_at = {"x": x, "y": y, "yaw": yaw}, self._clock() - age_sec

    def snapshot(self) -> dict:
        now = self._clock()
        with self._lock:
            grid, grid_at, pose, pose_at = self._grid, self._grid_at, self._pose, self._pose_at
        map_age = None if grid_at is None else now - grid_at
        pose_age = None if pose_at is None else now - pose_at
        pose_fresh = pose is not None and pose_age <= POSE_STALE_AFTER_SEC
        if grid is None:
            state = "waiting"
        elif map_age > MAP_STALE_AFTER_SEC:
            state = "stale"
        elif not pose_fresh:
            state = "no_pose"
        else:
            state = "mapping"
        return {
            "state": state,
            "map": grid,
            "pose": pose if pose_fresh else None,
            "map_age_sec": map_age,
            "pose_age_sec": pose_age,
        }


class MapStore:
    """The one saved map: slam_toolbox writes <base>.posegraph and <base>.data."""

    def __init__(self, directory: Path) -> None:
        self.directory = directory
        self.base = directory / "map"

    def _file(self, suffix: str) -> Path:
        return Path(f"{self.base}{suffix}")

    def exists(self) -> bool:
        return self._file(".posegraph").exists()

    def saved_at(self) -> float | None:
        try:
            return self._file(".posegraph").stat().st_mtime
        except FileNotFoundError:
            return None

    def commit(self, tmp_base: Path) -> None:
        tmp_data, tmp_graph = Path(f"{tmp_base}.data"), Path(f"{tmp_base}.posegraph")
        for path in (tmp_data, tmp_graph):
            if not path.exists():
                raise FileNotFoundError(f"serialized map missing: {path}")
        os.replace(tmp_data, self._file(".data"))
        os.replace(tmp_graph, self._file(".posegraph"))

    def delete(self) -> None:
        self._file(".posegraph").unlink(missing_ok=True)
        self._file(".data").unlink(missing_ok=True)

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
            except OSError as exc:
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


class SlamControl:
    """Save, delete and status for the single map; one lock serializes them."""

    def __init__(self, store: MapStore, process: SlamProcess, serialize: Callable[[str], str | None]) -> None:
        self._store = store
        self._process = process
        self._serialize = serialize
        self._lock = Lock()

    def start(self) -> None:
        self._process.start()

    def stop(self) -> None:
        self._process.stop()

    def save(self) -> str | None:
        with self._lock:
            tmp = self._store.directory / "map.saving"
            error = self._serialize(str(tmp))
            if error is not None:
                return error
            try:
                self._store.commit(tmp)
            except FileNotFoundError as exc:
                return str(exc)
            return None

    def delete(self) -> None:
        with self._lock:
            self._process.stop()
            self._store.delete()
            self._process.start()

    def status(self) -> dict:
        return {
            "process": self._process.status(),
            "saved_map": {"exists": self._store.exists(), "saved_at": self._store.saved_at()},
            "directory": str(self._store.directory),
        }


def slam_command(params_file: Path) -> list[str]:
    return [
        "ros2", "launch", "slam_toolbox", "online_async_launch.py",
        f"slam_params_file:={params_file}", "use_sim_time:=false",
    ]
