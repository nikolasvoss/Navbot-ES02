from __future__ import annotations

import importlib.util
import json
import math
import re
import socket
import subprocess
import sys
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[2]
SCRIPT = ROOT / "scripts/nav_course.py"
SPEC = importlib.util.spec_from_file_location("nav_course", SCRIPT)
NAV_COURSE = importlib.util.module_from_spec(SPEC)
sys.modules["nav_course"] = NAV_COURSE  # the dataclasses in the script look their module up here
SPEC.loader.exec_module(NAV_COURSE)

DESK = "goals:\n  - name: desk\n    x: 1.2\n    y: -0.4\n    yaw_deg: 90\n"
TWO_GOALS = DESK + "  - name: shelf\n    x: 0.5\n    y: 0.5\n    yaw_deg: 0\n"


class FakeHub:
    """A hub on 127.0.0.1 that answers /map.json and /nav/* the way the real hub does."""

    def __init__(self) -> None:
        self.mode = "navigation"
        self.supervisor_state = "ready"
        self.supervisor_reason = None
        self.goal_answer = (200, {"ok": True})
        self.outcome = "reached"
        self.stuck = False  # never sets an outcome
        self.stop_on_goal = False  # supervisor stops as the goal starts
        self.pose_offset = (0.0, 0.0, 0.0)  # robot error: dx, dy in m, dyaw in deg
        self.goal_id = 0
        self.goal = None
        self.posts: list[tuple[str, object]] = []
        self.url = ""

    def status(self) -> dict:
        nav = None
        if self.mode == "navigation":
            outcome = None if self.stuck or self.goal is None else self.outcome
            nav = {
                "supervisor": {"state": self.supervisor_state, "reason": self.supervisor_reason, "checks": []},
                "goal_id": self.goal_id, "goal": self.goal, "distance_remaining_m": None,
                "outcome": outcome, "error": "", "plan": [],
                "process": {"state": "running", "pid": 4242, "exit_code": None, "restarts": 0, "error": None},
            }
        pose = None
        if self.goal is not None:
            dx, dy, dyaw = self.pose_offset
            pose = {"x": self.goal["x"] + dx, "y": self.goal["y"] + dy,
                    "yaw": self.goal["yaw"] + math.radians(dyaw)}
        return {"control": {"mode": self.mode, "nav": nav}, "pose": pose}

    def start(self) -> ThreadingHTTPServer:
        hub = self

        class Handler(BaseHTTPRequestHandler):
            def log_message(self, *args) -> None:
                pass

            def _send(self, code: int, body: dict) -> None:
                raw = json.dumps(body).encode()
                self.send_response(code)
                self.send_header("Content-Type", "application/json")
                self.send_header("Content-Length", str(len(raw)))
                self.end_headers()
                self.wfile.write(raw)

            def do_GET(self) -> None:
                if self.path != "/map.json":
                    self._send(404, {"ok": False})
                    return
                self._send(200, hub.status())

            def do_POST(self) -> None:
                length = int(self.headers.get("Content-Length", 0))
                body = json.loads(self.rfile.read(length)) if length else None
                hub.posts.append((self.path, body))
                if self.path == "/nav/cancel":
                    self._send(200, {"ok": True})
                    return
                code, answer = hub.goal_answer
                if code == 200:
                    hub.goal_id += 1
                    hub.goal = body
                    if hub.stop_on_goal:
                        hub.supervisor_state, hub.supervisor_reason = "stopped", "no LiDAR scan for 1 s"
                self._send(code, answer)

        server = ThreadingHTTPServer(("127.0.0.1", 0), Handler)
        threading.Thread(target=server.serve_forever, daemon=True).start()
        self.url = f"http://127.0.0.1:{server.server_port}"
        return server


@pytest.fixture
def hub():
    fake = FakeHub()
    server = fake.start()
    yield fake
    server.shutdown()
    server.server_close()


def write_course(tmp_path: Path, text: str) -> Path:
    path = tmp_path / "course.yaml"
    path.write_text(text)
    return path


def run_script(*args: str) -> subprocess.CompletedProcess:
    return subprocess.run([sys.executable, str(SCRIPT), *args], text=True, capture_output=True, check=False)


def run_course(hub: FakeHub, course: Path, *extra: str) -> subprocess.CompletedProcess:
    return run_script(str(course), "--hub", hub.url, "--min-free-mb", "0", *extra)


def test_wrap_deg_maps_into_plus_minus_180():
    assert NAV_COURSE.wrap_deg(358.0) == -2.0
    assert NAV_COURSE.wrap_deg(-190.0) == 170.0
    assert NAV_COURSE.wrap_deg(12.5) == 12.5


def test_load_course_reads_repeat_and_goals(tmp_path):
    repeat, goals = NAV_COURSE.load_course(write_course(tmp_path, "repeat: 3\n" + TWO_GOALS))
    assert repeat == 3
    assert [goal.name for goal in goals] == ["desk", "shelf"]
    assert goals[0] == NAV_COURSE.Goal("desk", 1.2, -0.4, 90.0)


def test_mem_available_reads_kilobytes_as_megabytes(tmp_path):
    meminfo = tmp_path / "meminfo"
    meminfo.write_text("MemTotal: 4096 kB\nMemAvailable: 2048 kB\n")
    assert NAV_COURSE.mem_available_mb(meminfo) == 2.0


def test_happy_path_prints_literal_lines_and_exits_0(hub, tmp_path):
    hub.pose_offset = (0.03, 0.0, 2.1)
    result = run_course(hub, write_course(tmp_path, DESK))
    assert result.returncode == 0
    assert result.stdout == "preflight: ok\ndesk: PASS reached, 0.03 m, 2.1 deg\npassed 1/1\n"


def test_json_output_has_one_object_per_line(hub, tmp_path):
    result = run_course(hub, write_course(tmp_path, DESK), "--json")
    assert result.returncode == 0
    objects = [json.loads(line) for line in result.stdout.splitlines()]
    assert objects[0] == {"type": "preflight", "ok": True, "failures": []}
    assert objects[1]["type"] == "goal"
    assert objects[1]["name"] == "desk"
    assert objects[1]["outcome"] == "reached"
    assert objects[1]["passed"] is True
    assert objects[2] == {"type": "summary", "passed": 1, "total": 1}


def test_repeat_runs_every_goal_again(hub, tmp_path):
    result = run_course(hub, write_course(tmp_path, "repeat: 2\n" + DESK))
    assert result.returncode == 0
    assert result.stdout == (
        "preflight: ok\n"
        "desk: PASS reached, 0.00 m, 0.0 deg\n"
        "desk: PASS reached, 0.00 m, 0.0 deg\n"
        "passed 2/2\n"
    )
    assert [path for path, _ in hub.posts] == ["/nav/goal", "/nav/goal"]


def test_heading_wraps_across_180_degrees(hub, tmp_path):
    course = write_course(tmp_path, "goals:\n  - name: desk\n    x: 1.2\n    y: -0.4\n    yaw_deg: 179\n")
    hub.pose_offset = (0.0, 0.0, -358.0)
    result = run_course(hub, course)
    assert result.returncode == 0
    assert result.stdout.splitlines()[1] == "desk: PASS reached, 0.00 m, 2.0 deg"


def test_distance_over_limit_fails_with_exit_1(hub, tmp_path):
    hub.pose_offset = (0.2, 0.0, 0.0)
    result = run_course(hub, write_course(tmp_path, DESK))
    assert result.returncode == 1
    assert result.stdout == "preflight: ok\ndesk: FAIL reached, 0.20 m, 0.0 deg\npassed 0/1\n"


def test_cancelled_goal_stops_the_course_with_exit_1(hub, tmp_path):
    hub.outcome = "canceled"
    result = run_course(hub, write_course(tmp_path, TWO_GOALS))
    assert result.returncode == 1
    assert result.stdout == "preflight: ok\ndesk: FAIL canceled, 0.00 m, 0.0 deg\npassed 0/2\n"
    assert len(hub.posts) == 1


def test_supervisor_stop_fails_the_goal_with_exit_1(hub, tmp_path):
    hub.stuck = True
    hub.stop_on_goal = True
    result = run_course(hub, write_course(tmp_path, DESK))
    assert result.returncode == 1
    assert result.stdout == (
        "preflight: ok\n"
        "desk: FAIL stopped: no LiDAR scan for 1 s, 0.00 m, 0.0 deg\n"
        "passed 0/1\n"
    )


def test_timeout_cancels_the_goal_and_exits_1(hub, tmp_path):
    hub.stuck = True
    result = run_course(hub, write_course(tmp_path, DESK), "--timeout", "0.2")
    assert result.returncode == 1
    assert result.stdout == "preflight: ok\ndesk: FAIL timeout, 0.00 m, 0.0 deg\npassed 0/1\n"
    assert [path for path, _ in hub.posts] == ["/nav/goal", "/nav/cancel"]


def test_refused_goal_exits_1_with_the_hub_error(hub, tmp_path):
    hub.goal_answer = (409, {"ok": False, "error": "Goals need navigation mode"})
    result = run_course(hub, write_course(tmp_path, DESK))
    assert result.returncode == 1
    assert result.stdout == (
        "preflight: ok\ndesk: FAIL refused: Goals need navigation mode, no pose\npassed 0/1\n"
    )


def test_preflight_fails_outside_navigation_mode_with_exit_4(hub, tmp_path):
    hub.mode = "mapping"
    result = run_course(hub, write_course(tmp_path, DESK))
    assert result.returncode == 4
    assert result.stdout == "preflight: hub is in mapping mode, not navigation\n"
    assert hub.posts == []


def test_preflight_fails_when_the_supervisor_is_not_ready_with_exit_4(hub, tmp_path):
    hub.supervisor_state = "not_ready"
    hub.supervisor_reason = "no LiDAR scan for 1 s"
    result = run_course(hub, write_course(tmp_path, DESK))
    assert result.returncode == 4
    assert result.stdout == "preflight: supervisor is not_ready: no LiDAR scan for 1 s\n"
    assert hub.posts == []


def test_preflight_fails_on_low_memory_with_exit_4(hub, tmp_path):
    result = run_script(str(write_course(tmp_path, DESK)), "--hub", hub.url, "--min-free-mb", "1000000000")
    assert result.returncode == 4
    assert re.fullmatch(r"preflight: only \d+ MB free, need 1000000000 MB\n", result.stdout)


def test_preflight_only_checks_and_sends_no_goal(hub, tmp_path):
    result = run_course(hub, write_course(tmp_path, DESK), "--preflight-only")
    assert result.returncode == 0
    assert result.stdout == "preflight: ok\n"
    assert hub.posts == []


def test_unreachable_hub_exits_3(tmp_path):
    probe = socket.socket()
    probe.bind(("127.0.0.1", 0))
    port = probe.getsockname()[1]
    probe.close()
    result = run_script(str(write_course(tmp_path, DESK)), "--hub", f"http://127.0.0.1:{port}", "--min-free-mb", "0")
    assert result.returncode == 3
    assert "error: hub unreachable" in result.stderr


@pytest.mark.parametrize(
    ("text", "message"),
    [
        ("goals: []\n", "goals must be a non-empty list"),
        ("goals:\n  - name: desk\n    x: 1\n    y: 2\n", "needs a number for yaw_deg"),
        ("goals: [\n", "is not valid YAML"),
        ("repeat: 0\n" + DESK, "repeat must be a whole number from 1"),
    ],
)
def test_bad_course_file_exits_2(tmp_path, text, message):
    result = run_script(str(write_course(tmp_path, text)), "--min-free-mb", "0")
    assert result.returncode == 2
    assert message in result.stderr


def test_missing_course_file_exits_2(tmp_path):
    result = run_script(str(tmp_path / "absent.yaml"), "--min-free-mb", "0")
    assert result.returncode == 2
    assert "cannot read" in result.stderr


def test_missing_course_argument_exits_2():
    result = run_script()
    assert result.returncode == 2
    assert "course" in result.stderr
