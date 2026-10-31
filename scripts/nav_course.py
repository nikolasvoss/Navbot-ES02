#!/usr/bin/env python3
"""nav_course: drive a course of navigation goals through the hub and score each goal.

Uses only the hub's HTTP API, so it needs no ROS. The hub must be in navigation mode.
Keep course files outside the repository.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
import urllib.error
import urllib.request
from dataclasses import dataclass
from pathlib import Path
from urllib.parse import urlparse

import yaml

POLL_SEC = 0.5
HTTP_TIMEOUT_SEC = 5.0
MEMINFO = Path("/proc/meminfo")
LOCAL_HOSTS = ("127.0.0.1", "localhost")
EXIT_OK, EXIT_FAILED, EXIT_USAGE, EXIT_UNREACHABLE, EXIT_PREFLIGHT = 0, 1, 2, 3, 4


class CourseError(Exception):
    pass


class Unreachable(Exception):
    pass


@dataclass(frozen=True)
class Goal:
    name: str
    x: float
    y: float
    yaw_deg: float


@dataclass(frozen=True)
class Result:
    goal: Goal
    outcome: str
    reason: str | None
    distance_m: float | None
    heading_error_deg: float | None
    passed: bool

    def detail(self) -> str:
        return self.outcome if self.reason is None else f"{self.outcome}: {self.reason}"

    def human(self) -> str:
        verdict = "PASS" if self.passed else "FAIL"
        if self.distance_m is None:
            return f"{self.goal.name}: {verdict} {self.detail()}, no pose"
        return (f"{self.goal.name}: {verdict} {self.detail()}, "
                f"{self.distance_m:.2f} m, {self.heading_error_deg:.1f} deg")

    def as_json(self) -> dict:
        return {
            "type": "goal", "name": self.goal.name, "x": self.goal.x, "y": self.goal.y,
            "yaw_deg": self.goal.yaw_deg, "outcome": self.outcome, "reason": self.reason,
            "passed": self.passed, "distance_m": self.distance_m,
            "heading_error_deg": self.heading_error_deg,
        }


def _is_number(value) -> bool:
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def load_course(path: Path) -> tuple[int, list[Goal]]:
    try:
        data = yaml.safe_load(path.read_text())
    except OSError as exc:
        raise CourseError(f"cannot read {path}: {exc.strerror}") from exc
    except yaml.YAMLError as exc:
        raise CourseError(f"{path} is not valid YAML: {exc}") from exc
    if not isinstance(data, dict):
        raise CourseError(f"{path}: expected a mapping with goals")
    repeat = data.get("repeat", 1)
    if not isinstance(repeat, int) or isinstance(repeat, bool) or repeat < 1:
        raise CourseError(f"{path}: repeat must be a whole number from 1")
    raw_goals = data.get("goals")
    if not isinstance(raw_goals, list) or not raw_goals:
        raise CourseError(f"{path}: goals must be a non-empty list")
    return repeat, [_goal(path, raw) for raw in raw_goals]


def _goal(path: Path, raw) -> Goal:
    if not isinstance(raw, dict):
        raise CourseError(f"{path}: every goal must be a mapping")
    name = raw.get("name")
    if not isinstance(name, str) or not name:
        raise CourseError(f"{path}: every goal needs a name")
    for key in ("x", "y", "yaw_deg"):
        if not _is_number(raw.get(key)):
            raise CourseError(f"{path}: goal {name!r} needs a number for {key}")
    return Goal(name, float(raw["x"]), float(raw["y"]), float(raw["yaw_deg"]))


class Hub:
    def __init__(self, base: str) -> None:
        self.base = base.rstrip("/")
        self.host = urlparse(base).hostname or ""

    def get_map(self) -> dict:
        return self._request("GET", "/map.json", None)[1]

    def post(self, path: str, body: dict | None = None) -> tuple[int, dict]:
        return self._request("POST", path, body)

    def _request(self, method: str, path: str, body: dict | None) -> tuple[int, dict]:
        data = None if body is None else json.dumps(body).encode()
        request = urllib.request.Request(
            self.base + path, data=data, method=method, headers={"Content-Type": "application/json"})
        try:
            with urllib.request.urlopen(request, timeout=HTTP_TIMEOUT_SEC) as response:
                status, raw = response.status, response.read()
        except urllib.error.HTTPError as exc:
            status, raw = exc.code, exc.read()
        except OSError as exc:
            raise Unreachable(f"{self.base}: {exc}") from exc
        try:
            return status, json.loads(raw)
        except ValueError as exc:
            raise Unreachable(f"{self.base}{path} answered {status} without JSON") from exc


def wrap_deg(angle: float) -> float:
    return (angle + 180.0) % 360.0 - 180.0


def mem_available_mb(meminfo: Path) -> float | None:
    try:
        for line in meminfo.read_text().splitlines():
            if line.startswith("MemAvailable:"):
                return int(line.split()[1]) / 1024
    except (OSError, ValueError, IndexError):
        return None
    return None


def preflight(status: dict, host: str, min_free_mb: float) -> list[str]:
    failures = []
    control = status.get("control") or {}
    mode = control.get("mode")
    nav = control.get("nav")
    if mode != "navigation":
        failures.append(f"hub is in {mode} mode, not navigation")
    elif nav is None:
        failures.append("navigation status is missing")
    elif nav["supervisor"]["state"] != "ready":
        reason = nav["supervisor"]["reason"] or "no reason given"
        failures.append(f"supervisor is {nav['supervisor']['state']}: {reason}")
    if host in LOCAL_HOSTS:
        free_mb = mem_available_mb(MEMINFO)
        if free_mb is None:
            failures.append(f"cannot read MemAvailable from {MEMINFO}")
        elif free_mb < min_free_mb:
            failures.append(f"only {free_mb:.0f} MB free, need {min_free_mb:.0f} MB")
    return failures


def wait_for_goal(hub: Hub, goal_id: int, timeout: float) -> tuple[str, str | None]:
    deadline = time.monotonic() + timeout
    while True:
        nav = (hub.get_map().get("control") or {}).get("nav")
        if nav is not None:
            supervisor = nav["supervisor"]
            if supervisor["state"] == "stopped":
                return "stopped", supervisor["reason"]
            if nav["goal_id"] >= goal_id and nav["outcome"] is not None:
                return nav["outcome"], nav["error"] or None
        if time.monotonic() >= deadline:
            hub.post("/nav/cancel")
            return "timeout", None
        time.sleep(POLL_SEC)


def run_goal(hub: Hub, goal: Goal, args: argparse.Namespace) -> Result:
    before = (((hub.get_map().get("control") or {}).get("nav")) or {}).get("goal_id", 0)
    status, answer = hub.post("/nav/goal", {"x": goal.x, "y": goal.y, "yaw": math.radians(goal.yaw_deg)})
    if status == 200 and answer.get("ok"):
        outcome, reason = wait_for_goal(hub, before + 1, args.timeout)
    else:
        outcome, reason = "refused", answer.get("error")
    pose = hub.get_map().get("pose")
    distance = heading = None
    if pose is not None:
        distance = math.hypot(goal.x - pose["x"], goal.y - pose["y"])
        heading = abs(wrap_deg(goal.yaw_deg - math.degrees(pose["yaw"])))
    passed = (outcome == "reached" and distance is not None
              and distance <= args.max_error_m and heading <= args.max_error_deg)
    return Result(goal, outcome, reason, distance, heading, passed)


def emit(args: argparse.Namespace, human: str, obj: dict) -> None:
    print(json.dumps(obj) if args.json else human)


def parse_args(argv: list[str] | None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Drive a course of navigation goals and score each goal.")
    parser.add_argument("course", type=Path, help="course YAML file, kept outside the repository")
    parser.add_argument("--hub", default="http://127.0.0.1:8081")
    parser.add_argument("--preflight-only", action="store_true", help="check the hub and stop")
    parser.add_argument("--min-free-mb", type=float, default=300.0)
    parser.add_argument("--max-error-m", type=float, default=0.10)
    parser.add_argument("--max-error-deg", type=float, default=10.0)
    parser.add_argument("--timeout", type=float, default=120.0, help="seconds per goal")
    parser.add_argument("--json", action="store_true", help="print one JSON object per line")
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    try:
        repeat, goals = load_course(args.course)
    except CourseError as exc:
        print(f"error: {exc}", file=sys.stderr)
        return EXIT_USAGE
    hub = Hub(args.hub)
    try:
        failures = preflight(hub.get_map(), hub.host, args.min_free_mb)
        if args.json:
            print(json.dumps({"type": "preflight", "ok": not failures, "failures": failures}))
        elif failures:
            for failure in failures:
                print(f"preflight: {failure}")
        else:
            print("preflight: ok")
        if failures:
            return EXIT_PREFLIGHT
        if args.preflight_only:
            return EXIT_OK
        plan = [goal for _ in range(repeat) for goal in goals]
        passed = 0
        for goal in plan:
            result = run_goal(hub, goal, args)
            emit(args, result.human(), result.as_json())
            if not result.passed:
                break
            passed += 1
        emit(args, f"passed {passed}/{len(plan)}", {"type": "summary", "passed": passed, "total": len(plan)})
        return EXIT_OK if passed == len(plan) else EXIT_FAILED
    except Unreachable as exc:
        print(f"error: hub unreachable: {exc}", file=sys.stderr)
        return EXIT_UNREACHABLE


if __name__ == "__main__":
    sys.exit(main())
