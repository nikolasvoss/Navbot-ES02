#!/usr/bin/env python3
"""drive_steps: drives the robot through speed and turn steps and measures how it follows them.

Steps file (YAML): {rate_hz: 20, steps: [{speed_mps: 0.1, turn_rad_s: 0.0, seconds: 3.0}]}. The robot must have
CH5 in position 1 with balance running. The run ends with 2 s at zero speed. The capture uses the `steps` preset
and goes to OUT.csv; OUT.md gets one row per step. Only wheel velocities and roll are analysed, not turning.
Docs: docs/integration/software/navlink.md
"""

import argparse
import csv
import math
import queue
import statistics
import sys
import time
from dataclasses import dataclass
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

import navlink  # noqa: E402  (scripts/navlink.py: the shared link, capture and exit-code conventions)
from navbot_link.capture import CsvWriter, describe  # noqa: E402
from navbot_link.framing import Text  # noqa: E402
from navbot_link.link import (  # noqa: E402
    CommandRefused,
    Disconnected,
    FramesLost,
    Link,
    LogEvent,
    Restarted,
    Row,
    SubscriptionEnded,
)
from navbot_link.protocol import LinkError, NoReply, RobotRestarted, UsageError  # noqa: E402

DEFAULT_WHEEL_RADIUS_M = 0.01915
TAIL_S = 2.0  # zero speed at the end of every run, so the last moving step's stop is measured
STOP_SPEED_MPS = 0.02  # a stop is reached when the forward speed is at or below this
MOVED_SPEED_MPS = 0.02  # the wheels moved when the steady speed has the command's sign and reaches this
STEADY_FROM = 0.5  # the steady state is the second half of a step
LINK_MODE, BALANCE_RUN = 1, 2  # ch5_position 1 is link mode; balance_state 2 is Run
REFUSED_TEXT = "CH5 must be in position 1 with balance running"
EXIT_DONE, EXIT_LINK_ERROR, EXIT_USAGE, EXIT_NO_CONNECTION, EXIT_REFUSED = 0, 1, 2, 3, 4
EXIT_INTERRUPTED = 130


class Refused(Exception):
    """The robot will not take the drive: a precondition failed, or it refused a Command during the run."""


@dataclass(frozen=True)
class Step:
    speed_mps: float
    turn_rad_s: float
    seconds: float


def load_steps(path: Path) -> tuple[float, list[Step]]:
    """The command rate and the steps. A bad file is a usage error."""
    try:
        data = yaml.safe_load(path.read_text(encoding="utf-8"))
    except (OSError, yaml.YAMLError) as exc:
        raise UsageError(f"cannot read {path}: {exc}") from None
    if not isinstance(data, dict) or not isinstance(data.get("steps"), list) or not data["steps"]:
        raise UsageError(f"{path}: needs a non-empty 'steps' list")
    rate = _number(data.get("rate_hz"), "rate_hz")
    if rate <= 0:
        raise UsageError(f"{path}: rate_hz must be above 0")
    steps = []
    for index, item in enumerate(data["steps"]):
        if not isinstance(item, dict):
            raise UsageError(f"{path}: step {index} is not a mapping")
        step = Step(_number(item.get("speed_mps"), "speed_mps"),
                    _number(item.get("turn_rad_s", 0.0), "turn_rad_s"),
                    _number(item.get("seconds"), "seconds"))
        if step.seconds <= 0:
            raise UsageError(f"{path}: step {index} needs seconds above 0")
        steps.append(step)
    return rate, steps


def _number(value, name: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise UsageError(f"{name} must be a finite number, not {value!r}")
    return float(value)


def read_capture(path: Path) -> list[dict]:
    """The rows of a navlink CSV as numbers. Comment lines are skipped."""
    with open(path, encoding="utf-8", newline="") as f:
        lines = [line for line in f if not line.startswith("#")]
    rows = []
    for row in csv.DictReader(lines):
        values = {name: float(text) for name, text in row.items()}
        values["t_us"] = int(row["t_us"])
        rows.append(values)
    return rows


def trapezoid(points: list[tuple[float, float]]) -> float:
    return sum((t1 - t0) * (v0 + v1) / 2 for (t0, v0), (t1, v1) in zip(points, points[1:]))


def _reached(speed: float, target: float) -> bool:
    return speed >= 0.9 * target if target > 0 else speed <= 0.9 * target


def analyze_steps(rows: list[dict], steps: list[Step], wheel_radius_m: float) -> list[dict]:
    """One result per step. Forward speed is the mean wheel speed times the wheel radius, positive forward.

    t90_s: time from the step start until the speed reaches 90 % of the command.
    steady_error_mps: the mean speed over the second half of the step, minus the command.
    peak_roll_deg: the largest absolute roll in the step.
    stop_distance_m: distance covered from the end of a moving step until the speed first is at or below
        STOP_SPEED_MPS, within the window that ends at the next moving step or at the end of the capture.
    rollback_m: distance travelled backwards after that stop, in the same window.
    moved: the steady speed has the command's sign and reaches MOVED_SPEED_MPS.
    """
    first = next((row for row in rows
                  if math.isclose(row["cmd_speed_mps"], steps[0].speed_mps, abs_tol=1e-6)
                  and math.isclose(row["cmd_turn_rad_s"], steps[0].turn_rad_s, abs_tol=1e-6)), None)
    if first is None:
        raise ValueError("the robot never took the first step's command")
    t0 = first["t_us"]
    series = []
    for row in rows:
        t = ((row["t_us"] - t0 + 2 ** 31) % 2 ** 32 - 2 ** 31) / 1e6  # robot clock, wrap-safe, in seconds
        series.append((t, wheel_radius_m * (row["motor1_velocity_f"] + row["motor2_velocity_f"]) / 2, row))
    starts = [0.0]
    for step in steps:
        starts.append(starts[-1] + step.seconds)

    results = []
    for index, step in enumerate(steps):
        start, end = starts[index], starts[index + 1]
        window = [(t, v, row) for t, v, row in series if start <= t < end]
        steady = [v for t, v, _ in window if t >= start + step.seconds * STEADY_FROM]
        mean = statistics.fmean(steady) if steady else 0.0
        t90 = next((round(t - start, 3) for t, v, _ in window if step.speed_mps and _reached(v, step.speed_mps)),
                   None)
        result = {
            "speed_mps": step.speed_mps,
            "turn_rad_s": step.turn_rad_s,
            "seconds": step.seconds,
            "t90_s": t90,
            "steady_error_mps": round(mean - step.speed_mps, 3),
            "peak_roll_deg": round(max((abs(row["roll_deg"]) for _, _, row in window), default=0.0), 3),
            "stop_distance_m": None,
            "rollback_m": None,
            "moved": bool(step.speed_mps) and abs(mean) >= MOVED_SPEED_MPS and mean * step.speed_mps > 0,
        }
        if step.speed_mps:
            bound = next((starts[j] for j in range(index + 1, len(steps)) if steps[j].speed_mps), math.inf)
            phase = [(t, v) for t, v, _ in series if end <= t < bound]
            stop = next((k for k, (_, v) in enumerate(phase) if abs(v) <= STOP_SPEED_MPS), None)
            if stop is None:
                result["stop_distance_m"] = round(trapezoid(phase), 3)
            else:
                result["stop_distance_m"] = round(trapezoid(phase[:stop + 1]), 3)
                result["rollback_m"] = round(-trapezoid([(t, min(v, 0.0)) for t, v in phase[stop:]]), 3)
        results.append(result)
    return results


def smallest_moving_speed(results: list[dict]) -> float | None:
    """The smallest step speed, in m/s, at which the wheels moved."""
    return min((abs(r["speed_mps"]) for r in results if r["moved"]), default=None)


def write_report(path: Path, results: list[dict], wheel_radius_m: float) -> None:
    lines = [
        "| Speed m/s | Turn rad/s | Seconds | t90 s | Steady error m/s | Peak roll deg | Stop distance m "
        "| Rollback m | Moved |",
        "| --- | --- | --- | --- | --- | --- | --- | --- | --- |",
    ]
    for r in results:
        cells = [r["speed_mps"], r["turn_rad_s"], r["seconds"], r["t90_s"], r["steady_error_mps"],
                 r["peak_roll_deg"], r["stop_distance_m"], r["rollback_m"], "yes" if r["moved"] else "no"]
        lines.append("| " + " | ".join("-" if c is None else f"{c:g}" if isinstance(c, float) else str(c)
                                       for c in cells) + " |")
    smallest = smallest_moving_speed(results)
    lines += ["", f"Wheel radius {wheel_radius_m:g} m. Smallest speed that moved the wheels: "
                  + ("none" if smallest is None else f"{smallest:g} m/s") + "."]
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def check_preconditions(link: Link) -> None:
    if (link.get("ch5_position"), link.get("balance_state")) != (LINK_MODE, BALANCE_RUN):
        raise Refused(REFUSED_TEXT)


def _handle(event, sub, writer, err) -> None:
    if isinstance(event, Row):
        if event.slot == sub.slot:
            writer.row(event)
    elif isinstance(event, (LogEvent, FramesLost)):
        writer.event(event)
        print(describe(event), file=err)
    elif isinstance(event, Text):
        print(event.text.rstrip("\r\n"), file=err)
    elif isinstance(event, CommandRefused):
        raise Refused(f"command refused: {event.reason}")
    elif isinstance(event, Restarted):
        raise RobotRestarted("robot restarted")
    elif isinstance(event, SubscriptionEnded) and event.slot == sub.slot:
        writer.event(event)
        raise LinkError(f"capture stopped: {event.reason}")
    elif isinstance(event, Disconnected):
        raise LinkError(f"port closed: {event.reason}")


def _run_step(link: Link, sub, writer, step: Step, rate_hz: float, err) -> None:
    """Sends the step's Command at rate_hz for its duration and writes the rows that arrive meanwhile."""
    period = 1.0 / rate_hz
    end = time.monotonic() + step.seconds
    next_send = time.monotonic()
    while (now := time.monotonic()) < end:
        if now >= next_send:
            link.command(step.speed_mps, step.turn_rad_s)
            next_send = now + period
        wait = max(0.0, min(end, next_send) - time.monotonic())
        try:
            event = link.events.get(timeout=wait)
        except queue.Empty:
            continue
        _handle(event, sub, writer, err)


def drive(link: Link, steps: list[Step], rate_hz: float, output, presets: Path, err) -> None:
    """Sends the steps and the zero tail, and writes the `steps` preset to output while they run."""
    names, preset_rate = navlink.resolve_signals(["@steps"], presets)
    catalog = link.catalog()
    params = navlink.read_params(link)
    sub = link.subscribe(names, preset_rate)
    try:
        writer = CsvWriter(output, sub.entries, catalog.build, preset_rate, params)
        for step in [*steps, Step(0.0, 0.0, TAIL_S)]:
            _run_step(link, sub, writer, step, rate_hz, err)
    finally:
        try:
            link.command(0.0, 0.0)  # zero speed even when the run stopped part way
        except LinkError:
            pass
        try:
            link.unsubscribe(sub.slot)
        except LinkError:
            pass  # the subscription expires with its lease


def parse_args(argv):
    parser = argparse.ArgumentParser(prog="drive_steps", description=__doc__.splitlines()[0])
    parser.add_argument("steps", type=Path, help="steps YAML file")
    parser.add_argument("-o", "--output", required=True, type=Path,
                        help="capture CSV; the report goes next to it as .md")
    parser.add_argument("--port", default=navlink.DEFAULT_PORT,
                        help=f"the link multiplexer (default {navlink.DEFAULT_PORT}), or a serial port")
    parser.add_argument("--wheel-radius", type=float, default=DEFAULT_WHEEL_RADIUS_M, metavar="M",
                        help=f"wheel radius in metres (default {DEFAULT_WHEEL_RADIUS_M})")
    return parser.parse_args(argv)


def _fail(code: int, exc) -> int:
    print(f"drive_steps: error: {exc}", file=sys.stderr)
    return code


def main(argv=None, *, open_link=None) -> int:
    args = parse_args(argv)
    open_link = open_link or (lambda port, baud: Link(port, baud))
    link = None
    try:
        rate, steps = load_steps(args.steps)
        output = navlink._open_output(str(args.output))
        with output:
            link = navlink.open_ready(open_link, args.port, navlink.default_baud(args.port), 0)
            check_preconditions(link)
            drive(link, steps, rate, output, navlink.DEFAULT_PRESETS, sys.stderr)
    except UsageError as exc:
        return _fail(EXIT_USAGE, exc)
    except Refused as exc:
        return _fail(EXIT_REFUSED, exc)
    except (NoReply, OSError) as exc:
        return _fail(EXIT_NO_CONNECTION, exc)
    except LinkError as exc:
        dead = link is not None and link.dead_reason is not None
        return _fail(EXIT_NO_CONNECTION if dead else EXIT_LINK_ERROR, exc)
    except KeyboardInterrupt:
        print("drive_steps: interrupted", file=sys.stderr)
        return EXIT_INTERRUPTED
    finally:
        navlink._close_quietly(link)

    try:
        results = analyze_steps(read_capture(args.output), steps, args.wheel_radius)
    except ValueError as exc:
        return _fail(EXIT_LINK_ERROR, exc)
    report = args.output.with_suffix(".md")
    write_report(report, results, args.wheel_radius)
    print(f"{len(results)} steps -> {args.output}, {report}")
    return EXIT_DONE


if __name__ == "__main__":
    sys.exit(main())
