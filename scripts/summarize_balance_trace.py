#!/usr/bin/env python3
"""Create a compact, repeatable Markdown summary of navlink captures.

Reads the CSV that `navlink capture` writes: `#` header comments and a `t_us,<signal names>` column header.
"""

import argparse
import csv
import math
import re
import statistics
import sys
from collections import Counter
from pathlib import Path
from typing import NamedTuple


NAVLINK = "navlink"
NAVLINK_COLUMN_HEADER = "t_us,"
UINT32_RANGE = 2 ** 32
MAX_STEP_US = 10_000_000  # a bigger jump in t_us is not a wrap
LOST_FRAMES = re.compile(r"lost (\d+) frames before seq \d+")
MAX_LISTED_EVENTS = 5


class Trace(NamedTuple):
    kind: str             # always NAVLINK
    header: list          # t_us, time_ms, then the signal names
    rows: list            # (NAVLINK, {column: float})
    malformed: int
    notes: dict           # header comments and log events


def parse_numbers(names, values):
    """Map names to finite floats, or None when a value is not one."""
    parsed = {}
    for name, value in zip(names, values):
        try:
            number = float(value)
        except ValueError:
            return None
        if not math.isfinite(number):
            return None
        parsed[name] = number
    return parsed


def read_trace(path):
    # newline="" preserves physical line boundaries while csv handles fields.
    with path.open("r", encoding="utf-8", errors="replace", newline="") as source:
        lines = [line for line in (physical.replace("\x00", "").strip() for physical in source) if line]
    return read_navlink(lines, path)


def classify_t_us(raws, index, previous_raw):
    """Say what raw `t_us` value `raws[index]` is after `previous_raw`, the last accepted value.

    "step": continuous (a lower value than before means the clock wrapped at 2^32).
    "corrupt": an isolated bad value, because the next row continues from `previous_raw`.
    "restart": the robot rebooted, so a small value follows a jump.
    "jump": a large forward gap.
    """
    raw = raws[index]
    if (raw - previous_raw) % UINT32_RANGE < MAX_STEP_US:
        return "step"
    following = raws[index + 1] if index + 1 < len(raws) else None
    if following is not None and (following - previous_raw) % UINT32_RANGE < MAX_STEP_US:
        return "corrupt"
    if raw < previous_raw:
        return "restart" if raw < MAX_STEP_US else "corrupt"
    return "jump"


def read_navlink(lines, path):
    """Rows get `t_us` (unwrapped) and `time_ms` = `t_us` / 1000.

    `t_us` is the robot's `micros()`. A backward step that continues within MAX_STEP_US across 2^32
    is a wrap. A small value after a jump back is a robot restart: the timeline continues from the
    last time before it, so the span leaves out the unknown downtime. A single out-of-range value
    is a malformed row.
    """
    columns = None
    raw_rows = []
    malformed = 0
    notes = {"build": None, "rate": None, "params": [], "events": [], "lost_frames": 0, "warnings": []}

    for line in lines:
        if line.startswith("#"):
            comment = line[1:].strip()
            if comment.startswith("build:"):
                notes["build"] = comment[len("build:"):].strip()
            elif comment.startswith("rate:"):
                notes["rate"] = comment[len("rate:"):].strip()
            elif comment.startswith("param "):
                notes["params"].append(comment[len("param "):].strip())
            elif comment.startswith(("log ", "slot ")):
                notes["events"].append(comment)
            elif lost := LOST_FRAMES.fullmatch(comment):
                notes["lost_frames"] += int(lost[1])
            continue
        if columns is None:
            if line.startswith(NAVLINK_COLUMN_HEADER):
                columns = line.split(",")
                if "time_ms" in columns:
                    raise ValueError(f"{path}: Spalte `time_ms` ist in navlink-Captures nicht erlaubt "
                                     "(sie wird aus `t_us` berechnet)")
            continue
        if line.startswith(NAVLINK_COLUMN_HEADER):
            notes["warnings"].append("Zweiter Spaltenkopf `t_us,` in der Datei; spätere Zeilen wurden nicht ausgewertet")
            break
        try:
            fields = next(csv.reader([line]))
        except csv.Error:
            malformed += 1
            continue
        parsed = parse_numbers(columns, fields) if len(fields) == len(columns) else None
        if parsed is None or not 0 <= parsed["t_us"] < UINT32_RANGE:
            malformed += 1
            continue
        raw_rows.append(parsed)
    if columns is None:
        raise ValueError(f"{path}: kein navlink-Capture (der Spaltenkopf `t_us,` fehlt)")

    rows = []
    raws = [parsed["t_us"] for parsed in raw_rows]
    previous_raw = None
    offset = 0
    restarts = 0
    for index, parsed in enumerate(raw_rows):
        raw = parsed["t_us"]
        if previous_raw is not None:
            verdict = classify_t_us(raws, index, previous_raw)
            if verdict == "corrupt":
                malformed += 1
                continue
            if verdict == "step" and raw < previous_raw:
                offset += UINT32_RANGE
            elif verdict == "restart":
                restarts += 1
                offset = rows[-1][1]["t_us"]
        previous_raw = raw
        parsed["t_us"] = raw + offset
        parsed["time_ms"] = parsed["t_us"] / 1000
        rows.append((NAVLINK, parsed))
    if restarts:
        notes["warnings"].append(f"`t_us` fiel {restarts}-mal auf einen kleinen Wert zurück (Roboter-Neustart); "
                                 "die Zeitachse läuft danach weiter, die Ausfallzeit fehlt in der Zeitspanne")
    header = ["t_us", "time_ms", *columns[1:]]
    return Trace(NAVLINK, header, rows, malformed, notes)


def fmt(value):
    if value is None:
        return "—"
    return f"{value:.3f}".rstrip("0").rstrip(".")


def imu_sample_rate(data):
    """IMU samples per second between the first and last row, or None if not measurable."""
    elapsed_s = (data[-1]["time_ms"] - data[0]["time_ms"]) / 1000
    advanced = int(data[-1]["imu_sequence"]) - int(data[0]["imu_sequence"])
    return advanced / elapsed_s if elapsed_s > 0 and advanced > 0 else None


def navlink_lines(notes):
    lines = ["- Zeitbasis: `t_us` (Roboter-`micros()`); Zeilen sind nicht gleichmäßig verteilt"
             f"; Firmware: **{notes['build'] or '—'}**; Rate: **{notes['rate'] or '—'}**"]
    if notes["params"]:
        lines.append("- Parameter bei Start: " + ", ".join(f"`{param}`" for param in notes["params"]))
    lines.append(f"- Verlorene Frames (`# lost`): **{notes['lost_frames']}**")
    lines.extend(f"- Warnung: {warning}" for warning in notes["warnings"])
    events = notes["events"]
    if events:
        lines.append(f"- Log-Ereignisse: **{len(events)}**")
        lines.extend(f"  - `{event}`" for event in events[:MAX_LISTED_EVENTS])
        if len(events) > MAX_LISTED_EVENTS:
            lines.append(f"  - … {len(events) - MAX_LISTED_EVENTS} weitere")
    return lines


def summarize(path):
    _, header, rows, malformed, notes = read_trace(path)
    if not rows:
        return [f"### `{path}`", "", "Keine vollständigen Trace-Datensätze gefunden.",
                f"Beschädigte/ungültige Trace-Datensätze: **{malformed}**", ""]

    data = [row for _, row in rows]
    times = [row["time_ms"] for row in data]
    duration = (times[-1] - times[0]) / 1000 if len(times) > 1 else None
    intervals = [(b - a) for a, b in zip(times, times[1:]) if b > a]

    lines = [f"### `{path}`", "", f"- Trace-Modus: **{NAVLINK}**"
             f"; gültige Datensätze: **{len(rows)}**"
             f"; beschädigte/ungültige Datensätze: **{malformed}**"]
    if duration is not None:
        lines.append(f"- Zeitspanne der gültigen Datensätze: **{duration:.2f} s**"
                     + (f"; Medianabstand zwischen gültigen Datensätzen: **{statistics.median(intervals):.1f} ms**"
                        if intervals else ""))
    lines.extend(navlink_lines(notes))
    if "imu_sequence" in header:
        rate = imu_sample_rate(data)
        lines.append("- IMU-Abtastrate aus `imu_sequence`: **"
                     + (f"{rate:.1f} Hz" if rate is not None else "—")
                     + "**; Zeilen überspringen IMU-Samples, daher keine Lückenzählung")

    if "ch5_position" in header:
        modes = [int(row["ch5_position"]) for row in data]
        counts = Counter(modes)
        transitions = [(times[index], modes[index], modes[index + 1])
                       for index in range(len(modes) - 1)
                       if modes[index] != modes[index + 1]]
        mode_text = ", ".join(f"{mode}: {count}" for mode, count in sorted(counts.items()))
        lines.append(f"- CH5-Modi (Datensätze): {mode_text or '—'}")
        if transitions:
            transition_text = ", ".join(f"{before}→{after} bei {t:g} ms" for t, before, after in transitions)
            lines.append(f"- Erkannte Wechsel: {transition_text}")

    # Preset columns differ per capture, so report every signal.
    available = header[2:]
    if available:
        lines.extend(["", "| Messgröße | Min | Max | Mittel | Anfang → Ende |",
                      "|---|---:|---:|---:|---:|"])
        for name in available:
            values = [row[name] for row in data if name in row]
            if not values:
                continue
            lines.append(f"| `{name}` | {fmt(min(values))} | {fmt(max(values))} "
                         f"| {fmt(statistics.fmean(values))} | {fmt(values[0])} → {fmt(values[-1])} |")
    lines.append("")
    return lines


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("logs", nargs="+", type=Path, help="eine oder mehrere navlink-CSV-Dateien")
    parser.add_argument("--output", "-o", type=Path,
                        help="Markdown-Ziel; ohne Option Ausgabe auf stdout")
    args = parser.parse_args()

    sections = ["# Balance-Trace: Run-Zusammenfassung", ""]
    for path in args.logs:
        if not path.is_file():
            parser.error(f"Logdatei nicht gefunden: {path}")
        try:
            sections.extend(summarize(path))
        except ValueError as error:
            print(f"summarize_balance_trace: {error}", file=sys.stderr)
            return 1
    report = "\n".join(sections)
    if args.output:
        try:
            args.output.write_text(report, encoding="utf-8")
        except OSError as error:
            print(f"summarize_balance_trace: {error}", file=sys.stderr)
            return 1
        print(f"Gespeichert: {args.output.resolve()}")
    else:
        print(report, end="")
    return 0


if __name__ == "__main__":
    sys.exit(main())
