#!/usr/bin/env python3
"""Create a compact, repeatable Markdown summary of balance trace log files.

Reads both capture formats: the K55 to K58 profile logs (TRACE, CTRL, BAL and DRIVE rows) and
the navlink CSV (`navlink capture`: `#` header comments, a `t_us,<signal names>` column header).
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


RECORD_START = re.compile(r"(?=(?:TRACE|CTRL|BAL|DRIVE),)")
TRACE_PREFIXES = {"TRACE", "CTRL", "BAL", "DRIVE"}
NAVLINK = "navlink"
NAVLINK_COLUMN_HEADER = "t_us,"
UINT32_RANGE = 2 ** 32
MAX_STEP_US = 10_000_000  # a bigger jump in t_us is not a wrap
LOST_FRAMES = re.compile(r"lost (\d+) frames before seq \d+")
MAX_LISTED_EVENTS = 5

# Keep the report useful to an agent without reproducing the whole time series.
REPORT_FIELDS = {
    "TRACE": (
        "voltage_min_raw_v", "voltage_filtered_v", "roll_deg", "pitch_deg",
        "servo1_range_deg", "servo2_range_deg", "servo3_range_deg", "servo4_range_deg",
        "motor1_target", "motor2_target",
    ),
    "CTRL": (
        "voltage_min_raw_v", "voltage_filtered_v", "roll_deg", "motor1_target", "motor2_target",
    ),
    "BAL": (
        "voltage_min_raw_v", "roll_deg", "motor1_target", "motor2_target",
    ),
    "DRIVE": (
        "voltage_min_raw_v", "voltage_filtered_v", "imu_dt_s", "imu_long_gap_count", "imu_max_interval_us",
        "control_execution_us", "max_loop_entry_interval_us",
        "motor1_target", "motor2_target", "motor1_current_sp", "motor2_current_sp",
        "motor1_voltage_q", "motor2_voltage_q", "motor1_shaft_velocity", "motor2_shaft_velocity",
        "battery_raw_instant_v", "battery_filtered_instant_v",
    ),
}


class Trace(NamedTuple):
    kind: str | None      # TRACE, CTRL, BAL, DRIVE or NAVLINK
    header: list | None
    rows: list            # (kind, {column: float})
    malformed: int
    notes: dict           # navlink header and event comments; empty for K logs


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
    if any(line.startswith(NAVLINK_COLUMN_HEADER) for line in lines):
        return read_navlink(lines, path)
    return read_k_profile(lines)


def read_k_profile(lines):
    header = None
    record_type = None
    rows = []
    malformed = 0

    for line in lines:
        if line.startswith("# "):
            candidate = line[2:].strip()
            first = candidate.split(",", 1)[0]
            if first in TRACE_PREFIXES:
                try:
                    header = next(csv.reader([candidate]))
                    record_type = first
                except csv.Error:
                    malformed += 1
            continue

        starts = [match.start() for match in RECORD_START.finditer(line)]
        if not starts:
            continue
        if starts[0] > 0:
            # Firmware startup text or a damaged prefix before a trace row.
            line = line[starts[0]:]
            starts = [match.start() for match in RECORD_START.finditer(line)]
        chunks = [line[start:end] for start, end in zip(starts, starts[1:] + [len(line)])]
        for chunk in chunks:
            try:
                fields = next(csv.reader([chunk]))
            except csv.Error:
                malformed += 1
                continue
            kind = fields[0] if fields else ""
            if kind not in TRACE_PREFIXES:
                continue
            # Logs normally carry a single header. If absent, retain records
            # for the count but report them as malformed rather than guessing units.
            if not header or header[0] != kind or len(fields) != len(header):
                malformed += 1
                continue
            parsed = parse_numbers(header[1:], fields[1:])
            if parsed is None:
                malformed += 1
            else:
                rows.append((kind, parsed))
    return Trace(record_type, header, rows, malformed, {})


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
    """Rows get `t_us` (unwrapped) and `time_ms` = `t_us` / 1000, the column K logs call `time_ms`.

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
    header = ["t_us", "time_ms", *columns[1:]] if columns else None
    return Trace(NAVLINK if columns else None, header, rows, malformed, notes)


def fmt(value):
    if value is None:
        return "—"
    return f"{value:.3f}".rstrip("0").rstrip(".")


def missing_sequence_count(data, name):
    sequence = [int(row[name]) for row in data]
    return sum(max(0, current - previous - 1) for previous, current in zip(sequence, sequence[1:]))


def imu_sample_rate(data):
    """IMU samples per second between the first and last DRIVE row, or None if not measurable."""
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
    kind, header, rows, malformed, notes = read_trace(path)
    actual_kinds = Counter(row_kind for row_kind, _ in rows)
    if not rows:
        return [f"### `{path}`", "", "Keine vollständigen Trace-Datensätze gefunden.",
                f"Beschädigte/ungültige Trace-Datensätze: **{malformed}**", ""]

    kind = rows[0][0]
    data = [row for row_kind, row in rows if row_kind == kind]
    times = [row.get("time_ms") for row in data if row.get("time_ms") is not None]
    duration = (times[-1] - times[0]) / 1000 if len(times) > 1 else None
    intervals = [(b - a) for a, b in zip(times, times[1:]) if b > a]

    lines = [f"### `{path}`", "", f"- Trace-Modus: **{kind}**"
             f"; gültige Datensätze: **{len(rows)}**"
             f"; beschädigte/ungültige Datensätze: **{malformed}**"]
    if duration is not None:
        lines.append(f"- Zeitspanne der gültigen Datensätze: **{duration:.2f} s**"
                     + (f"; Medianabstand zwischen gültigen Datensätzen: **{statistics.median(intervals):.1f} ms**"
                        if intervals else ""))
    if kind == NAVLINK:
        lines.extend(navlink_lines(notes))
    if header and "sequence" in header:
        missing = missing_sequence_count(data, "sequence")
        lines.append(f"- Sequenzlücken: **{missing}**; Gerätezeitstempel: `time_ms`, nicht PC-Empfangszeit")
    if header and "imu_sequence" in header:
        rate = imu_sample_rate(data)
        lines.append("- IMU-Abtastrate aus `imu_sequence`: **"
                     + (f"{rate:.1f} Hz" if rate is not None else "—")
                     + "**; Zeilen überspringen IMU-Samples, daher keine Lückenzählung")
    if header and "logger_dropped" in header:
        dropped = max(int(row["logger_dropped"]) for row in data)
        write_failures = max(int(row["uart_write_failures"]) for row in data)
        lines.append(f"- Senderzähler am letzten erfassten Snapshot: verworfene Queue-Datensätze **{dropped}**, UART-Schreibfehler **{write_failures}**")

    if "ch5_mode" in (header or []):
        modes = [int(row["ch5_mode"]) for row in data if "ch5_mode" in row]
        counts = Counter(modes)
        transitions = [(times[index], modes[index], modes[index + 1])
                       for index in range(min(len(times), len(modes)) - 1)
                       if modes[index] != modes[index + 1]]
        mode_text = ", ".join(f"{mode}: {count}" for mode, count in sorted(counts.items()))
        lines.append(f"- CH5-Modi (Datensätze): {mode_text or '—'}")
        if transitions:
            transition_text = ", ".join(f"{before}→{after} bei {t:g} ms" for t, before, after in transitions)
            lines.append(f"- Erkannte Wechsel: {transition_text}")

    if kind == NAVLINK:
        # Preset columns differ per capture, so report every signal.
        available = header[2:]
    else:
        available = [field for field in REPORT_FIELDS.get(kind, ()) if field in (header or [])]
    if available:
        lines.extend(["", "| Messgröße | Min | Max | Mittel | Anfang → Ende |",
                      "|---|---:|---:|---:|---:|"])
        for name in available:
            values = [row[name] for row in data if name in row]
            if not values:
                continue
            lines.append(f"| `{name}` | {fmt(min(values))} | {fmt(max(values))} "
                         f"| {fmt(statistics.fmean(values))} | {fmt(values[0])} → {fmt(values[-1])} |")
    if len(actual_kinds) > 1:
        lines.append(f"\nWeitere Datensatztypen in der Datei: {dict(actual_kinds)}.")
    lines.append("")
    return lines


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("logs", nargs="+", type=Path, help="eine oder mehrere Capture-Logdateien (K55 bis K58 oder navlink-CSV)")
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
