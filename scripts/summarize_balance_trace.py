#!/usr/bin/env python3
"""Create a compact, repeatable Markdown summary of balance trace log files."""

import argparse
import csv
import math
import re
import statistics
import sys
from collections import Counter
from pathlib import Path


RECORD_START = re.compile(r"(?=(?:TRACE|CTRL|BAL|DRIVE),)")
TRACE_PREFIXES = {"TRACE", "CTRL", "BAL", "DRIVE"}

# Keep the report useful to an agent without reproducing the whole time series.
REPORT_FIELDS = {
    "TRACE": (
        "voltage_min_raw_v", "voltage_filtered_v", "roll_deg", "pitch_deg",
        "servo1_range_deg", "servo2_range_deg", "servo3_range_deg", "servo4_range_deg",
        "motor1_target", "motor2_target",
    ),
    "CTRL": (
        "voltage_min_raw_v", "voltage_filtered_v", "roll_deg", "angle_error_deg",
        "angle_output", "yaw_error", "yaw_output", "motor1_target", "motor2_target",
    ),
    "BAL": (
        "voltage_min_raw_v", "roll_deg", "angle_error_deg", "angle_out_p",
        "angle_out_i", "angle_out_d", "body_x", "motor1_target", "motor2_target",
    ),
    "DRIVE": (
        "voltage_min_raw_v", "voltage_filtered_v", "speed_error", "speed_output",
        "body_x", "body_pitching_f", "angle_output", "angle_out_p", "angle_out_i",
        "angle_out_d", "motor1_target", "motor2_target",
    ),
}


def read_trace(path):
    header = None
    record_type = None
    rows = []
    malformed = 0

    # newline="" preserves physical line boundaries while csv handles fields.
    with path.open("r", encoding="utf-8", errors="replace", newline="") as source:
        for physical_line in source:
            line = physical_line.replace("\x00", "").strip()
            if not line:
                continue
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
                parsed = {}
                valid = True
                for name, value in zip(header[1:], fields[1:]):
                    try:
                        number = float(value)
                    except ValueError:
                        valid = False
                        break
                    if not math.isfinite(number):
                        valid = False
                        break
                    parsed[name] = number
                if valid:
                    rows.append((kind, parsed))
                else:
                    malformed += 1
    return record_type, header, rows, malformed


def fmt(value):
    if value is None:
        return "—"
    return f"{value:.3f}".rstrip("0").rstrip(".")


def summarize(path):
    kind, header, rows, malformed = read_trace(path)
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
    if header and "sequence" in header:
        sequence = [int(row["sequence"]) for row in data]
        missing = sum(max(0, current - previous - 1) for previous, current in zip(sequence, sequence[1:]))
        lines.append(f"- Sequenzlücken: **{missing}**; Gerätezeitstempel: `time_ms`, nicht PC-Empfangszeit")
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
    parser.add_argument("logs", nargs="+", type=Path, help="eine oder mehrere Capture-Logdateien")
    parser.add_argument("--output", "-o", type=Path,
                        help="Markdown-Ziel; ohne Option Ausgabe auf stdout")
    args = parser.parse_args()

    sections = ["# Balance-Trace: Run-Zusammenfassung", ""]
    for path in args.logs:
        if not path.is_file():
            parser.error(f"Logdatei nicht gefunden: {path}")
        sections.extend(summarize(path))
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
