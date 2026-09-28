#!/usr/bin/env python3
"""Check DRIVE captures and compare repeatable drive/stop measurements.

This reads saved logs only. It never opens a serial port or changes the robot.
"""

import argparse
import json
import statistics
import sys
from pathlib import Path

from summarize_balance_trace import read_trace


REQUIRED = {
    "time_ms", "ch5_mode", "ch3_target", "motor1_velocity_f",
    "motor2_velocity_f", "roll_ok", "motor1_target", "motor2_target",
    "voltage_min_raw_v",
}
MIN_ROWS = 20
MAX_BAD_FRACTION = 0.02
NEUTRAL_THRESHOLD = 1.0
STOP_WINDOW_MS = 1000


def rounded(value, places=3):
    return None if value is None else round(value, places)


def direction(row):
    if row["ch5_mode"] < 1 or abs(row["ch3_target"]) < NEUTRAL_THRESHOLD:
        return 0
    return 1 if row["ch3_target"] > 0 else -1


def analyze(path):
    kind, header, records, malformed = read_trace(path)
    if kind != "DRIVE" or not header:
        raise ValueError(f"{path}: kein DRIVE-Header")
    missing = sorted(REQUIRED - set(header))
    if missing:
        raise ValueError(f"{path}: fehlende DRIVE-Spalten: {', '.join(missing)}")
    rows = [row for record_kind, row in records if record_kind == "DRIVE"]
    if not rows:
        raise ValueError(f"{path}: keine vollständigen DRIVE-Zeilen")
    active = [row for row in rows if row["ch5_mode"] >= 1]
    intervals = [b["time_ms"] - a["time_ms"] for a, b in zip(rows, rows[1:])
                 if 0 < b["time_ms"] - a["time_ms"] < 1000]
    first_active = active[0] if active else None

    # A release counts only after at least two consecutive command samples.
    # The window ends at the next command or CH5-off sample.
    releases = []
    command_run = []
    previous_direction = 0
    for index, row in enumerate(rows):
        current = direction(row)
        if current:
            command_run = command_run + [row] if current == previous_direction else [row]
        elif previous_direction and len(command_run) >= 2 and row["ch5_mode"] >= 1:
            window = []
            start_ms = row["time_ms"]
            for candidate in rows[index:]:
                if (candidate["time_ms"] - start_ms > STOP_WINDOW_MS
                        or direction(candidate) or candidate["ch5_mode"] < 1):
                    break
                window.append(candidate)
            if window:
                speeds = [(sample["motor1_velocity_f"] + sample["motor2_velocity_f"]) / 2
                          for sample in window]
                opposite = max(0.0, max(-previous_direction * speed for speed in speeds))
                releases.append({
                    "at_ms": rounded(start_ms, 1),
                    "command_direction": "vorwaerts" if previous_direction > 0 else "rueckwaerts",
                    "command_peak": rounded(max(abs(sample["ch3_target"]) for sample in command_run)),
                    "command_duration_s": rounded((command_run[-1]["time_ms"] - command_run[0]["time_ms"]) / 1000, 2),
                    "speed_before_release": rounded((command_run[-1]["motor1_velocity_f"]
                                                      + command_run[-1]["motor2_velocity_f"]) / 2),
                    "opposite_speed_peak": rounded(opposite),
                    "neutral_samples": len(window),
                })
            command_run = []
        elif not current:
            command_run = []
        previous_direction = current

    total = len(rows) + malformed
    bad_fraction = malformed / total if total else 1.0
    angle_i = [abs(row["angle_out_i"]) for row in active if "angle_out_i" in row]
    motor_targets = [abs(row[field]) for row in active
                     for field in ("motor1_target", "motor2_target")]
    result = {
        "file": str(path),
        "valid_rows": len(rows),
        "malformed_rows": malformed,
        "bad_fraction": rounded(bad_fraction, 4),
        "capture_ok": len(rows) >= MIN_ROWS and bad_fraction <= MAX_BAD_FRACTION,
        "duration_s": rounded((rows[-1]["time_ms"] - rows[0]["time_ms"]) / 1000, 2),
        "median_interval_ms": rounded(statistics.median(intervals), 1) if intervals else None,
        "active_rows": len(active),
        "activation_roll_deg": rounded(first_active["roll_ok"]) if first_active else None,
        "command_rows": sum(bool(direction(row)) for row in rows),
        "releases": releases,
        "opposite_speed_peak": max((item["opposite_speed_peak"] for item in releases), default=None),
        "angle_i_abs_peak": rounded(max(angle_i)) if angle_i else None,
        "motor_target_abs_peak": rounded(max(motor_targets)) if motor_targets else None,
        "voltage_raw_min_v": rounded(min(row["voltage_min_raw_v"] for row in active)) if active else None,
    }
    result["stop_available"] = bool(result["capture_ok"] and releases
                                    and first_active is not None
                                    and abs(first_active["roll_ok"]) <= 10)
    return result


def fmt(value):
    return "—" if value is None else str(value)


def markdown(results):
    lines = ["# DRIVE-Vergleich", "",
             "| Log | gültig/defekt | aktiv | Startneigung | CH3-Zeilen | Stopps | Gegenlauf max. | Winkel-I max. | Motorziel max. | Board-ADC min. |",
             "|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|"]
    for run in results:
        lines.append("| `{file}` | {valid_rows}/{malformed_rows} | {active_rows} | "
                     "{activation_roll} | {command_rows} | {stops} | "
                     "{opposite_speed_peak} | {angle_i_abs_peak} | "
                     "{motor_target_abs_peak} | {voltage} |".format(
                         **{key: fmt(value) for key, value in run.items()},
                         activation_roll=(f"{run['activation_roll_deg']}°"
                                          if run["activation_roll_deg"] is not None else "—"),
                         voltage=(f"{run['voltage_raw_min_v']} V"
                                  if run["voltage_raw_min_v"] is not None else "—"),
                         stops=len(run["releases"])))
    lines.extend(["", "## Auswertbarkeit", ""])
    for run in results:
        issues = []
        if not run["capture_ok"]:
            issues.append(f"Tracequalität unzureich ({run['valid_rows']} gültig, {run['malformed_rows']} defekt)")
        if not run["active_rows"]:
            issues.append("CH5 nie aktiv")
        if not run["command_rows"]:
            issues.append("CH3 blieb neutral")
        if run["command_rows"] and not run["releases"]:
            issues.append("kein vollständiger CH3→neutral-Stopp")
        if run["activation_roll_deg"] is not None and abs(run["activation_roll_deg"]) > 10:
            issues.append("Startneigung über 10°; Vergleich durch Anlauf gestört")
        if not issues:
            issues.append("Stoppereignis vorhanden; für einen Vergleich Impulsstärke und -dauer beachten")
        lines.append(f"- `{run['file']}`: " + "; ".join(issues) + ".")
    if any(run["releases"] for run in results):
        lines.extend(["", "## Einzelne Stopps", "",
                      "| Log | Richtung | CH3 max. | Impulsdauer | Tempo vor Loslassen | Gegenlauf max. |",
                      "|---|---|---:|---:|---:|---:|"])
        for run in results:
            for release in run["releases"]:
                lines.append(f"| `{run['file']}` | {release['command_direction']} | "
                             f"{release['command_peak']} | {release['command_duration_s']} s | "
                             f"{release['speed_before_release']} | {release['opposite_speed_peak']} |")
    lines.extend(["", "Gegenlauf max. ist der größte Betrag der entgegengesetzten mittleren Raddrehzahl "
                  "innerhalb 1 s nach CH3→neutral; Einheit wie im Firmware-Trace. "
                  "Unterschiedliche Impulse sind kein direkter A/B-Vergleich. "
                  "Die Kennzahl beschreibt den Trace, nicht die Ursache oder das sichtbare Fahrverhalten.", ""])
    return "\n".join(lines)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("logs", nargs="+", type=Path, help="ein oder mehrere K58-DRIVE-Logs")
    parser.add_argument("--format", choices=("markdown", "json"), default="markdown")
    parser.add_argument("--output", type=Path, help="Bericht speichern statt auf stdout ausgeben")
    parser.add_argument("--require-stop", action="store_true",
                        help="Fehlerstatus, wenn kein auswertbarer Stopp vorliegt")
    args = parser.parse_args()
    try:
        results = [analyze(path) for path in args.logs]
        report = (json.dumps(results, ensure_ascii=False, indent=2) + "\n" if args.format == "json"
                  else markdown(results))
        if args.output:
            args.output.write_text(report, encoding="utf-8")
            print(f"Gespeichert: {args.output.resolve()}")
        else:
            print(report, end="")
    except (OSError, ValueError) as error:
        print(f"analyze_drive_trace: {error}", file=sys.stderr)
        return 1
    if args.require_stop and not all(result["stop_available"] for result in results):
        return 2
    return 0


if __name__ == "__main__":
    sys.exit(main())
