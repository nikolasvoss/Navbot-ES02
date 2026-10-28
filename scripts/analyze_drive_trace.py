#!/usr/bin/env python3
"""Check drive captures and compare repeatable drive/stop measurements.

Reads navlink CSVs taken with the `drive` preset (see REQUIRED; `time_ms` comes from `t_us`). This reads saved logs
only. It never opens a serial port or changes the robot.
"""

import argparse
import json
import statistics
import sys
from pathlib import Path

from summarize_balance_trace import imu_sample_rate, read_trace


REQUIRED = {
    "time_ms", "ch5_mode", "ch3_target", "motor1_velocity_f",
    "motor2_velocity_f", "roll_deg", "motor1_target", "motor2_target",
    "voltage_min_raw_v", "imu_sequence", "imu_dt_s", "imu_accepted_samples",
    "imu_long_gap_count", "imu_max_interval_us", "control_execution_us",
    "loop_interval_max_us", "motor1_current_sp", "motor2_current_sp",
    "motor1_voltage_q", "motor2_voltage_q", "battery_raw_instant_v",
    "imu_read_time_us",
}
MIN_ROWS = 20
MAX_BAD_FRACTION = 0.02
NEUTRAL_THRESHOLD = 1.0
STOP_WINDOW_MS = 1000
UINT32_MODULO = 2 ** 32


def rounded(value, places=3):
    return None if value is None else round(value, places)


def direction(row):
    if row["ch5_mode"] < 1 or abs(row["ch3_target"]) < NEUTRAL_THRESHOLD:
        return 0
    return 1 if row["ch3_target"] > 0 else -1


def analyze(path):
    _, header, records, malformed, notes = read_trace(path)
    missing = sorted(REQUIRED - set(header))
    if missing:
        raise ValueError(f"{path}: fehlende DRIVE-Spalten: {', '.join(missing)}")
    rows = [row for _, row in records]
    if not rows:
        raise ValueError(f"{path}: keine vollständigen DRIVE-Zeilen")
    active = [row for row in rows if row["ch5_mode"] >= 1]
    intervals = [b["time_ms"] - a["time_ms"] for a, b in zip(rows, rows[1:]) if b["time_ms"] > a["time_ms"]]
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
    motor_targets = [abs(row[field]) for row in active
                     for field in ("motor1_target", "motor2_target")]
    imu_sequence_advance = int(rows[-1]["imu_sequence"] - rows[0]["imu_sequence"])
    imu_read_advance_us = int(rows[-1]["imu_read_time_us"] - rows[0]["imu_read_time_us"]) % UINT32_MODULO
    imu_accepted = int(rows[-1]["imu_accepted_samples"] - rows[0]["imu_accepted_samples"])
    imu_long_gaps = int(rows[-1]["imu_long_gap_count"] - rows[0]["imu_long_gap_count"])
    result = {
        "file": str(path),
        "valid_rows": len(rows),
        "malformed_rows": malformed,
        "bad_fraction": rounded(bad_fraction, 4),
        "capture_ok": len(rows) >= MIN_ROWS and bad_fraction <= MAX_BAD_FRACTION,
        "duration_s": rounded((rows[-1]["time_ms"] - rows[0]["time_ms"]) / 1000, 2),
        "median_interval_ms": rounded(statistics.median(intervals), 1) if intervals else None,
        "active_rows": len(active),
        "activation_roll_deg": rounded(first_active["roll_deg"]) if first_active else None,
        "command_rows": sum(bool(direction(row)) for row in rows),
        "releases": releases,
        "opposite_speed_peak": max((item["opposite_speed_peak"] for item in releases), default=None),
        "motor_target_abs_peak": rounded(max(motor_targets)) if motor_targets else None,
        "voltage_raw_min_v": rounded(min(row["voltage_min_raw_v"] for row in active)) if active else None,
        # IMU counters are cumulative since boot; per-row loop interval resets on each row.
        "imu_rate_hz": rounded(imu_sample_rate(rows), 1),
        "imu_mean_interval_s": (rounded(imu_read_advance_us / imu_sequence_advance / 1e6, 6)
                                if imu_sequence_advance > 0 else None),
        "imu_long_gaps": imu_long_gaps,
        "imu_long_gap_fraction": rounded(imu_long_gaps / imu_accepted, 4) if imu_accepted > 0 else None,
        "imu_max_interval_us": int(rows[-1]["imu_max_interval_us"]),
        "control_execution_us_peak": int(max(row["control_execution_us"] for row in rows)),
        "loop_entry_interval_us_peak": int(max(row["loop_interval_max_us"] for row in rows)),
        "motor_current_sp_abs_peak": rounded(max(abs(row[name]) for row in rows
                                                 for name in ("motor1_current_sp", "motor2_current_sp"))),
        "motor_voltage_q_abs_peak": rounded(max(abs(row[name]) for row in rows
                                                for name in ("motor1_voltage_q", "motor2_voltage_q"))),
        "battery_raw_instant_min_v": rounded(min(row["battery_raw_instant_v"] for row in rows)),
    }
    if notes.get("warnings"):
        result["warnings"] = notes["warnings"]
    result["stop_available"] = bool(result["capture_ok"] and releases
                                    and first_active is not None
                                    and abs(first_active["roll_deg"]) <= 10)
    return result


def fmt(value):
    return "—" if value is None else str(value)


def markdown(results):
    lines = ["# DRIVE-Vergleich", "",
             "| Log | gültig/defekt | aktiv | Startneigung | CH3-Zeilen | Stopps | Gegenlauf max. | Motorziel max. | Board-ADC min. |",
             "|---|---:|---:|---:|---:|---:|---:|---:|---:|"]
    for run in results:
        lines.append("| `{file}` | {valid_rows}/{malformed_rows} | {active_rows} | "
                     "{activation_roll} | {command_rows} | {stops} | "
                     "{opposite_speed_peak} | "
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
        issues.extend(f"Warnung: {warning}" for warning in run.get("warnings", []))
        lines.append(f"- `{run['file']}`: " + "; ".join(issues) + ".")
    lines.extend(["", "## Zeit- und Motorgrößen", "",
                  "| Log | IMU-Rate | IMU-Intervall Mittel | IMU-Lücken (Anteil) | IMU-Intervall max. (seit Start) "
                  "| Regelzeit max. | Schleifenintervall max. | Strom-Soll max. | Spannung q max. "
                  "| Bus roh min. (momentan) |",
                  "|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|"])
    for run in results:
        gap_fraction = run["imu_long_gap_fraction"]
        gaps = f"{run['imu_long_gaps']} ({gap_fraction * 100:.2f} %)" if gap_fraction is not None else str(run["imu_long_gaps"])
        lines.append(f"| `{run['file']}` | {fmt(run['imu_rate_hz'])} Hz | {fmt(run['imu_mean_interval_s'])} s "
                     f"| {gaps} | {run['imu_max_interval_us']} µs "
                     f"| {run['control_execution_us_peak']} µs | {run['loop_entry_interval_us_peak']} µs "
                     f"| {fmt(run['motor_current_sp_abs_peak'])} A | {fmt(run['motor_voltage_q_abs_peak'])} V "
                     f"| {fmt(run['battery_raw_instant_min_v'])} V |")
    lines.extend(["", "Strom-Soll und Spannung q sind Motorbetrag-Maxima je Rad (`motor1`/`motor2`, motor-lokales Vorzeichen). "
                  "IMU-Intervall Mittel ist Lesezeit-Differenz durch Sequenz-Differenz im Fenster der gültigen Zeilen. "
                  "IMU-Lücken zählt den Zuwachs seit Capture-Start; der Anteil bezieht sich auf die akzeptierten Samples "
                  "im selben Fenster. Das Intervall-Maximum ist seit Firmware-Start kumulativ. "
                  "Regelzeit und Schleifenintervall gelten je Zeile; ihr Maximum ist über alle Zeilen gebildet.", ""])
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
    parser.add_argument("logs", nargs="+", type=Path, help="eine oder mehrere navlink-CSVs des drive-Presets")
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
