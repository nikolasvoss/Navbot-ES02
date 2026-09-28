#!/usr/bin/env python3
"""Capture one K58 trial and immediately write a compact DRIVE report."""

import argparse
import datetime as dt
import subprocess
import sys
from pathlib import Path

from analyze_drive_trace import analyze, markdown


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path,
                        help="new log path (default: drive-YYYYMMDD-HHMMSS.log)")
    parser.add_argument("--seconds", type=float, default=25)
    parser.add_argument("--port", help="CH340 serial port; default from capture_balance_trace.py")
    parser.add_argument("--baud-rate", type=int, default=115200)
    parser.add_argument("--interactive", action="store_true",
                        help="allow documented live-gain commands during capture")
    args = parser.parse_args()
    if args.seconds <= 0:
        parser.error("--seconds must be positive")
    log_path = args.output or Path(f"drive-{dt.datetime.now():%Y%m%d-%H%M%S}.log")
    report_path = log_path.with_suffix(".md")
    if log_path.exists() or report_path.exists():
        parser.error(f"Ausgabe existiert bereits: {log_path} oder {report_path}")
    if not log_path.parent.is_dir():
        parser.error(f"Verzeichnis fehlt: {log_path.parent}")

    command = [sys.executable, str(Path(__file__).with_name("capture_balance_trace.py")),
               "--trace-mode", "58", "--baud-rate", str(args.baud_rate),
               "--seconds", str(args.seconds), "--output", str(log_path)]
    if args.port:
        command.extend(["--port", args.port])
    if args.interactive:
        command.append("--interactive")

    print("K58-Versuch: CH3/CH4 zunächst neutral, CH9/CH10 mittig; ",
          "CH5 und Startlage prüfen. Nach Aufnahmebeginn CH5 aktivieren, ",
          "CH3-Impuls geben und auf neutral zurückstellen.", sep="", flush=True)
    print(f"Log: {log_path}; Bericht: {report_path}", flush=True)
    try:
        with subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                              text=True, bufsize=1) as process:
            for line in process.stdout:
                if not line.startswith(("DRIVE,", "Summarize this run with:")):
                    print(line, end="", flush=True)
            returncode = process.wait()
    except OSError as error:
        print(f"run_drive_trial: {error}", file=sys.stderr)
        return 1

    if returncode:
        print(f"Aufnahme fehlgeschlagen (Exit {returncode}); vorhandenes Log behalten: {log_path}",
              file=sys.stderr)
        return returncode
    try:
        result = analyze(log_path)
        report_path.write_text(markdown([result]), encoding="utf-8")
    except (OSError, ValueError) as error:
        print(f"run_drive_trial: {error}; Log behalten: {log_path}", file=sys.stderr)
        return 1
    print(f"Bericht: {report_path.resolve()}")
    print(f"Daten: {result['valid_rows']} DRIVE-Zeilen, {result['malformed_rows']} defekt.")
    if not result["capture_ok"]:
        print("Tracequalität unzureich; Bericht und Log prüfen.", file=sys.stderr)
        return 2
    if not result["stop_available"]:
        print("Kein auswertbarer Stoppvergleich; Bericht zeigt den Grund.", file=sys.stderr)
        return 2
    print("Schicke für den Vergleich den Berichtpfad und dein beobachtetes Fahrverhalten.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
