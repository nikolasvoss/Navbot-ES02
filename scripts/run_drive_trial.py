#!/usr/bin/env python3
"""Capture one drive trial with the navlink `drive` preset and immediately write a compact report."""

import argparse
import datetime as dt
import sys
from pathlib import Path

from analyze_drive_trace import analyze, markdown
from navlink import main as navlink_main


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path,
                        help="new CSV path (default: drive-YYYYMMDD-HHMMSS.csv)")
    parser.add_argument("--seconds", type=float, default=25)
    parser.add_argument("--port", required=True,
                        help="tcp:127.0.0.1:7650 (the link multiplexer, which owns /dev/ttyAMA3) or /dev/ttyUSB0")
    parser.add_argument("--baud", type=int, help="default: navlink's baud rate for the port")
    args = parser.parse_args()
    if args.seconds <= 0:
        parser.error("--seconds must be positive")
    log_path = args.output or Path(f"drive-{dt.datetime.now():%Y%m%d-%H%M%S}.csv")
    report_path = log_path.with_suffix(".md")
    if log_path.exists() or report_path.exists():
        parser.error(f"Ausgabe existiert bereits: {log_path} oder {report_path}")
    if not log_path.parent.is_dir():
        parser.error(f"Verzeichnis fehlt: {log_path.parent}")

    print("Drive-Versuch: CH3/CH4 zunächst neutral, CH9/CH10 mittig; ",
          "CH5 und Startlage prüfen. Nach Aufnahmebeginn CH5 auf Position 2 (das ferne Ende, RC-Balance) schalten, ",
          "CH3-Impuls geben und auf neutral zurückstellen.", sep="", flush=True)
    print(f"Log: {log_path}; Bericht: {report_path}", flush=True)
    capture = ["capture", "--port", args.port, "--preset", "drive",
               "--seconds", str(args.seconds), "-o", str(log_path)]
    if args.baud:
        capture += ["--baud", str(args.baud)]
    returncode = navlink_main(capture)
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
    print(f"Daten: {result['valid_rows']} Zeilen, {result['malformed_rows']} defekt.")
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
