#!/usr/bin/env python3
"""Compile the logging module against host queue/UART stubs."""

import importlib.util
import subprocess
import sys
import tempfile
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
SKETCH = ROOT / "src/ES-02/OllieFOCdrive"
# Sentinel columns: the C++ test writes column index + 1 into each DRIVE column.
DRIVE_SENTINELS = {"sequence": 30, "motor2_shaft_velocity": 26, "battery_filtered_instant_v": 28}


def drive_columns():
    spec = importlib.util.spec_from_file_location("capture_balance_trace", ROOT / "scripts/capture_balance_trace.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return list(module.DRIVE_COLUMNS)


def check_drive_row(executable):
    """The DRIVE row must have one field per header column, in header order, with its own value."""
    output = subprocess.run([str(executable), "drive"], check=True, cwd=ROOT, capture_output=True, text=True).stdout
    tag, _, body = output.strip().partition(",")
    columns = drive_columns()
    values = body.split(",")
    problems = []
    if tag != "DRIVE":
        problems.append("row tag is %r, expected 'DRIVE'" % tag)
    if len(values) != len(columns):
        problems.append("row has %d fields after the tag, header has %d columns" % (len(values), len(columns)))
    else:
        for index, (name, value) in enumerate(zip(columns, values)):
            if float(value) != index + 1:
                problems.append("column %s holds %r, expected %d" % (name, value, index + 1))
        for name, expected in DRIVE_SENTINELS.items():
            if name in columns and float(values[columns.index(name)]) != expected:
                problems.append("sentinel column %s does not hold %d" % (name, expected))
    if problems:
        print("\n".join(problems), file=sys.stderr)
        raise SystemExit("DRIVE row does not match the header in scripts/capture_balance_trace.py")
    print("drive_row_columns=%d header_matched=yes" % len(columns))


def main():
    with tempfile.TemporaryDirectory(prefix="serial-logger-check-") as directory:
        executable = Path(directory) / "serial_logger_check"
        subprocess.run([
            "g++", "-std=c++11", "-Wall", "-Wextra", "-Werror", "-Wno-switch", "-pthread",
            "-I", str(ROOT / "tests/serial_logger/support"),
            "-I", str(SKETCH),
            str(ROOT / "tests/serial_logger/test_serial_logger.cpp"),
            str(SKETCH / "Logging.cpp"),
            str(SKETCH / "LogFormat.cpp"),
            str(ROOT / "tests/serial_logger/support/host_stubs.cpp"),
            "-o", str(executable),
        ], check=True)
        for mode in ("format", "queue", "messages", "writefail", "startfail", "pace"):
            arguments = [str(executable), mode]
            if mode == "format":
                arguments.append(str(ROOT / "tests/serial_logger/logging_formats.txt"))
            subprocess.run(arguments, check=True, cwd=ROOT)
        check_drive_row(executable)


if __name__ == "__main__":
    main()
