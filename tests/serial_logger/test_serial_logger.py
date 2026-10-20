#!/usr/bin/env python3
"""Compile the logging module against host queue/UART stubs."""

import subprocess
import tempfile
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
SKETCH = ROOT / "src/ES-02/OllieFOCdrive"


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


if __name__ == "__main__":
    main()
