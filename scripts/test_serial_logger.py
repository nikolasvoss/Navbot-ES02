#!/usr/bin/env python3
"""Compile the device formatter and sender against host queue/UART stubs."""

import subprocess
import tempfile
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SKETCH = ROOT / "src/ES-02/OllieFOCdrive"


def main():
    with tempfile.TemporaryDirectory(prefix="serial-logger-check-") as directory:
        executable = Path(directory) / "serial_logger_check"
        subprocess.run([
            "g++", "-std=c++17", "-pthread",
            "-I", str(ROOT / "scripts/serial_logger_stubs"),
            "-I", str(SKETCH),
            str(ROOT / "scripts/test_serial_logger.cpp"),
            str(SKETCH / "SerialLogger.cpp"),
            str(ROOT / "scripts/serial_logger_stubs/host_stubs.cpp"),
            "-o", str(executable),
        ], check=True)
        subprocess.run([str(executable)], check=True)
        subprocess.run([str(executable), "overload"], check=True)


if __name__ == "__main__":
    main()
