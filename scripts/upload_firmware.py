#!/usr/bin/env python3
"""Upload an already compiled OllieFOCdrive Arduino build to the NavBot ES02.

Build first with `python3 scripts/build_firmware.py`, then pass its output directory here.
Requires arduino-cli and pyserial. This script never compiles source code.
"""

import argparse
import shutil
import subprocess
import sys
from pathlib import Path

from firmware_build_config import DEFAULT_FQBN, DEFAULT_SKETCH

try:
    from serial.tools import list_ports
except ImportError:
    sys.exit("pyserial is required: python3 -m pip install pyserial")


CH340_USB_ID = (0x1A86, 0x7523)
REQUIRED_BINARIES = (
    "OllieFOCdrive.ino.bin",
    "OllieFOCdrive.ino.bootloader.bin",
    "OllieFOCdrive.ino.partitions.bin",
)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "build_dir",
        type=Path,
        help="directory of binaries made by arduino-cli compile --output-dir",
    )
    parser.add_argument("--port", help="CH340 serial port; auto-detect if exactly one is connected")
    args = parser.parse_args()

    if not shutil.which("arduino-cli"):
        parser.error("arduino-cli is not in PATH")

    build_dir = args.build_dir.expanduser().resolve()
    if not build_dir.is_dir():
        parser.error(f"build directory does not exist: {build_dir}")
    missing = [name for name in REQUIRED_BINARIES if not (build_dir / name).is_file()]
    if missing:
        parser.error(f"missing build files in {build_dir}: {', '.join(missing)}")

    ports = list(list_ports.comports())
    matches = [p for p in ports if (p.vid, p.pid) == CH340_USB_ID]
    if args.port:
        selected = next(
            (p for p in matches if args.port == p.device or Path(args.port).resolve() == Path(p.device).resolve()),
            None,
        )
        if selected is None:
            parser.error(f"{args.port} is not a detected CH340 (1a86:7523); connected: {[p.device for p in matches]}")
    elif len(matches) == 1:
        selected = matches[0]
    else:
        parser.error(f"expected one CH340 (1a86:7523), found {[p.device for p in matches]}; use --port")

    command = [
        "arduino-cli", "upload", "--fqbn", DEFAULT_FQBN,
        "--port", selected.device, "--input-dir", str(build_dir),
        "--verify", str(DEFAULT_SKETCH),
    ]
    print(f"Uploading {build_dir} to {selected.device} ({selected.description})", flush=True)
    print(f"Board: {DEFAULT_FQBN}; verify: enabled", flush=True)
    print("Close SerialPlot and other serial monitors before upload.", flush=True)
    try:
        return subprocess.call(command)
    except OSError as error:
        print(f"upload_firmware: {error}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
