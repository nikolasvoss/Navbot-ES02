#!/usr/bin/env python3
"""Upload an already compiled OllieFOCdrive Arduino build to the NavBot ES02.

Build first with `python3 scripts/build_firmware.py`, then pass its output directory here.
Uses arduino-cli when it is in PATH, otherwise esptool. Requires pyserial and one of the two.
This script never compiles source code.
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
ESPTOOL_CHIP = "esp32s3"  # the chip in DEFAULT_FQBN
# Offsets for the ESP32-S3 default partition table. The underscore command names work in
# esptool 4.x (apt) and 5.x (pip, with a deprecation warning).
FLASH_LAYOUT = (
    ("OllieFOCdrive.ino.bootloader.bin", "0x0"),
    ("OllieFOCdrive.ino.partitions.bin", "0x8000"),
    ("OllieFOCdrive.ino.bin", "0x10000"),
)
# Erase OTA data so the bootloader starts app0 rather than a slot chosen by an earlier OTA update.
OTADATA_REGION = ("0xe000", "0x2000")
ESPTOOL_FAILED_HINT = (
    "upload_firmware: esptool failed. If it stopped on a missing stub_flasher file, the distro "
    "package (apt) ships without flasher stubs and cannot flash the ESP32-S3: pipx install esptool"
)


def upload_with_arduino_cli(build_dir, port):
    return subprocess.call([
        "arduino-cli", "upload", "--fqbn", DEFAULT_FQBN,
        "--port", port, "--input-dir", str(build_dir),
        "--verify", str(DEFAULT_SKETCH),
    ])


def upload_with_esptool(build_dir, port):
    flash_args = [part for name, offset in FLASH_LAYOUT for part in (offset, str(build_dir / name))]
    # no_reset keeps the chip in its bootloader after the erase, so the old app never boots
    # (and never drives the motors) between the two steps.
    status = run_esptool(port, "--after", "no_reset", "erase_region", *OTADATA_REGION)
    if not status:
        status = run_esptool(port, "write_flash", *flash_args)
    if status:
        print(ESPTOOL_FAILED_HINT, file=sys.stderr)
    return status


def run_esptool(port, *args):
    return subprocess.call(["esptool", "--chip", ESPTOOL_CHIP, "--port", port, *args])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "build_dir",
        type=Path,
        help="directory of binaries made by arduino-cli compile --output-dir",
    )
    parser.add_argument("--port", help="CH340 serial port; auto-detect if exactly one is connected")
    args = parser.parse_args()

    build_dir = args.build_dir.expanduser().resolve()
    if not build_dir.is_dir():
        parser.error(f"build directory does not exist: {build_dir}")
    missing = [name for name, _ in FLASH_LAYOUT if not (build_dir / name).is_file()]
    if missing:
        parser.error(f"missing build files in {build_dir}: {', '.join(missing)}")

    if shutil.which("arduino-cli"):
        upload, uploader = upload_with_arduino_cli, "arduino-cli"
    elif shutil.which("esptool"):
        upload, uploader = upload_with_esptool, "esptool"
    else:
        parser.error("neither arduino-cli nor esptool is in PATH")

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

    print(f"Uploading {build_dir} to {selected.device} ({selected.description})", flush=True)
    print(f"Board: {DEFAULT_FQBN}; uploader: {uploader}", flush=True)
    print("Close SerialPlot and other serial monitors before upload.", flush=True)
    try:
        return upload(build_dir, selected.device)
    except OSError as error:
        print(f"upload_firmware: {error}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
