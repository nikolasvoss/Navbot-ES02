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

from firmware_build_artifact import read_build_manifest, required_binary_names
from firmware_build_config import DEFAULT_FQBN, DEFAULT_SKETCH

try:
    from serial.tools import list_ports
except ImportError:
    sys.exit("pyserial is required: python3 -m pip install pyserial")


CH340_USB_ID = (0x1A86, 0x7523)
PORT_VISIBILITY_HINT = (
    "If a serial MCP or another host can see the CH340 but this uploader cannot, "
    "the USB device is not exposed to this process. Use the configured navbot_flash "
    "MCP tool or run the uploader on the USB-owning host."
)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "build_dir",
        type=Path,
        help="directory of binaries made by arduino-cli compile --output-dir",
    )
    parser.add_argument("--port", help="CH340 serial port; auto-detect if exactly one is connected")
    parser.add_argument("--fqbn", help="board configuration for a legacy build without a manifest")
    args = parser.parse_args()

    if not shutil.which("arduino-cli"):
        parser.error("arduino-cli is not in PATH")

    build_dir = args.build_dir.expanduser().resolve()
    if not build_dir.is_dir():
        parser.error(f"build directory does not exist: {build_dir}")
    try:
        manifest = read_build_manifest(build_dir)
    except ValueError as error:
        parser.error(str(error))

    if manifest is None:
        fqbn = args.fqbn or DEFAULT_FQBN
        sketch = DEFAULT_SKETCH
        print(
            "Build manifest missing; using the default sketch and board settings. "
            "Rebuild with build_firmware.py to bind settings to the images.",
            file=sys.stderr,
        )
    else:
        fqbn = manifest["fqbn"]
        sketch = Path(manifest["sketch"])
        if args.fqbn and args.fqbn != fqbn:
            parser.error("--fqbn does not match the build manifest")
        if manifest["build_properties"]:
            print(
                "This build used custom build properties. The upload helper reuses its FQBN "
                "but cannot reapply compile-only properties.",
                file=sys.stderr,
            )

    missing = [name for name in required_binary_names(sketch) if not (build_dir / name).is_file()]
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
            message = f"{args.port} is not a detected CH340 (1a86:7523); connected: {[p.device for p in matches]}"
            if not matches:
                message += f". {PORT_VISIBILITY_HINT}"
            parser.error(message)
    elif len(matches) == 1:
        selected = matches[0]
    else:
        if not matches:
            parser.error(f"expected one CH340 (1a86:7523), found []. {PORT_VISIBILITY_HINT}")
        parser.error(f"expected one CH340 (1a86:7523), found {[p.device for p in matches]}; use --port")

    command = [
        "arduino-cli", "upload", "--fqbn", fqbn,
        "--port", selected.device, "--input-dir", str(build_dir),
        "--verify", str(sketch),
    ]
    print(f"Uploading {build_dir} to {selected.device} ({selected.description})", flush=True)
    print(f"Board: {fqbn}; verify: enabled", flush=True)
    if manifest is not None:
        print(f"Partition: {manifest['partition_scheme']}; OTA app slot: {manifest['supports_ota']}", flush=True)
        if manifest["supports_ota"] is False:
            print("Warning: this build has no OTA app slot.", file=sys.stderr)
    print("Close SerialPlot and other serial monitors before upload.", flush=True)
    try:
        return subprocess.call(command)
    except OSError as error:
        print(f"upload_firmware: {error}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
