#!/usr/bin/env python3
"""Compile an Arduino sketch without uploading it."""

import argparse
import shutil
import subprocess
import sys
from dataclasses import dataclass
from pathlib import Path

from firmware_build_config import DEFAULT_FQBN, DEFAULT_OUTPUT_DIR, DEFAULT_SKETCH


@dataclass(frozen=True)
class BuildRequest:
    sketch: Path
    fqbn: str
    output_dir: Path
    build_path: Path
    build_properties: tuple[str, ...]
    clean: bool


def _caller_path(value: str) -> Path:
    return Path(value).expanduser().resolve()


def _request(args: argparse.Namespace) -> BuildRequest:
    output_dir = DEFAULT_OUTPUT_DIR if args.output_dir is None else _caller_path(args.output_dir)
    return BuildRequest(
        sketch=DEFAULT_SKETCH if args.sketch is None else _caller_path(args.sketch),
        fqbn=DEFAULT_FQBN if args.fqbn is None else args.fqbn,
        output_dir=output_dir,
        build_path=(
            output_dir.parent / f".{output_dir.name}.arduino-build"
            if args.build_path is None
            else _caller_path(args.build_path)
        ),
        build_properties=tuple(args.build_property),
        clean=args.clean,
    )


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--sketch", help=f"sketch directory (default: {DEFAULT_SKETCH})")
    parser.add_argument("--fqbn", help=f"Arduino fully qualified board name (default: {DEFAULT_FQBN})")
    parser.add_argument("--output-dir", help=f"compiled build output directory (default: {DEFAULT_OUTPUT_DIR})")
    parser.add_argument(
        "--build-path",
        help="Arduino CLI working directory (default: a hidden sibling of the output directory)",
    )
    parser.add_argument(
        "--build-property",
        action="append",
        default=[],
        metavar="KEY=VALUE",
        help="Arduino CLI build property; may be repeated",
    )
    parser.add_argument("--clean", action="store_true", help="discard cached build artifacts before compiling")
    return parser


def _validate(parser: argparse.ArgumentParser, request: BuildRequest) -> None:
    if not request.sketch.is_dir():
        parser.error(f"sketch directory does not exist: {request.sketch}")
    if not any(request.sketch.rglob("*.ino")):
        parser.error(f"no .ino sketch file found under: {request.sketch}")
    if not request.fqbn.strip():
        parser.error("FQBN cannot be empty")
    if not shutil.which("arduino-cli"):
        parser.error("arduino-cli is not in PATH; install Arduino CLI and make it available on PATH")
    if request.build_path == request.output_dir:
        parser.error("build path and output directory must be different")
    try:
        request.output_dir.mkdir(parents=True, exist_ok=True)
        request.build_path.mkdir(parents=True, exist_ok=True)
    except OSError as error:
        parser.error(f"cannot create build or output directory: {error}")
    if not request.output_dir.is_dir():
        parser.error(f"output path is not a directory: {request.output_dir}")


def _compile(request: BuildRequest) -> int:
    command = [
        "arduino-cli", "compile", "--fqbn", request.fqbn,
        "--output-dir", str(request.output_dir), "--build-path", str(request.build_path),
    ]
    for prop in request.build_properties:
        command.extend(("--build-property", prop))
    if request.clean:
        command.append("--clean")
    command.append(str(request.sketch))

    print(f"Building {request.sketch}", flush=True)
    print(f"Board: {request.fqbn}", flush=True)
    print(f"Output: {request.output_dir}", flush=True)
    print(f"Build path: {request.build_path}", flush=True)
    try:
        return subprocess.call(command)
    except OSError as error:
        print(f"build_firmware: could not run arduino-cli: {error}", file=sys.stderr)
        return 1


def main() -> int:
    parser = _parser()
    request = _request(parser.parse_args())
    _validate(parser, request)
    return _compile(request)


if __name__ == "__main__":
    sys.exit(main())
