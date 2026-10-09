#!/usr/bin/env python3
"""Compile an Arduino sketch without uploading it."""

import argparse
import json
import shlex
import shutil
import subprocess
import sys
from dataclasses import dataclass
from pathlib import Path

from firmware_build_config import DEFAULT_FQBN, DEFAULT_OUTPUT_DIR, DEFAULT_SKETCH, REPOSITORY_ROOT


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
        result = subprocess.call(command)
    except OSError as error:
        print(f"build_firmware: could not run arduino-cli: {error}", file=sys.stderr)
        return 1
    if result != 0:
        return result
    try:
        manifest = _build_environment(request)
        manifest_path = REPOSITORY_ROOT / "firmware-build-environment.json"
        temporary_path = manifest_path.with_suffix(".json.tmp")
        temporary_path.write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
        temporary_path.replace(manifest_path)
    except (OSError, ValueError, KeyError, RuntimeError) as error:
        print(f"build_firmware: compiled firmware, but could not write build environment: {error}", file=sys.stderr)
        return 1
    print(f"Build environment: {manifest_path}", flush=True)
    return 0


def _expanded_properties(request: BuildRequest) -> dict[str, str]:
    command = [
        "arduino-cli", "compile", "--fqbn", request.fqbn,
        "--show-properties=expanded", "--build-path", str(request.build_path),
    ]
    for prop in request.build_properties:
        command.extend(("--build-property", prop))
    command.append(str(request.sketch))
    result = subprocess.run(command, capture_output=True, text=True, check=False)
    if result.returncode:
        raise RuntimeError(result.stderr.strip() or "Arduino CLI could not resolve build properties")
    properties = {}
    for line in result.stdout.splitlines():
        key, separator, value = line.partition("=")
        if separator:
            properties[key] = value
    return properties


def _library_versions(compile_commands_path: Path, platform_path: Path) -> list[dict[str, str]]:
    compile_commands = json.loads(compile_commands_path.read_text(encoding="utf-8"))
    library_roots = set()
    for entry in compile_commands:
        arguments = entry.get("arguments")
        if arguments is None:
            arguments = shlex.split(entry["command"])
        include_paths = []
        for index, argument in enumerate(arguments):
            if argument.startswith("-I") and len(argument) > 2:
                include_paths.append(Path(argument[2:]))
            elif argument == "-I" and index + 1 < len(arguments):
                include_paths.append(Path(arguments[index + 1]))
        for include_path in include_paths:
            for parent in (include_path, *include_path.parents):
                properties_path = parent / "library.properties"
                if properties_path.is_file():
                    if platform_path not in properties_path.parents:
                        library_roots.add(parent)
                    break

    libraries = {}
    for root in library_roots:
        properties = {}
        for line in (root / "library.properties").read_text(encoding="utf-8").splitlines():
            key, separator, value = line.partition("=")
            if separator:
                properties[key] = value
        if "name" in properties and "version" in properties:
            library = {
                "name": properties["name"],
                "version": properties["version"],
            }
            if properties.get("url"):
                library["url"] = properties["url"]
            libraries[(library["name"], library["version"], library.get("url", ""))] = library
    return sorted(libraries.values(), key=lambda library: (library["name"].casefold(), library["version"]))


def _build_environment(request: BuildRequest) -> dict[str, object]:
    properties = _expanded_properties(request)
    build_path = Path(properties["build.path"])
    platform_path = Path(properties["runtime.platform.path"])
    platform_properties = {}
    for line in (platform_path / "platform.txt").read_text(encoding="utf-8").splitlines():
        key, separator, value = line.partition("=")
        if separator:
            platform_properties[key] = value
    return {
        "schema_version": 1,
        "fqbn": request.fqbn,
        "platform": {
            "id": ":".join(request.fqbn.split(":")[:2]),
            "version": platform_properties["version"],
        },
        "libraries": _library_versions(build_path / "compile_commands.json", platform_path),
    }


def main() -> int:
    parser = _parser()
    request = _request(parser.parse_args())
    _validate(parser, request)
    return _compile(request)


if __name__ == "__main__":
    sys.exit(main())
