"""Metadata shared by the firmware builder and uploader."""

import hashlib
import json
import os
from pathlib import Path
from typing import Any


MANIFEST_NAME = "firmware-build.json"
MANIFEST_SCHEMA_VERSION = 1


def required_binary_names(sketch: Path) -> tuple[str, ...]:
    stem = sketch.name
    return (
        f"{stem}.ino.bin",
        f"{stem}.ino.bootloader.bin",
        f"{stem}.ino.partitions.bin",
    )


def write_build_manifest(
    output_dir: Path,
    *,
    sketch: Path,
    fqbn: str,
    partition_scheme: str | None,
    supports_ota: bool | None,
    app_size_bytes: int,
    app_limit_bytes: int | None,
    build_properties: tuple[str, ...],
) -> None:
    hashes = {}
    for name in required_binary_names(sketch):
        path = output_dir / name
        if not path.is_file():
            raise ValueError(f"compiled build is missing {name}")
        hashes[name] = hashlib.sha256(path.read_bytes()).hexdigest()

    manifest = {
        "schema_version": MANIFEST_SCHEMA_VERSION,
        "sketch": str(sketch.resolve()),
        "fqbn": fqbn,
        "partition_scheme": partition_scheme,
        "supports_ota": supports_ota,
        "app_size_bytes": app_size_bytes,
        "app_limit_bytes": app_limit_bytes,
        "build_properties": list(build_properties),
        "binary_sha256": hashes,
    }
    path = output_dir / MANIFEST_NAME
    temporary_path = path.with_suffix(path.suffix + ".tmp")
    temporary_path.write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
    os.replace(temporary_path, path)


def read_build_manifest(output_dir: Path) -> dict[str, Any] | None:
    path = output_dir / MANIFEST_NAME
    if not path.exists():
        return None
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        raise ValueError(f"cannot read build manifest {path}: {error}") from error
    if (
        not isinstance(value, dict)
        or type(value.get("schema_version")) is not int
        or value["schema_version"] != MANIFEST_SCHEMA_VERSION
    ):
        raise ValueError(f"unsupported build manifest schema in {path}")

    required_fields = {
        "sketch", "fqbn", "partition_scheme", "supports_ota", "app_size_bytes",
        "app_limit_bytes", "build_properties", "binary_sha256",
    }
    if not required_fields.issubset(value):
        raise ValueError(f"incomplete build manifest in {path}")

    sketch_value = value.get("sketch")
    fqbn = value.get("fqbn")
    hashes = value.get("binary_sha256")
    if not isinstance(sketch_value, str) or not isinstance(fqbn, str) or not fqbn:
        raise ValueError(f"invalid sketch or FQBN in build manifest {path}")
    if not isinstance(hashes, dict):
        raise ValueError(f"invalid binary hashes in build manifest {path}")
    scheme = value.get("partition_scheme")
    supports_ota = value.get("supports_ota")
    app_size = value.get("app_size_bytes")
    app_limit = value.get("app_limit_bytes")
    build_properties = value.get("build_properties")
    if scheme is not None and not isinstance(scheme, str):
        raise ValueError(f"invalid partition scheme in build manifest {path}")
    if supports_ota is not None and type(supports_ota) is not bool:
        raise ValueError(f"invalid OTA status in build manifest {path}")
    if type(app_size) is not int or app_size < 0:
        raise ValueError(f"invalid application size in build manifest {path}")
    if app_limit is not None and (type(app_limit) is not int or app_limit < 0):
        raise ValueError(f"invalid application limit in build manifest {path}")
    if not isinstance(build_properties, list) or not all(isinstance(item, str) for item in build_properties):
        raise ValueError(f"invalid build properties in build manifest {path}")

    sketch = Path(sketch_value)
    expected_names = required_binary_names(sketch)
    if set(hashes) != set(expected_names):
        raise ValueError(f"build manifest {path} does not list the expected firmware images")
    for name in expected_names:
        digest = hashes[name]
        if not isinstance(digest, str) or len(digest) != 64:
            raise ValueError(f"invalid SHA-256 for {name} in build manifest {path}")
        image_path = output_dir / name
        if not image_path.is_file():
            raise ValueError(f"build image is missing: {image_path}")
        actual_digest = hashlib.sha256(image_path.read_bytes()).hexdigest()
        if actual_digest != digest:
            raise ValueError(f"build image does not match {path}: {name}")

    if not sketch.is_dir():
        raise ValueError(f"manifest sketch directory does not exist: {sketch}")
    return value
