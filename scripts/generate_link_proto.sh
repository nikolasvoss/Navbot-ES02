#!/usr/bin/env bash
# Regenerates the NavBot link schema code: nanopb C code for the sketch and
# Python protobuf code for the navbot_link library.
# Requires nanopb_generator 0.4.9.1 (pip install nanopb==0.4.9.1) and protoc 3.21.x on PATH.
# The Python output must match the ros2_pc runtime (python3-protobuf 3.21, Ubuntu 24.04
# apt protobuf-compiler). Newer protoc versions emit code that needs a newer runtime.
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
schema_dir="$repo_root/src/navbot_link"
out_dir="$repo_root/src/ES-02/OllieFOCdrive/src/navbot_link"
py_out_dir="$schema_dir"

if ! command -v protoc >/dev/null 2>&1 || ! protoc --version | grep -q '^libprotoc 3\.21\.'; then
  echo "error: protoc 3.21.x is required on PATH (ros2_pc runtime is python3-protobuf 3.21)." >&2
  echo "       found: $(protoc --version 2>/dev/null || echo 'no protoc')" >&2
  exit 1
fi

mkdir -p "$out_dir"
nanopb_generator \
  -I "$schema_dir" \
  -f "$schema_dir/navbot_link.options" \
  -L quote \
  -D "$out_dir" \
  "$schema_dir/navbot_link.proto"

protoc -I "$schema_dir" --python_out="$py_out_dir" navbot_link.proto
