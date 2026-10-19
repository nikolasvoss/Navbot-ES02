#!/usr/bin/env bash
# Regenerates the nanopb C code for the NavBot link schema into the sketch.
# Requires nanopb_generator 0.4.9.1 (pip install nanopb==0.4.9.1) and protoc on PATH
# (pip install grpcio-tools provides one). Override the generator with
#   NANOPB_GENERATOR=/path/to/nanopb_generator.
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
schema_dir="$repo_root/src/navbot_link"
out_dir="$repo_root/src/ES-02/OllieFOCdrive/src/navbot_link"

mkdir -p "$out_dir"
"${NANOPB_GENERATOR:-nanopb_generator}" \
  -I "$schema_dir" \
  -f "$schema_dir/navbot_link.options" \
  -L quote \
  -D "$out_dir" \
  "$schema_dir/navbot_link.proto"
