#!/usr/bin/env bash
# Compares the DebugTelemetry sample stream with golden_stream.txt, which was recorded from the
# print_data() implementation that DebugTelemetry replaced and re-recorded after each later deletion.
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
build_dir="$(mktemp -d)"
trap 'rm -rf "$build_dir"' EXIT

c++ -std=c++17 -Wall -Wextra -Werror \
  -I"$repo_root/tests/telemetry/support" \
  -I"$repo_root/src/ES-02/OllieFOCdrive" \
  "$repo_root/src/ES-02/OllieFOCdrive/DebugTelemetry.cpp" \
  "$repo_root/tests/telemetry/support/RecordingStubs.cpp" \
  "$repo_root/tests/telemetry/record_stream.cpp" \
  -o "$build_dir/record_stream"

"$build_dir/record_stream" > "$build_dir/stream.txt"
if ! diff -q "$repo_root/tests/telemetry/golden_stream.txt" "$build_dir/stream.txt" >/dev/null; then
  diff "$repo_root/tests/telemetry/golden_stream.txt" "$build_dir/stream.txt" | head -20
  echo "telemetry stream differs from golden_stream.txt" >&2
  exit 1
fi
echo "telemetry stream matches golden_stream.txt"
