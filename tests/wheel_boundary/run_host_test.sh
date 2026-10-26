#!/usr/bin/env bash
set -euo pipefail
repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
build_dir="$(mktemp -d)"
trap 'rm -rf "$build_dir"' EXIT
g++ -std=c++17 -Wall -Wextra -Werror \
  "$repo_root/src/ES-02/OllieFOCdrive/WheelBoundary.cpp" \
  "$repo_root/tests/wheel_boundary/test_wheel_boundary.cpp" \
  -o "$build_dir/test_wheel_boundary"
"$build_dir/test_wheel_boundary"
