#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
build_dir="$(mktemp -d)"
trap 'rm -rf "$build_dir"' EXIT

c++ -std=c++11 -Wall -Wextra -Werror \
  -I"$repo_root/src/ES-02/OllieFOCdrive" \
  "$repo_root/src/ES-02/OllieFOCdrive/Calibration.cpp" \
  "$repo_root/tests/calibration/test_calibration.cpp" \
  -o "$build_dir/test_calibration"

"$build_dir/test_calibration"
