#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "$0")/.." && pwd)"
build_dir="$(mktemp -d)"
trap 'rm -rf "$build_dir"' EXIT

g++ -std=c++17 -Wall -Wextra -Werror \
  -I "$repo_root/tests/calibration_store/support" \
  -I "$repo_root/src/ES-02/OllieFOCdrive" \
  "$repo_root/src/ES-02/OllieFOCdrive/CalibrationStore.cpp" \
  "$repo_root/tests/calibration_store/FakePreferences.cpp" \
  "$repo_root/tests/calibration_store/test_calibration_store.cpp" \
  -o "$build_dir/test_calibration_store"

"$build_dir/test_calibration_store"
