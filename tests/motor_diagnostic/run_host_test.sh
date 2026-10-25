#!/usr/bin/env bash
set -euo pipefail
repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
build_dir="$(mktemp -d)"
trap 'rm -rf "$build_dir"' EXIT
g++ -std=c++17 -Wall -Wextra -Werror \
  "$repo_root/src/ES-02/OllieFOCdrive/MotorDiagnostic.cpp" \
  "$repo_root/tests/motor_diagnostic/test_motor_diagnostic.cpp" \
  -o "$build_dir/test_motor_diagnostic"
"$build_dir/test_motor_diagnostic"
