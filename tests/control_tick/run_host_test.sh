#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
sketch_dir="$repo_root/src/ES-02/OllieFOCdrive"
build_dir="$(mktemp -d)"
trap 'rm -rf "$build_dir"' EXIT

c++ -std=c++17 -Wall -Wextra -Werror \
  -I"$repo_root/src/sim" -I"$sketch_dir" \
  "$sketch_dir/ControlTick.cpp" \
  "$sketch_dir/BalanceController.cpp" \
  "$sketch_dir/OverspeedGuard.cpp" \
  "$sketch_dir/WheelBoundary.cpp" \
  "$sketch_dir/filter.cpp" \
  "$sketch_dir/RobotConfig.cpp" \
  "$repo_root/tests/control_tick/test_control_tick.cpp" \
  -o "$build_dir/test_control_tick"

"$build_dir/test_control_tick"
