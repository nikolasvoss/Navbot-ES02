#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
build_dir="$(mktemp -d)"
trap 'rm -rf "$build_dir"' EXIT

c++ -std=c++17 -Wall -Wextra -Werror \
  -I"$repo_root/src/sim" \
  -I"$repo_root/src/ES-02/OllieFOCdrive" \
  "$repo_root/src/sim/sensors.cpp" \
  "$repo_root/src/sim/hal.cpp" \
  "$repo_root/src/ES-02/OllieFOCdrive/imu.cpp" \
  "$repo_root/src/ES-02/OllieFOCdrive/filter.cpp" \
  "$repo_root/src/ES-02/OllieFOCdrive/MahonyFilter.cpp" \
  "$repo_root/tests/sim_sensors/test_sim_sensors.cpp" \
  -o "$build_dir/test_sim_sensors"
"$build_dir/test_sim_sensors"
