#!/usr/bin/env bash
set -euo pipefail
repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
build_dir="$(mktemp -d)"
trap 'rm -rf "$build_dir"' EXIT
g++ -std=c++17 -Wall -Wextra -Werror \
  -I "$repo_root/src/sim" -I "$repo_root/src/ES-02/OllieFOCdrive" \
  "$repo_root/src/sim/plant.cpp" \
  "$repo_root/src/sim/motor.cpp" \
  "$repo_root/tests/sim_model/test_sim_model.cpp" \
  -o "$build_dir/test_sim_model"
"$build_dir/test_sim_model"
