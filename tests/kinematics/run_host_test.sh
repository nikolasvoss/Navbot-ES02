#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
build_dir="$(mktemp -d)"
trap 'rm -rf "$build_dir"' EXIT

c++ -std=c++11 -Wall -Wextra -Werror \
  -I "$repo_root/src/ES-02/OllieFOCdrive" \
  "$repo_root/src/ES-02/OllieFOCdrive/LegKinematics.cpp" \
  "$repo_root/tests/kinematics/leg_kinematics_baseline.cpp" \
  "$repo_root/tests/kinematics/test_leg_kinematics.cpp" \
  -o "$build_dir/test_leg_kinematics"
"$build_dir/test_leg_kinematics"
