#!/usr/bin/env bash
set -euo pipefail
repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
build_dir="$(mktemp -d)"
trap 'rm -rf "$build_dir"' EXIT
g++ -std=c++17 -Wall -Wextra -Werror \
  "$repo_root/tests/leg_pose_ramp/test_leg_pose_ramp.cpp" \
  -o "$build_dir/test_leg_pose_ramp"
"$build_dir/test_leg_pose_ramp"
