#!/usr/bin/env bash
set -euo pipefail
repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
build_dir="$(mktemp -d)"
trap 'rm -rf "$build_dir"' EXIT
g++ -std=c++17 -Wall -Wextra -Werror \
  "$repo_root/tests/rc_mode/test_rc_mode.cpp" \
  -o "$build_dir/test_rc_mode"
"$build_dir/test_rc_mode"
