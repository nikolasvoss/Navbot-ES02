#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
build_dir="$(mktemp -d)"
trap 'rm -rf "$build_dir"' EXIT

c++ -std=c++11 -Wall -Wextra -Werror \
  -I"$repo_root/tests/pid/support" \
  -I"$repo_root/src/ES-02/OllieFOCdrive" \
  "$repo_root/src/ES-02/OllieFOCdrive/filter.cpp" \
  "$repo_root/tests/pid/test_pid_controller.cpp" \
  -lm -o "$build_dir/test_pid_controller"

"$build_dir/test_pid_controller"
