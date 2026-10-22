#!/usr/bin/env bash
set -euo pipefail
repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
build_dir="$(mktemp -d)"
trap 'rm -rf "$build_dir"' EXIT
g++ -std=c++17 -Wall -Wextra -Werror \
  "$repo_root/src/ES-02/OllieFOCdrive/OverspeedGuard.cpp" \
  "$repo_root/tests/overspeed_guard/test_overspeed_guard.cpp" \
  -o "$build_dir/test_overspeed_guard"
"$build_dir/test_overspeed_guard"
