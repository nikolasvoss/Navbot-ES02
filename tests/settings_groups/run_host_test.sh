#!/usr/bin/env bash
set -euo pipefail
repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
build_dir="$(mktemp -d)"
trap 'rm -rf "$build_dir"' EXIT
g++ -std=c++17 -Wall -Wextra -Werror \
  -I"$repo_root/src/ES-02/OllieFOCdrive" \
  "$repo_root/src/ES-02/OllieFOCdrive/BalanceController.cpp" \
  "$repo_root/tests/settings_groups/test_settings_groups.cpp" \
  -o "$build_dir/test_settings_groups"
"$build_dir/test_settings_groups"
