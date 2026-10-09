#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
build_dir="$(mktemp -d)"
trap 'rm -rf "$build_dir"' EXIT

c++ -std=c++17 -Wall -Wextra -Werror -Wno-error=reorder \
  -I"$repo_root/tests/imu/mocks" \
  -I"$repo_root/src/ES-02/OllieFOCdrive" \
  "$repo_root/src/ES-02/OllieFOCdrive/imu.cpp" \
  "$repo_root/src/ES-02/OllieFOCdrive/filter.cpp" \
  "$repo_root/src/ES-02/OllieFOCdrive/MahonyFilter.cpp" \
  "$repo_root/tests/imu/test_imu.cpp" \
  -o "$build_dir/test_imu"
"$build_dir/test_imu"

c++ -std=c++17 -Wall -Wextra -Werror -Wno-unused-parameter \
  -I"$repo_root/tests/imu/mocks" \
  -I"$repo_root/src/ES-02/OllieFOCdrive" \
  "$repo_root/src/ES-02/OllieFOCdrive/ICM42688.cpp" \
  "$repo_root/tests/imu/test_driver.cpp" \
  -o "$build_dir/test_driver"
"$build_dir/test_driver"
