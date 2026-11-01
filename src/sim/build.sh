#!/usr/bin/env bash
# Builds the host simulation into OUT_DIR/navbot_sim (plan, T4). Prints nothing on success.
set -euo pipefail

if [ "$#" -ne 1 ]; then
  echo "usage: build.sh OUT_DIR" >&2
  exit 2
fi

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
sketch_dir="$repo_root/src/ES-02/OllieFOCdrive"
nanopb_dir="$sketch_dir/src/navbot_link"
out_dir="$1"
mkdir -p "$out_dir"
build_dir="$(mktemp -d)"
trap 'rm -rf "$build_dir"' EXIT

for source in pb_common pb_encode pb_decode navbot_link.pb; do
  cc -std=c99 -Wall -Wextra -Werror -c "$nanopb_dir/$source.c" -o "$build_dir/$source.o"
done

c++ -std=c++17 -O2 -Wall -Wextra -Werror \
  -I"$repo_root/src/sim" -I"$sketch_dir" \
  "$repo_root"/src/sim/*.cpp \
  "$sketch_dir/BalanceController.cpp" \
  "$sketch_dir/ControlTick.cpp" \
  "$sketch_dir/OverspeedGuard.cpp" \
  "$sketch_dir/RobotConfig.cpp" \
  "$sketch_dir/WheelBoundary.cpp" \
  "$sketch_dir/filter.cpp" \
  "$sketch_dir/imu.cpp" \
  "$sketch_dir/MahonyFilter.cpp" \
  "$sketch_dir/Logging.cpp" \
  "$sketch_dir/link_frame.cpp" \
  "$sketch_dir/link_tx.cpp" \
  "$sketch_dir/link_subscriptions.cpp" \
  "$sketch_dir/link_endpoint.cpp" \
  "$sketch_dir/link_log.cpp" \
  "$build_dir"/*.o \
  -o "$out_dir/navbot_sim"
