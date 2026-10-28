#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
sketch_dir="$repo_root/src/ES-02/OllieFOCdrive"
nanopb_dir="$sketch_dir/src/navbot_link"
build_dir="$(mktemp -d)"
trap 'rm -rf "$build_dir"' EXIT

for source in pb_common pb_encode pb_decode navbot_link.pb; do
  cc -std=c99 -Wall -Wextra -Werror -c "$nanopb_dir/$source.c" -o "$build_dir/$source.o"
done

c++ -std=c++11 -Wall -Wextra -Werror \
  -I"$sketch_dir" \
  "$sketch_dir/link_frame.cpp" \
  "$repo_root/tests/link/test_link_frame.cpp" \
  "$build_dir/pb_common.o" "$build_dir/pb_encode.o" "$build_dir/pb_decode.o" "$build_dir/navbot_link.pb.o" \
  -o "$build_dir/test_link_frame"

"$build_dir/test_link_frame"

c++ -std=c++11 -Wall -Wextra -Werror \
  -I"$sketch_dir" \
  "$repo_root/tests/link/test_link_schedule.cpp" \
  -o "$build_dir/test_link_schedule"

"$build_dir/test_link_schedule"

c++ -std=c++11 -Wall -Wextra -Werror \
  -I"$sketch_dir" \
  "$sketch_dir/link_frame.cpp" \
  "$sketch_dir/link_tx.cpp" \
  "$repo_root/tests/link/test_link_tx.cpp" \
  "$build_dir/pb_common.o" "$build_dir/pb_encode.o" "$build_dir/pb_decode.o" "$build_dir/navbot_link.pb.o" \
  -o "$build_dir/test_link_tx"

"$build_dir/test_link_tx"

c++ -std=c++11 -Wall -Wextra -Werror \
  -I"$sketch_dir" \
  "$sketch_dir/link_frame.cpp" \
  "$repo_root/tests/link/test_link_rx.cpp" \
  "$build_dir/pb_common.o" "$build_dir/pb_encode.o" "$build_dir/pb_decode.o" "$build_dir/navbot_link.pb.o" \
  -o "$build_dir/test_link_rx"

"$build_dir/test_link_rx"

c++ -std=c++17 -Wall -Wextra -Werror \
  -I"$sketch_dir" -I"$repo_root/tests/link" \
  "$repo_root/tests/link/test_signal_table.cpp" \
  -o "$build_dir/test_signal_table"

"$build_dir/test_signal_table"

c++ -std=c++17 -Wall -Wextra -Werror \
  -I"$sketch_dir" \
  "$sketch_dir/link_subscriptions.cpp" \
  "$repo_root/tests/link/test_link_subscriptions.cpp" \
  -o "$build_dir/test_link_subscriptions"

"$build_dir/test_link_subscriptions"

# Each bad entry file must stop the build with the named reason.
expect_rejected() {
  local file="$1" reason="$2"
  if c++ -std=c++17 -fsyntax-only -I"$sketch_dir" -I"$repo_root/tests/link" \
      -DSIGNAL_DEF_FILE="\"$file\"" "$repo_root/tests/link/reject_signals.cpp" 2> "$build_dir/reject.log"; then
    echo "$file: FAIL (compiled)"
    exit 1
  fi
  grep -q "$reason" "$build_dir/reject.log" || { echo "$file: FAIL (no '$reason' in the error)"; exit 1; }
  echo "$file: rejected ($reason)"
}

expect_rejected reject_duplicate.def.h "duplicate signal name"
expect_rejected reject_long_name.def.h "longer than 24 characters"
expect_rejected reject_wide_type.def.h "at most 32 bits"
