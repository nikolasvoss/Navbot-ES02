#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
sketch_dir="$repo_root/src/ES-02/OllieFOCdrive"
nanopb_dir="$sketch_dir/src/navbot_link"
build_dir="$(mktemp -d)"
trap 'rm -rf "$build_dir"' EXIT

for source in pb_common pb_encode navbot_link.pb; do
  cc -std=c99 -Wall -Wextra -Werror -c "$nanopb_dir/$source.c" -o "$build_dir/$source.o"
done

c++ -std=c++11 -Wall -Wextra -Werror \
  -I"$sketch_dir" \
  "$sketch_dir/link_frame.cpp" \
  "$repo_root/tests/link/test_link_frame.cpp" \
  "$build_dir/pb_common.o" "$build_dir/pb_encode.o" "$build_dir/navbot_link.pb.o" \
  -o "$build_dir/test_link_frame"

"$build_dir/test_link_frame"
