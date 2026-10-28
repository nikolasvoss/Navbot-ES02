#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "$0")/../.." && pwd)"
temp_dir="$(mktemp -d)"
trap 'rm -rf "$temp_dir"' EXIT

python3 - "$repo_root" "$temp_dir" <<'PY'
from pathlib import Path
import sys

root = Path(sys.argv[1])
target = Path(sys.argv[2])
source = (root / "src/ES-02/OllieFOCdrive/OllieFOCdrive.ino").read_text()
start = source.index("const SerialCommandEntry commandEntries[] = {")
end = source.index("\n};", start) + len("\n};")
(target / "command_table.inc").write_text(source[start:end])
PY

g++ -std=c++17 -Wall -Wextra -Werror -I"$temp_dir" \
  "$repo_root/tests/serial_commands/test_serial_command_router.cpp" -o "$temp_dir/test_serial_command_router"
"$temp_dir/test_serial_command_router"
