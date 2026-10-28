#!/usr/bin/env python3
"""List the entries of the firmware's signals.def.h: name, unit and kind, in catalog order.

The catalog index of an entry is its position in the file. The tests use this list as the expected catalog, so a
preset that names a signal the firmware no longer has fails on the PC, without a robot.
"""

import argparse
import re
import sys
from dataclasses import dataclass
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_FILE = ROOT / "src" / "ES-02" / "OllieFOCdrive" / "signals.def.h"

# SIGNAL(name, "unit", expression), also SIGNAL_MIN and SIGNAL_MAX. PARAM and ACTION lines have other arguments and
# need their own pattern when the firmware gets them.
ENTRY = re.compile(r'^(SIGNAL_MIN|SIGNAL_MAX|SIGNAL)\s*\(\s*(\w+)\s*,\s*"([^"]*)"\s*,\s*(\S.*)\)$')


class DefError(Exception):
    pass


@dataclass(frozen=True)
class DefEntry:
    index: int
    name: str
    unit: str
    kind: str  # SIGNAL, SIGNAL_MIN or SIGNAL_MAX


def parse(text: str) -> list[DefEntry]:
    entries: list[DefEntry] = []
    for number, raw in enumerate(text.splitlines(), 1):
        line = raw.split("//", 1)[0].strip()
        if not line:
            continue
        match = ENTRY.match(line)
        if not match:
            raise DefError(f"line {number}: not an entry this tool knows: {raw.strip()}")
        kind, name, unit, _expression = match.groups()
        if any(e.name == name for e in entries):
            raise DefError(f"line {number}: duplicate name {name}")
        entries.append(DefEntry(len(entries), name, unit, kind))
    return entries


def main(argv: list[str]) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("file", nargs="?", type=Path, default=DEFAULT_FILE, help="entry file (default: the firmware's)")
    args = parser.parse_args(argv)
    try:
        entries = parse(args.file.read_text(encoding="utf-8"))
    except DefError as error:
        print(f"{args.file}: {error}", file=sys.stderr)
        return 1
    for e in entries:
        print(f"{e.index}\t{e.name}\t{e.unit}\t{e.kind}")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
