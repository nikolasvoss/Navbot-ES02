#!/usr/bin/env python3
"""List the entries of the firmware's signals.def.h: name, unit and kind, in catalog order.

Kinds: SIGNAL, SIGNAL_MIN, SIGNAL_MAX, PARAM and ACTION. An action's argument bounds are checked for shape here and
printed by navlink on the PC; the firmware owns their values.

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

# SIGNAL(name, "unit", expression), also SIGNAL_MIN and SIGNAL_MAX.
ENTRY = re.compile(r'^(SIGNAL_MIN|SIGNAL_MAX|SIGNAL)\s*\(\s*(\w+)\s*,\s*"([^"]*)"\s*,\s*(\S.*)\)$')
# PARAM(name, "unit", group, field)
PARAM = re.compile(r'^PARAM\s*\(\s*(\w+)\s*,\s*"([^"]*)"\s*,\s*(\w+)\s*,\s*(\w+)\s*\)$')
# ACTION(name, handler, ARG(name, min, max), ...) and ACTION_NO_ARGS(name, handler)
ACTION = re.compile(r'^ACTION\s*\(\s*(\w+)\s*,\s*(\w+)\s*,\s*(ARG\(.*\))\s*\)$')
ACTION_NO_ARGS = re.compile(r'^ACTION_NO_ARGS\s*\(\s*(\w+)\s*,\s*(\w+)\s*\)$')
ARG = re.compile(r'ARG\(\s*(\w+)\s*,\s*([^,()]+?)\s*,\s*([^,()]+?)\s*\)')


class DefError(Exception):
    pass


@dataclass(frozen=True)
class DefEntry:
    index: int
    name: str
    unit: str
    kind: str  # SIGNAL, SIGNAL_MIN, SIGNAL_MAX, PARAM or ACTION


def _check_args(number: int, text: str) -> None:
    """Every ARG(name, min, max) of an ACTION line must have the shape the firmware macro takes."""
    rest = ARG.sub("", text).replace(",", "").strip()
    if rest:
        raise DefError(f"line {number}: not an ARG(name, min, max) list: {text}")
    if not ARG.search(text):
        raise DefError(f"line {number}: an ACTION needs at least one ARG; use ACTION_NO_ARGS")


def parse(text: str) -> list[DefEntry]:
    entries: list[DefEntry] = []
    for number, raw in enumerate(text.splitlines(), 1):
        line = raw.split("//", 1)[0].strip()
        if not line:
            continue
        if match := ENTRY.match(line):
            kind, name, unit = match[1], match[2], match[3]
        elif match := PARAM.match(line):
            name, unit, kind = match[1], match[2], "PARAM"
        elif match := ACTION.match(line):
            name, unit, kind = match[1], "", "ACTION"
            _check_args(number, match[3])
        elif match := ACTION_NO_ARGS.match(line):
            name, unit, kind = match[1], "", "ACTION"
        else:
            raise DefError(f"line {number}: not an entry this tool knows: {raw.strip()}")
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
