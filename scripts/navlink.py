#!/usr/bin/env python3
"""navlink: shell and CSV capture for the robot's signals, parameters and actions.

Talks the NavBot link protocol; docs: docs/integration/software/navlink.md
"""

import argparse
import cmd
import fnmatch
import queue
import sys
import threading
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from navbot_link.capture import CsvWriter, describe, load_presets, rate_text  # noqa: E402
from navbot_link.framing import Text  # noqa: E402
from navbot_link.link import (  # noqa: E402
    Disconnected,
    FramesLost,
    Link,
    LogEvent,
    Restarted,
    Row,
    SubscriptionEnded,
    check_args,
)
from navbot_link.protocol import LinkError, format_value, parse_value  # noqa: E402

DEFAULT_PRESETS = ROOT / "scripts" / "navlink_presets.yaml"
USB_BAUD = 576_000
UART_BAUD = 2_000_000
STALL_S = 2.0


def default_baud(port: str) -> int:
    return USB_BAUD if "USB" in port or "ACM" in port else UART_BAUD


def resolve_signals(spec: list[str], presets_path: Path) -> tuple[list[str], float | None]:
    """Names and `@preset` references; returns the names and the preset rate, if any."""
    names, rate = [], None
    for item in spec:
        if not item.startswith("@"):
            names.append(item)
            continue
        presets = load_presets(presets_path)
        preset = presets.get(item[1:])
        if preset is None:
            raise LinkError(f"unknown preset {item[1:]} (have: {', '.join(sorted(presets))})")
        names += preset.signals
        rate = preset.rate_hz
    return names, rate


def read_params(link: Link) -> list:
    return [(e, link.get(e.name)) for e in link.catalog().params]


def capture(link: Link, args, out) -> int:
    spec = [f"@{args.preset}"] if args.preset else args.signals.split(",")
    names, preset_rate = resolve_signals(spec, args.presets)
    rate = args.rate if args.rate is not None else (preset_rate or 0)
    catalog = link.catalog()
    for name in names:
        catalog.signal(name)  # fail on an unknown name before anything streams
    params = read_params(link)
    sub = link.subscribe(names, rate)
    period_s = sub.period_us / 1e6
    rows, t0 = 0, None
    try:
        with open(args.output, "w", newline="", encoding="utf-8") as f:
            writer = CsvWriter(f, sub.entries, catalog.build, rate, params)
            while True:
                try:
                    event = link.events.get(timeout=max(STALL_S, 3 * period_s))
                except queue.Empty:
                    raise LinkError(f"no data for {max(STALL_S, 3 * period_s):g} s") from None
                if isinstance(event, Row) and event.slot == sub.slot:
                    t0 = event.t_us if t0 is None else t0
                    # Robot time, not PC time: the file holds exactly --seconds of data.
                    if (event.t_us - t0) & 0xFFFFFFFF >= args.seconds * 1e6:
                        break
                    writer.row(event)
                    rows += 1
                elif isinstance(event, (LogEvent, FramesLost)):
                    writer.event(event)
                    print(describe(event), file=sys.stderr)
                elif isinstance(event, Text):
                    print(event.text.rstrip("\r\n"), file=sys.stderr)
                elif isinstance(event, SubscriptionEnded) and event.slot == sub.slot:
                    writer.event(event)
                    raise LinkError(f"capture stopped: {event.reason}")
                elif isinstance(event, Disconnected):
                    raise LinkError(f"port closed: {event.reason}")
    finally:
        try:
            link.unsubscribe(sub.slot)
        except LinkError:
            pass  # the subscription expires with its lease
    print(f"{rows} rows, {len(names)} signals @ {rate_text(rate)} -> {args.output}", file=out)
    return 0


class Shell(cmd.Cmd):
    prompt = "navlink> "

    def __init__(self, link: Link, presets: Path, stdin, stdout):
        super().__init__(stdin=stdin, stdout=stdout)
        self.use_rawinput = stdin is sys.stdin and sys.stdin.isatty()
        if not self.use_rawinput:
            self.prompt = ""
        self.link = link
        self.presets = presets
        self.show_logs = True
        self._writers: dict[int, CsvWriter] = {}
        self._out_lock = threading.Lock()
        self._catalog = link.catalog()
        self._pump = threading.Thread(target=self._drain_events, daemon=True)
        self._pump.start()

    def say(self, text: str) -> None:
        with self._out_lock:
            self.stdout.write(text + "\n")
            self.stdout.flush()

    def onecmd(self, line):
        try:
            return super().onecmd(line)
        except (LinkError, ValueError, OSError) as exc:
            self.say(f"error: {exc}")
            return False

    def emptyline(self):
        return False

    def default(self, line):
        self.say(f"unknown command: {line.split()[0]} (try help)")

    def do_list(self, arg):
        """list [pattern]: entries whose name matches the glob pattern."""
        for e in self.link.catalog().entries:
            if fnmatch.fnmatch(e.name, arg.strip() or "*"):
                detail = ", ".join(f"{a.name} [{a.min:g}, {a.max:g}]" for a in e.args)
                kind = e.kind.name.lower()
                type_ = e.type.name.lower() if e.type is not None else ""
                self.say(f"{e.name:<24} {kind:<10} {type_:<4} {e.unit or detail}".rstrip())

    def do_get(self, arg):
        """get <name>...: current values."""
        catalog = self.link.catalog()
        for name in arg.split():
            entry = catalog.signal(name)
            self.say(f"{name} = {format_value(entry.type, self.link.get(name))}")

    def do_set(self, arg):
        """set <param> <value>"""
        name, text = _two_words(arg, "set <param> <value>")
        entry = self.link.catalog().param(name)
        old = self.link.get(name)
        new = self.link.set(name, parse_value(entry.type, text))
        self.say(f"{name}: {format_value(entry.type, old)} -> {format_value(entry.type, new)}")

    def do_call(self, arg):
        """call <action> [arg=value]...: asks y/N before it sends."""
        words = arg.split()
        if not words:
            raise ValueError("call <action> [arg=value]...")
        entry = self.link.catalog().action(words[0])
        kwargs = dict(w.split("=", 1) for w in words[1:])
        values = check_args(entry, kwargs)
        shown = ", ".join(f"{a.name}={v:g}" for a, v in zip(entry.args, values))
        with self._out_lock:
            self.stdout.write(f"{entry.name}({shown})? [y/N] ")
            self.stdout.flush()
        if self.stdin.readline().strip().lower() != "y":
            self.say("not sent")
            return
        self.link.call(entry.name, **{a.name: v for a, v in zip(entry.args, values)})
        self.say(f"{entry.name}: started")

    def do_sub(self, arg):
        """sub <names|@preset>... [@<Hz>] [-o file.csv]: stream signals."""
        words, output, rate = arg.split(), None, None
        if "-o" in words:
            i = words.index("-o")
            output = words[i + 1] if i + 1 < len(words) else None
            if output is None:
                raise ValueError("-o needs a file name")
            del words[i:i + 2]
        spec = []
        for w in words:
            if w.startswith("@") and _is_number(w[1:]):
                rate = float(w[1:])
            else:
                spec.append(w)
        names, preset_rate = resolve_signals(spec, self.presets)
        rate = rate if rate is not None else (preset_rate or 0)
        params = read_params(self.link) if output else []
        sub = self.link.subscribe(names, rate)
        if output:
            f = open(output, "w", newline="", encoding="utf-8")
            self._writers[sub.slot] = CsvWriter(f, sub.entries, self.link.catalog().build, rate, params)
        load = f", {(4 + 4 * len(names)) * rate / 1000:.1f} kB/s" if rate else ""
        self.say(f"slot {sub.slot}: {len(names)} entries @ {rate_text(rate)}{load}"
                 f"{' -> ' + output if output else ''}")

    def do_unsub(self, arg):
        """unsub <slot>"""
        slot = int(arg)
        self.link.unsubscribe(slot)
        self._close_writer(slot)
        self.say(f"slot {slot} closed")

    def do_log(self, arg):
        """log on|off: show log events and raw text."""
        if arg.strip() not in ("on", "off"):
            raise ValueError("log on|off")
        self.show_logs = arg.strip() == "on"

    def do_quit(self, arg):
        """quit"""
        for slot in list(self._writers):
            self._close_writer(slot)
        return True

    do_EOF = do_quit

    def _close_writer(self, slot: int) -> None:
        writer = self._writers.pop(slot, None)
        if writer is not None:
            writer.close()

    def _drain_events(self) -> None:
        while True:
            event = self.link.events.get()
            if isinstance(event, Row):
                writer = self._writers.get(event.slot)
                if writer is not None:
                    writer.row(event)
                else:
                    self.say(f"slot {event.slot} t_us={event.t_us} "
                             + " ".join(str(v) for v in event.values))
            elif isinstance(event, (LogEvent, FramesLost)):
                for writer in list(self._writers.values()):
                    writer.event(event)
                if self.show_logs or isinstance(event, FramesLost):
                    self.say(describe(event))
            elif isinstance(event, Text):
                if self.show_logs:
                    self.say(event.text.rstrip("\r\n"))
            elif isinstance(event, SubscriptionEnded):
                writer = self._writers.get(event.slot)
                if writer is not None:
                    writer.event(event)
                self._close_writer(event.slot)
                self.say(describe(event))
            elif isinstance(event, Restarted):
                self._report_restart()
            elif isinstance(event, Disconnected):
                self.say(f"port closed: {event.reason}")
                return

    def _report_restart(self) -> None:
        old = {e.name for e in self._catalog.entries}
        try:
            self._catalog = self.link.catalog()
        except LinkError as exc:
            self.say(f"robot restarted; catalog not read: {exc}")
            return
        new = {e.name for e in self._catalog.entries}
        added, removed = sorted(new - old), sorted(old - new)
        self.say(f"robot restarted: build {self._catalog.build}"
                 + (f"; added {', '.join(added)}" if added else "")
                 + (f"; removed {', '.join(removed)}" if removed else ""))


def _two_words(arg: str, usage: str) -> tuple[str, str]:
    words = arg.split()
    if len(words) != 2:
        raise ValueError(usage)
    return words[0], words[1]


def _is_number(text: str) -> bool:
    try:
        float(text)
    except ValueError:
        return False
    return True


def parse_args(argv):
    parser = argparse.ArgumentParser(prog="navlink", description=__doc__.splitlines()[0])
    common = argparse.ArgumentParser(add_help=False)
    common.add_argument("--port", required=True, help="/dev/ttyAMA3 (stop the bridge first) or /dev/ttyUSB0")
    common.add_argument("--baud", type=int, help=f"default {UART_BAUD}, or {USB_BAUD} for a USB port")
    common.add_argument("--presets", type=Path, default=DEFAULT_PRESETS, help="preset YAML file")
    sub = parser.add_subparsers(dest="command", required=True)
    sub.add_parser("shell", parents=[common], help="interactive shell")
    cap = sub.add_parser("capture", parents=[common], help="stream signals to a CSV file")
    what = cap.add_mutually_exclusive_group(required=True)
    what.add_argument("--preset", help="preset name from the preset file")
    what.add_argument("--signals", help="comma-separated signal names")
    cap.add_argument("--rate", type=float, help="Hz; 0 = every control tick (default: the preset's rate)")
    cap.add_argument("--seconds", type=float, required=True, help="robot time to capture")
    cap.add_argument("-o", "--output", required=True, help="CSV file")
    return parser.parse_args(argv)


def main(argv=None, *, open_link=None, stdin=None, stdout=None) -> int:
    args = parse_args(argv)
    stdin, stdout = stdin or sys.stdin, stdout or sys.stdout
    open_link = open_link or (lambda port, baud: Link(port, baud))
    try:
        with open_link(args.port, args.baud or default_baud(args.port)) as link:
            if args.command == "capture":
                return capture(link, args, stdout)
            Shell(link, args.presets, stdin, stdout).cmdloop()
            return 0
    except (LinkError, ValueError, OSError) as exc:
        print(f"navlink: error: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
