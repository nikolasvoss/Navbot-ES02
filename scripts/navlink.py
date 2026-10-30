#!/usr/bin/env python3
"""navlink: shell and CSV capture for the robot's signals, parameters and actions.

Talks the NavBot link protocol; docs: docs/integration/software/navlink.md
"""

import argparse
import cmd
import fnmatch
import json
import math
import queue
import sys
import threading
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from navbot_link.capture import CsvWriter, describe, load_presets, rate_text  # noqa: E402
from navbot_link.framing import Text  # noqa: E402
from navbot_link.link import (  # noqa: E402
    ROW_FRAME_OVERHEAD,
    CommandRefused,
    Disconnected,
    FramesLost,
    Link,
    LogEvent,
    Restarted,
    Row,
    SubscriptionEnded,
    check_args,
)
from navbot_link.protocol import (  # noqa: E402
    Kind,
    LinkError,
    NoReply,
    Type,
    UnknownName,
    UsageError,
    format_value,
    parse_value,
)

DEFAULT_PRESETS = ROOT / "scripts" / "navlink_presets.yaml"
DEFAULT_PORT = "tcp:127.0.0.1:7650"  # the link multiplexer
USB_BAUD = 576_000
UART_BAUD = 2_000_000
STALL_S = 2.0
WAIT_STEP_S = 0.25
# Exit codes of the one-shot commands.
EXIT_DONE, EXIT_REFUSED, EXIT_USAGE, EXIT_NO_CONNECTION = 0, 1, 2, 3
LEVEL_RANK = {"DEBUG": 1, "INFO": 2, "WARN": 3, "ERROR": 4, "FATAL": 5}


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
            raise UsageError(f"unknown preset {item[1:]} (have: {', '.join(sorted(presets))})")
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
                    raise NoReply(f"no data for {max(STALL_S, 3 * period_s):g} s") from None
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
        self._writers_lock = threading.Lock()  # the event thread writes, commands close
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
        for e in matching(self.link.catalog(), arg.strip() or "*"):
            self.say(entry_line(e))

    def do_get(self, arg):
        """get <name>...: current values."""
        catalog = self.link.catalog()
        for name in arg.split():
            entry = catalog.signal(name)
            self.say(f"{name} = {format_value(entry.type, self.link.get(name))}")

    def do_set(self, arg):
        """set <param> <value>"""
        name, text = _two_words(arg, "set <param> <value>")
        entry, old, new = set_param(self.link, name, text)
        self.say(f"{name}: {format_value(entry.type, old)} -> {format_value(entry.type, new)}")

    def do_call(self, arg):
        """call <action> [arg=value]...: asks y/N before it sends."""
        words = arg.split()
        if not words:
            raise UsageError("call <action> [arg=value]...")
        entry = self.link.catalog().action(words[0])
        values = action_values(entry, words[1:])
        shown = ", ".join(f"{a.name}={v:g} [{a.min:g}, {a.max:g}]" for a, v in zip(entry.args, values))
        with self._out_lock:
            self.stdout.write(f"{entry.name}({shown})? [y/N] ")
            self.stdout.flush()
        if self.stdin.readline().strip().lower() != "y":
            self.say("not sent")
            return
        self.link.call(entry.name, **dict(zip((a.name for a in entry.args), values)))
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
        catalog = self.link.catalog()
        entries = tuple(catalog.signal(n) for n in names)
        writer = None
        if output:
            f = open(output, "w", newline="", encoding="utf-8")
            writer = CsvWriter(f, entries, catalog.build, rate, read_params(self.link))
        # Held until the writer is registered, so the first rows cannot miss the file.
        with self._writers_lock:
            try:
                sub = self.link.subscribe(names, rate)
            except LinkError:
                if writer is not None:
                    writer.close()
                raise
            if writer is not None:
                self._writers[sub.slot] = writer
        # The firmware budgets a row at its values plus the frame cost, so the figure is the port's real load.
        load = f", {(ROW_FRAME_OVERHEAD + 4 + 4 * len(names)) * rate / 1000:.1f} kB/s" if rate else ""
        self.say(f"slot {sub.slot}: {len(names)} entries @ {rate_text(rate)}{load}"
                 f"{' -> ' + output if output else ''}")

    def do_unsub(self, arg):
        """unsub <slot>"""
        slot = int(arg)
        try:
            self.link.unsubscribe(slot)
        finally:
            self._close_writer(slot)
        self.say(f"slot {slot} closed")

    def do_log(self, arg):
        """log on|off: show log events and raw text."""
        if arg.strip() not in ("on", "off"):
            raise ValueError("log on|off")
        self.show_logs = arg.strip() == "on"

    def do_quit(self, arg):
        """quit: stops every subscription and closes all files."""
        for slot in self.link.slots:
            try:
                self.link.unsubscribe(slot)
            except LinkError:
                pass  # the subscription expires with its lease
        for slot in list(self._writers):
            self._close_writer(slot)
        return True

    do_EOF = do_quit

    def _close_writer(self, slot: int) -> None:
        with self._writers_lock:
            writer = self._writers.pop(slot, None)
            if writer is not None:
                writer.close()

    def _drain_events(self) -> None:
        while True:
            event = self.link.events.get()
            if isinstance(event, Row):
                with self._writers_lock:
                    writer = self._writers.get(event.slot)
                    if writer is not None:
                        writer.row(event)
                if writer is None:
                    self.say(f"slot {event.slot} t_us={event.t_us} "
                             + " ".join(str(v) for v in event.values))
            elif isinstance(event, (LogEvent, FramesLost)):
                with self._writers_lock:
                    for writer in self._writers.values():
                        writer.event(event)
                if self.show_logs or isinstance(event, FramesLost):
                    self.say(describe(event))
            elif isinstance(event, Text):
                if self.show_logs:
                    self.say(event.text.rstrip("\r\n"))
            elif isinstance(event, SubscriptionEnded):
                with self._writers_lock:
                    writer = self._writers.get(event.slot)
                    if writer is not None:
                        writer.event(event)
                self._close_writer(event.slot)
                self.say(describe(event))
            elif isinstance(event, CommandRefused):
                self.say(f"command refused: {event.reason}")
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


def matching(catalog, pattern: str) -> list:
    return [e for e in catalog.entries if fnmatch.fnmatch(e.name, pattern)]


def entry_line(e) -> str:
    detail = ", ".join(f"{a.name} [{a.min:g}, {a.max:g}]" for a in e.args)
    kind = e.kind.name.lower()
    type_ = e.type.name.lower() if e.type is not None else ""
    live = ""
    if e.kind == Kind.PARAM and e.live_step > 0:
        live = " live" if math.isinf(e.live_step) else f" live ±{e.live_step:g}"
    return f"{e.name:<24} {kind:<10} {type_:<4} {e.unit or detail}{live}".rstrip()


def json_number(value: float):
    """JSON has no infinity: it goes out as the string the text output uses."""
    return value if math.isfinite(value) else str(value)


def json_value(type_: Type, value):
    if type_ == Type.BOOL:
        return bool(value)
    if type_ == Type.F32:
        return json_number(float(format_value(type_, value)))
    return value


def entry_record(e) -> dict:
    record = {"name": e.name, "kind": e.kind.name}
    if e.type is not None:
        record["type"] = e.type.name
    record["unit"] = e.unit
    if e.kind == Kind.PARAM:
        record["live_step"] = json_value(Type.F32, e.live_step)
    if e.kind == Kind.ACTION:
        record["args"] = [{"name": a.name, "min": json_number(a.min), "max": json_number(a.max)} for a in e.args]
    return record


def set_param(link: Link, name: str, text: str):
    """The parameter's entry, the value before and the value now in effect."""
    entry = link.catalog().param(name)
    old = link.get(name)
    new = link.set(name, parse_value(entry.type, text))
    return entry, old, new


def action_values(entry, words: list[str]) -> list[float]:
    """Argument values in catalog order from `name=value` words."""
    kwargs = {}
    for word in words:
        name, sep, text = word.partition("=")
        if not sep:
            raise UsageError(f"{word}: an argument is written name=value")
        kwargs[name] = text
    return check_args(entry, kwargs)


def open_ready(open_link, port: str, baud: int, wait_s: float) -> Link:
    """Opens the link and reads the catalog. With wait_s, retries until the robot answers or the time is up."""
    deadline = time.monotonic() + wait_s
    while True:
        link = None
        try:
            link = open_link(port, baud)
            link.catalog()
            return link
        except (LinkError, OSError):
            if link is not None:
                link.close()
            if time.monotonic() >= deadline:
                raise
            time.sleep(WAIT_STEP_S)


def cmd_info(link: Link, args, out) -> int:
    catalog = link.catalog()
    if args.json:
        print(json.dumps({"build": catalog.build, "entries": len(catalog.entries), "port": args.port}), file=out)
    else:
        print(f"build {catalog.build}\nentries {len(catalog.entries)}\nport {args.port}", file=out)
    return EXIT_DONE


def cmd_list(link: Link, args, out) -> int:
    entries = matching(link.catalog(), args.pattern)
    if args.json:
        print(json.dumps([entry_record(e) for e in entries]), file=out)
    else:
        for e in entries:
            print(entry_line(e), file=out)
    return EXIT_DONE


def cmd_get(link: Link, args, out) -> int:
    catalog = link.catalog()
    values = {}
    for name in args.names:
        entry = catalog.signal(name)
        values[name] = (entry, link.get(name))
    if args.json:
        print(json.dumps({name: json_value(e.type, v) for name, (e, v) in values.items()}), file=out)
    else:
        for name, (e, v) in values.items():
            print(f"{name} = {format_value(e.type, v)}", file=out)
    return EXIT_DONE


def cmd_set(link: Link, args, out) -> int:
    entry, old, new = set_param(link, args.name, args.value)
    if args.json:
        print(json.dumps({"name": args.name, "old": json_value(entry.type, old),
                          "new": json_value(entry.type, new)}), file=out)
    else:
        print(f"{args.name}: {format_value(entry.type, old)} -> {format_value(entry.type, new)}", file=out)
    return EXIT_DONE


def cmd_call(link: Link, args, out) -> int:
    entry = link.catalog().action(args.action)
    named = dict(zip((a.name for a in entry.args), action_values(entry, args.args)))
    link.call(entry.name, **named)
    if args.json:
        print(json.dumps({"name": entry.name, "args": named, "accepted": True}), file=out)
    else:
        print(f"{entry.name}: started", file=out)
    return EXIT_DONE


def cmd_log(link: Link, args, out) -> int:
    floor = LEVEL_RANK[args.min_level]
    deadline = time.monotonic() + args.seconds
    while (remaining := deadline - time.monotonic()) > 0:
        try:
            event = link.events.get(timeout=remaining)
        except queue.Empty:
            break
        if isinstance(event, Disconnected):
            raise LinkError(f"port closed: {event.reason}")
        if isinstance(event, LogEvent) and LEVEL_RANK.get(event.level, 5) >= floor:
            if args.json:
                print(json.dumps({"t_ms": event.t_ms, "level": event.level, "tag": event.tag,
                                  "text": event.text}), file=out)
            else:
                print(describe(event), file=out)
    return EXIT_DONE


ONE_SHOT = {"info": cmd_info, "list": cmd_list, "get": cmd_get, "set": cmd_set, "call": cmd_call, "log": cmd_log}


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
    common.add_argument("--port", default=DEFAULT_PORT,
                        help=f"the link multiplexer (default {DEFAULT_PORT}), or a serial port such as /dev/ttyUSB0")
    common.add_argument("--baud", type=int, help=f"default {UART_BAUD}, or {USB_BAUD} for a USB port")
    common.add_argument("--presets", type=Path, default=DEFAULT_PRESETS, help="preset YAML file")
    common.add_argument("--wait", type=float, default=0, metavar="SECONDS",
                        help="retry opening the link and reading the catalog for this long (after a reboot or a flash)")
    as_json = argparse.ArgumentParser(add_help=False)
    as_json.add_argument("--json", action="store_true", help="JSON on stdout instead of text")
    sub = parser.add_subparsers(dest="command", required=True)
    sub.add_parser("shell", parents=[common], help="interactive shell")
    cap = sub.add_parser("capture", parents=[common], help="stream signals to a CSV file")
    what = cap.add_mutually_exclusive_group(required=True)
    what.add_argument("--preset", help="preset name from the preset file")
    what.add_argument("--signals", help="comma-separated signal names")
    cap.add_argument("--rate", type=float, help="Hz; 0 = every control tick (default: the preset's rate)")
    cap.add_argument("--seconds", type=float, required=True, help="robot time to capture")
    cap.add_argument("-o", "--output", required=True, help="CSV file")
    sub.add_parser("info", parents=[common, as_json], help="build ID, entry count and port")
    lst = sub.add_parser("list", parents=[common, as_json], help="entries whose name matches a glob pattern")
    lst.add_argument("pattern", nargs="?", default="*")
    get = sub.add_parser("get", parents=[common, as_json], help="current values of signals and parameters")
    get.add_argument("names", nargs="+")
    set_cmd = sub.add_parser("set", parents=[common, as_json], help="set a parameter")
    set_cmd.add_argument("name")
    set_cmd.add_argument("value")
    call = sub.add_parser("call", parents=[common, as_json], help="start an action")
    call.add_argument("action")
    call.add_argument("args", nargs="*", metavar="arg=value")
    call.add_argument("--yes", action="store_true", help="required: starts the action on the robot")
    log = sub.add_parser("log", parents=[common, as_json], help="print log events for a few seconds")
    log.add_argument("--seconds", type=float, required=True, help="PC time to listen")
    log.add_argument("--min-level", choices=["DEBUG", "INFO", "WARN", "ERROR"], default="DEBUG")
    return parser.parse_args(argv)


def _fail(code: int, exc) -> int:
    print(f"navlink: error: {exc}", file=sys.stderr)
    return code


def main(argv=None, *, open_link=None, stdin=None, stdout=None) -> int:
    args = parse_args(argv)
    stdin, stdout = stdin or sys.stdin, stdout or sys.stdout
    open_link = open_link or (lambda port, baud: Link(port, baud))
    if args.command == "call" and not args.yes:
        return _fail(EXIT_USAGE, "call needs --yes")
    link = None
    try:
        link = open_ready(open_link, args.port, args.baud or default_baud(args.port), args.wait)
        if args.command == "capture":
            return capture(link, args, stdout)
        if args.command == "shell":
            Shell(link, args.presets, stdin, stdout).cmdloop()
            return EXIT_DONE
        return ONE_SHOT[args.command](link, args, stdout)
    except (UnknownName, UsageError, ValueError) as exc:
        return _fail(EXIT_USAGE, exc)
    except (NoReply, OSError) as exc:
        return _fail(EXIT_NO_CONNECTION, exc)
    except LinkError as exc:
        dead = link is not None and link.dead_reason is not None
        return _fail(EXIT_NO_CONNECTION if dead else EXIT_REFUSED, exc)
    finally:
        if link is not None:
            link.close()


if __name__ == "__main__":
    sys.exit(main())
