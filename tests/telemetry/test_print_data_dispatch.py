#!/usr/bin/env python3
"""Check the debug selector to log profile dispatch of the DebugTelemetry module."""

from pathlib import Path
import subprocess
import sys
import tempfile


ROOT = Path(__file__).resolve().parents[2]
SKETCH = ROOT / "src/ES-02/OllieFOCdrive"
TESTS = ROOT / "tests/telemetry"

REMOVED = [26, 27, 29, 30, 34, 35, 36, 38, 39, 40]
SELECTED = [selector for selector in range(1, 49) if selector not in REMOVED]
CHANNELS = list(range(60, 76))
SILENT = [0, 49, 54, 59, 76, 80] + REMOVED
TRACE_KIND = {55: "Trace", 56: "Control", 57: "Balance", 58: "Drive"}


def compile_driver(directory: Path) -> Path:
    executable = directory / "dispatch_driver"
    subprocess.run([
        "g++", "-std=c++17", "-Wall", "-Wextra", "-Werror",
        "-I", str(TESTS / "support"), "-I", str(SKETCH),
        str(SKETCH / "DebugTelemetry.cpp"),
        str(TESTS / "support/RecordingStubs.cpp"),
        str(TESTS / "dispatch_driver.cpp"),
        "-o", str(executable),
    ], check=True)
    return executable


def run_ticks(executable: Path, ticks):
    """Runs ticks (selection, gate, due, variant) against one fresh telemetry state."""
    script = "".join("%d %d %d %d\n" % tick for tick in ticks)
    result = subprocess.run([str(executable)], input=script, capture_output=True, text=True, check=True)
    return result.stdout.splitlines()


def expect(failures, label, actual, expected):
    if actual != expected:
        failures.append("%s: expected %r, got %r" % (label, expected, actual))


def main() -> int:
    failures = []
    with tempfile.TemporaryDirectory(prefix="print-data-dispatch-") as directory:
        executable = compile_driver(Path(directory))

        # Gate 1 keeps the trace profiles quiet.
        for selector in SELECTED + CHANNELS:
            (due, paced) = run_ticks(executable, [(selector, 1, 1, 0), (selector, 1, 0, 0)])
            expect(failures, "selector %d due" % selector, due, "%d 1 -> K%d" % (selector, selector))
            expect(failures, "selector %d paced out" % selector, paced, "%d 1 -> -" % selector)

        for selector in SILENT + list(TRACE_KIND):
            (off_gate,) = run_ticks(executable, [(selector, 1, 1, 0)])
            expect(failures, "selector %d off the trace gate" % selector, off_gate, "%d 1 -> -" % selector)
        for selector in SILENT:
            (on_gate,) = run_ticks(executable, [(selector, 0, 1, 0)])
            expect(failures, "selector %d on the trace gate" % selector, on_gate, "%d 0 -> -" % selector)

        for selector, kind in TRACE_KIND.items():
            first, quiet, second, third = run_ticks(
                executable, [(selector, 0, 1, 0), (selector, 6, 1, 0), (selector, 7, 1, 0), (selector, 14, 0, 0)])
            expect(failures, "selector %d first tick" % selector, first, "%d 0 -> %s#0" % (selector, kind))
            expect(failures, "selector %d tick between gates" % selector, quiet, "%d 6 -> -" % selector)
            expect(failures, "selector %d second row" % selector, second, "%d 7 -> %s#1" % (selector, kind))
            expect(failures, "selector %d row ignores selected-debug pacing" % selector, third,
                   "%d 14 -> %s#2" % (selector, kind))

    for failure in failures:
        print("FAIL " + failure, file=sys.stderr)
    if failures:
        return 1
    print("print_data dispatch: %d selected, %d channel, %d trace selectors ok"
          % (len(SELECTED), len(CHANNELS), len(TRACE_KIND)))
    return 0


if __name__ == "__main__":
    sys.exit(main())
