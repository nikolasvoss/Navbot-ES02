"""Tier A: checks that run unattended, with the transmitter off. Plan: docs/integration/software/agent-workflow.md, T9.

Against the fake firmware, except the two tests marked robot, which need a real robot's signal list and stream.
"""

import importlib.util
import queue
import time
from pathlib import Path

import pytest

from navbot_link import navbot_link_pb2 as pb
from navbot_link.capture import load_presets
from navbot_link.link import FramesLost, LogEvent, Row
from navbot_link.protocol import NoReply

ROOT = Path(__file__).resolve().parents[2]
RATE_TOLERANCE = 0.10
HEAP_MIN_FREE_B = 235_000  # about 80 % of the 294392 B seen idle on 2026-10-31
STACK_MIN_FREE_B = 2_400  # about 80 % of the 2992 B link task minimum seen idle on 2026-10-31
LINK_COUNTERS = ("link_tx_drop_uart", "link_tx_drop_usb", "link_rx_bad_uart", "link_rx_bad_usb",
                 "link_req_dropped", "link_undecodable")
STREAM_S = 10.0
COUNTER_QUIET_S = 5.0
LOG_QUIET_S = 5.0
ENDPOINT_TAGS = ("param", "action", "link")
REBOOT_WAIT_S = 40.0


def load_script(name):
    spec = importlib.util.spec_from_file_location(name, ROOT / "scripts" / f"{name}.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


signals_catalog = load_script("signals_catalog")


def drain(link):
    """Drop the events already queued, so a test sees only what arrives after this call."""
    while True:
        try:
            link.events.get_nowait()
        except queue.Empty:
            return


def collect(link, seconds):
    """Every event that arrives within the next `seconds`."""
    out = []
    deadline = time.monotonic() + seconds
    while (remaining := deadline - time.monotonic()) > 0:
        try:
            out.append(link.events.get(timeout=remaining))
        except queue.Empty:
            break
    return out


@pytest.mark.robot
def test_catalog_matches_signals_def(link):
    expected = [e.name for e in signals_catalog.parse(signals_catalog.DEFAULT_FILE.read_text(encoding="utf-8"))]
    assert [e.name for e in link.catalog().entries] == expected


@pytest.mark.robot
def test_stream_rate_and_no_lost_frames(link):
    trace = load_presets(ROOT / "scripts" / "navlink_presets.yaml")["trace"]
    drain(link)
    sub = link.subscribe(trace.signals, trace.rate_hz)
    try:
        events = collect(link, STREAM_S)
    finally:
        link.unsubscribe(sub.slot)
    rows = [e for e in events if isinstance(e, Row) and e.slot == sub.slot]
    expected = STREAM_S * trace.rate_hz
    assert abs(len(rows) - expected) <= expected * RATE_TOLERANCE
    assert [e for e in events if isinstance(e, FramesLost)] == []


def test_health_within_limits(link):
    assert link.get("heap_min_free_b") >= HEAP_MIN_FREE_B
    assert link.get("stack_free_loop_b") >= STACK_MIN_FREE_B
    assert link.get("stack_free_link_b") >= STACK_MIN_FREE_B
    first = link.get("uptime_s")
    time.sleep(2)
    assert link.get("uptime_s") > first


def test_link_counters_not_rising(link):
    before = [link.get(name) for name in LINK_COUNTERS]
    time.sleep(COUNTER_QUIET_S)
    assert [link.get(name) for name in LINK_COUNTERS] == before


def test_log_level_round_trip(link, fake):
    old = link.get("log_level")
    try:
        link.set("log_level", 3)
        drain(link)
        if fake is not None:
            fake.log("info line", "test")  # the fake drops it: below log_level 3
            fake.log("warn line", "test", pb.Log.WARN)
        events = collect(link, LOG_QUIET_S)
    finally:
        link.set("log_level", old)
    # The endpoint's own param, action and link lines go out at every level.
    assert [e for e in events if isinstance(e, LogEvent) and e.level == "INFO" and e.tag not in ENDPOINT_TAGS] == []
    if fake is not None:
        assert [e.text for e in events if isinstance(e, LogEvent)] == ["warn line"]


def test_param_set_restore(link):
    old = link.get("speed_cap")
    try:
        assert link.set("speed_cap", 0.25) == 0.25
        assert link.get("speed_cap") == 0.25
    finally:
        link.set("speed_cap", old)


def test_reboot(link):
    time.sleep(2)  # uptime_s has risen above zero, so a reboot must lower it
    before = link.get("uptime_s")
    link.call("reboot")
    # The firmware reboots kRebootDelayMs after the Ack, so the first reads may still see the old uptime.
    deadline = time.monotonic() + REBOOT_WAIT_S
    after = before
    while after >= before:
        if time.monotonic() > deadline:
            pytest.fail(f"uptime_s did not restart within {REBOOT_WAIT_S} s: {after}")
        time.sleep(0.5)
        try:
            after = link.get("uptime_s")
        except NoReply:  # includes RobotRestarted: the first reply after the reboot reveals it
            pass
    assert after < before
    assert link.get("reset_reason") == 3
