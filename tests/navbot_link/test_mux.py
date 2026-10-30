"""The multiplexer over a pseudo-terminal: Link clients over TCP share one fake robot."""

import queue
import socket
import threading
import time
from contextlib import contextmanager

import pytest

from fake_firmware import FakeFirmware, PtyFirmware
from navbot_link.link import FramesLost, Link, LogEvent, Restarted, Row
from navbot_link.mux import Multiplexer
from navbot_link.protocol import LinkError


@contextmanager
def running_mux(firmware):
    robot = PtyFirmware(firmware)
    listener = socket.create_server(("127.0.0.1", 0))
    url = f"tcp:127.0.0.1:{listener.getsockname()[1]}"
    mux = Multiplexer(robot.path, 2_000_000, listener)
    thread = threading.Thread(target=mux.serve, daemon=True)
    thread.start()
    try:
        yield url
    finally:
        mux.close()
        thread.join(timeout=5)
        robot.close()


def wait_for(events, kind, timeout_s=3.0):
    """Returns the first event of `kind` and every event seen before it."""
    seen, deadline = [], time.monotonic() + timeout_s
    while True:
        event = events.get(timeout=max(0.0, deadline - time.monotonic()))
        seen.append(event)
        if isinstance(event, kind):
            return event, seen


def rows_of(events, count, timeout_s=3.0):
    rows, deadline = [], time.monotonic() + timeout_s
    while len(rows) < count:
        event = events.get(timeout=max(0.0, deadline - time.monotonic()))
        if isinstance(event, Row):
            rows.append(event)
    return rows


def drain(events):
    out = []
    while True:
        try:
            out.append(events.get_nowait())
        except queue.Empty:
            return out


def test_two_clients_share_the_robot_and_each_gets_only_its_own_rows_and_replies():
    fw = FakeFirmware()
    with running_mux(fw) as url:
        a = Link(url, timeout_s=2.0)
        b = Link(url, timeout_s=2.0)
        try:
            assert a.catalog().build == "fake-build-1"
            assert b.catalog().signal("speed_measured").index == 1

            sub_a = a.subscribe(["speed_target"], rate_hz=50)
            sub_b = b.subscribe(["loop_count"], rate_hz=50)
            assert sub_a.slot != sub_b.slot
            assert all(row.slot == sub_a.slot and row.values == (0.5,) for row in rows_of(a.events, 3))
            assert all(row.slot == sub_b.slot for row in rows_of(b.events, 3))

            assert a.get("speed_kp") == pytest.approx(0.08)
            assert b.set("speed_kp", 0.1) == pytest.approx(0.1)
            assert a.get("speed_kp") == pytest.approx(0.1)

            # Both clients see the Log of the applied set; the reply went to b alone.
            for events in (a.events, b.events):
                log, _ = wait_for(events, LogEvent)
                assert (log.tag, log.text) == ("param", "speed_kp: 0.08 -> 0.1")

            # b does not own a's slot, so it cannot unsubscribe it; a keeps streaming.
            with pytest.raises(LinkError, match="^unsubscribe refused: slot in use$"):
                b.unsubscribe(sub_a.slot)
            assert all(row.slot == sub_a.slot for row in rows_of(a.events, 2))

            time.sleep(0.2)
            assert not [e for e in drain(a.events) + drain(b.events) if isinstance(e, FramesLost)]
        finally:
            a.close()
            b.close()


def test_closing_a_client_frees_its_slot_on_the_robot():
    fw = FakeFirmware()
    with running_mux(fw) as url:
        client = Link(url, timeout_s=2.0)
        sub = client.subscribe(["speed_target"], rate_hz=50)
        assert fw.subscribed_slots() == [sub.slot]
        client.close()
        deadline = time.monotonic() + 3.0
        while fw.subscribed_slots() and time.monotonic() < deadline:
            time.sleep(0.01)
        assert fw.subscribed_slots() == []


def test_a_robot_restart_tells_every_client_and_loses_no_frames():
    fw = FakeFirmware()
    with running_mux(fw) as url:
        a = Link(url, timeout_s=2.0)
        b = Link(url, timeout_s=2.0)
        try:
            a.catalog()
            b.catalog()
            fw.restart()
            fw.log("after the restart", "test")
            for events in (a.events, b.events):
                _, seen = wait_for(events, Restarted)
                assert not [e for e in seen if isinstance(e, FramesLost)]
        finally:
            a.close()
            b.close()


def test_a_gap_on_the_robot_is_a_gap_for_every_client():
    fw = FakeFirmware()
    with running_mux(fw) as url:
        a = Link(url, timeout_s=2.0)
        b = Link(url, timeout_s=2.0)
        try:
            a.catalog()
            b.catalog()
            fw.drop_frames(2)
            fw.log("after the gap", "test")
            for events in (a.events, b.events):
                lost, _ = wait_for(events, FramesLost)
                assert lost.count == 2
        finally:
            a.close()
            b.close()
