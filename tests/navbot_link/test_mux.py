"""The multiplexer over a pseudo-terminal: Link clients over TCP share one fake robot."""

import queue
import socket
import threading
import time
from contextlib import contextmanager

import pytest

from fake_firmware import FakeFirmware, PtyFirmware
from navbot_link import mux as mux_module
from navbot_link import navbot_link_pb2 as pb
from navbot_link.framing import FrameDecoder, encode_frame
from navbot_link.link import FramesLost, Link, LogEvent, Restarted, Row
from navbot_link.mux import Multiplexer
from navbot_link.protocol import LinkError, SeqTracker


@contextmanager
def running_mux(firmware):
    robot = PtyFirmware(firmware)
    listener = socket.create_server(("127.0.0.1", 0))
    url = f"tcp:127.0.0.1:{listener.getsockname()[1]}"
    mux = Multiplexer(robot.path, 2_000_000, listener)
    thread = threading.Thread(target=mux.serve, daemon=True)
    thread.start()
    try:
        yield url, mux
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
    with running_mux(fw) as (url, _):
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
    with running_mux(fw) as (url, _):
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
    with running_mux(fw) as (url, _):
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
    with running_mux(fw) as (url, _):
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


def wait_until(predicate, timeout_s=3.0):
    deadline = time.monotonic() + timeout_s
    while not predicate():
        if time.monotonic() > deadline:
            raise AssertionError("timed out waiting for the multiplexer")
        time.sleep(0.01)


def row_times(events, count, timeout_s=3.0):
    times, deadline = [], time.monotonic() + timeout_s
    while len(times) < count:
        event = events.get(timeout=max(0.0, deadline - time.monotonic()))
        if isinstance(event, Row):
            times.append(time.monotonic())
    return times


class RawClient:
    """A client that sends frames itself, so a test decides what it sends and when it reads."""

    def __init__(self, url):
        port = int(url.rsplit(":", 1)[1])
        self.sock = socket.create_connection(("127.0.0.1", port), timeout=3.0)
        self._decoder = FrameDecoder()
        self._inbox = []

    def send(self, msg):
        self.sock.sendall(encode_frame(msg.SerializeToString()))

    def next(self):
        while not self._inbox:
            data = self.sock.recv(65536)
            if not data:
                raise EOFError("the multiplexer closed the connection")
            for item in self._decoder.feed(data):
                if isinstance(item, bytes):
                    self._inbox.append(pb.Message.FromString(item))
        return self._inbox.pop(0)

    def next_body(self, body):
        """The next message with this body; rows and logs that come first are skipped."""
        while True:
            msg = self.next()
            if msg.WhichOneof("body") == body:
                return msg

    def close(self):
        self.sock.close()


def subscribe_msg(slot, ids):
    return pb.Message(subscribe=pb.Subscribe(slot=slot, ids=ids, period_us=0))


def flood_logs(fw, count=2500, chunk=10):
    """Queues `count` Logs, which every client receives, a few at a time: the robot reads and writes
    once per loop, so small chunks keep its rows flowing. Frames are built here; fw.log is too slow."""
    for _ in range(count // chunk):
        with fw._lock:
            port = fw._ports[0]
            blob = b"".join(
                encode_frame(pb.Message(seq=port.seq + 1 + i, log=pb.Log(level=pb.Log.INFO, text="x" * 200, tag="flood"))
                             .SerializeToString())
                for i in range(chunk))
            port.seq += chunk
            port.out += blob
        time.sleep(0.001)


def test_a_client_that_never_reads_is_disconnected_and_does_not_stall_the_others(monkeypatch):
    fw = FakeFirmware(lease_s=60.0)  # the flood can take longer than the default lease of 5 s
    waits = []  # how long each send in the multiplexer's loop took
    send_raw = mux_module.Multiplexer._send_raw

    def timed_send_raw(self, client, data):
        started = time.monotonic()
        send_raw(self, client, data)
        waits.append(time.monotonic() - started)

    monkeypatch.setattr(mux_module.Multiplexer, "_send_raw", timed_send_raw)
    with running_mux(fw) as (url, mux):
        stalled = socket.socket()
        stalled.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4096)
        stalled.connect(("127.0.0.1", int(url.rsplit(":", 1)[1])))
        b = Link(url, timeout_s=2.0)
        try:
            b.subscribe(["loop_count"], rate_hz=1000)  # one row per loop of the robot
            row_times(b.events, 50)
            flooder = threading.Thread(target=flood_logs, args=(fw,), daemon=True)
            flooder.start()
            flooder.join()
            time.sleep(0.2)  # the robot's last writes reach the multiplexer before teardown
            # No send in the loop blocked: a stalled client costs the others nothing.
            assert max(waits) < 0.5
            row_times(b.events, 300, timeout_s=10.0)  # b keeps streaming after the flood
            # The stalled client was disconnected: it reads what the kernel holds, then end of file.
            stalled.settimeout(2.0)
            while stalled.recv(65536):
                pass
        finally:
            stalled.close()
            b.close()
            wait_until(lambda: fw.subscribed_slots() == [])  # the robot stops streaming before teardown


def test_a_client_whose_counter_restarted_gets_seq_1_even_after_a_robot_gap():
    fw = FakeFirmware()
    with running_mux(fw) as (url, mux):
        raw = RawClient(url)
        try:
            raw.send(pb.Message(get_param=pb.GetParam(id=0)))
            tracker = SeqTracker()
            tracker.observe(raw.next_body("ack").seq)  # the client's last seq is 1
            fw.restart()
            fw.emit(pb.Message(signal_batch=pb.SignalBatch(slot=7, rows=b"")))  # restart seen; nobody owns slot 7
            fw.drop_frames(2)
            fw.log("after the gap", "test")
            step = tracker.observe(raw.next_body("log").seq)
            assert step.restarted is True
            assert step.lost == 0
        finally:
            raw.close()


def test_a_refused_catalog_request_stops_waiting_for_its_index():
    fw = FakeFirmware()
    with running_mux(fw) as (url, mux):
        raw = RawClient(url)
        try:
            raw.send(pb.Message(get_catalog=pb.GetCatalog(index=999)))
            assert raw.next_body("ack").ack.accepted is False
            assert 999 not in mux._catalog_waits
        finally:
            raw.close()


def test_a_robot_restart_clears_catalog_waits_and_pending_requests():
    fw = FakeFirmware()
    with running_mux(fw) as (url, mux):
        raw = RawClient(url)
        try:
            raw.send(pb.Message(get_catalog=pb.GetCatalog(index=0)))
            raw.next_body("catalog_entry")  # the robot has sent one frame, so its next restart is seen
            with fw._lock:  # the robot answers nothing while held
                raw.send(pb.Message(get_catalog=pb.GetCatalog(index=3)))
                wait_until(lambda: 3 in mux._catalog_waits)
                fw._ports[0].seq = 0  # the robot restarts: its next frame is seq 1
            wait_until(lambda: mux._seq_in.restarts == 1)
            wait_until(lambda: not mux._pending)
            assert mux._catalog_waits == {}
        finally:
            raw.close()


def test_a_slot_the_robot_refuses_or_expires_is_free_for_other_clients():
    fw = FakeFirmware()
    with running_mux(fw) as (url, mux):
        owner = RawClient(url)
        other = Link(url, timeout_s=2.0)
        try:
            owner.send(subscribe_msg(0, [0]))
            slot = int.from_bytes(owner.next_body("ack").ack.value, "little")
            with pytest.raises(LinkError, match="^unsubscribe refused: slot in use$"):
                other.unsubscribe(slot)
            owner.send(subscribe_msg(slot, [999]))  # refused by the robot: the owner loses the slot
            assert owner.next_body("ack").ack.accepted is False
            other.unsubscribe(slot)

            owner.send(subscribe_msg(0, [0]))
            slot = int.from_bytes(owner.next_body("ack").ack.value, "little")
            fw.log(f"slot {slot} lease expired", "link")
            assert owner.next_body("log").log.text == f"slot {slot} lease expired"
            other.unsubscribe(slot)
        finally:
            owner.close()
            other.close()
            wait_until(lambda: fw.subscribed_slots() == [])


def test_a_client_that_disconnects_mid_chunk_gets_none_of_its_later_frames_forwarded():
    fw = FakeFirmware()
    with running_mux(fw) as (url, mux):
        owner = Link(url, timeout_s=2.0)
        raw = RawClient(url)
        try:
            slot = owner.subscribe(["loop_count"], rate_hz=50).slot
            refused = encode_frame(pb.Message(unsubscribe=pb.Unsubscribe(slot=slot)).SerializeToString())
            asks = encode_frame(pb.Message(get_param=pb.GetParam(id=0)).SerializeToString())
            # Each refusal goes back to this client at once; it has closed, so the replies fail
            # part way through the chunk. The get_param frames after them must not reach the robot.
            raw.sock.sendall(refused * 2000 + asks * 500)
            raw.close()
            wait_until(lambda: len(mux._clients) == 1)
            time.sleep(0.3)
            asked = [p for p in fw.received if pb.Message.FromString(p).WhichOneof("body") == "get_param"]
            assert asked == []
        finally:
            owner.close()
            wait_until(lambda: fw.subscribed_slots() == [])


def test_the_ack_of_a_request_whose_client_has_left_is_dropped():
    fw = FakeFirmware()
    with running_mux(fw) as (url, mux):
        other = Link(url, timeout_s=2.0)
        raw = RawClient(url)
        try:
            with fw._lock:  # the robot answers nothing while held
                raw.send(pb.Message(get_param=pb.GetParam(id=0)))
                wait_until(lambda: mux._pending)
                raw.close()
                wait_until(lambda: len(mux._clients) == 1)
            wait_until(lambda: not mux._pending)  # the robot's Ack arrives and is dropped
            assert other.get("speed_kp") == pytest.approx(0.08)
        finally:
            raw.close()
            other.close()


def test_a_subscribe_in_flight_when_its_client_leaves_is_undone_when_accepted():
    fw = FakeFirmware()
    with running_mux(fw) as (url, mux):
        raw = RawClient(url)
        with fw._lock:  # the robot answers nothing while held
            raw.send(subscribe_msg(0, [0]))
            wait_until(lambda: any(p.body == "subscribe" for p in mux._pending.values()))
            raw.close()
            wait_until(lambda: len(mux._clients) == 0)
        # The robot accepts the subscribe; the multiplexer then unsubscribes it, so the slot is gone again.
        wait_until(lambda: any(pb.Message.FromString(p).WhichOneof("body") == "unsubscribe" for p in fw.received))
        assert fw.subscribed_slots() == []


def test_pending_requests_expire_after_their_time_limit(monkeypatch):
    monkeypatch.setattr(mux_module, "PENDING_S", 0.2)
    fw = FakeFirmware()
    with running_mux(fw) as (url, mux):
        raw = RawClient(url)
        try:
            with fw._lock:  # the robot answers nothing while held
                raw.send(pb.Message(get_param=pb.GetParam(id=0)))
                wait_until(lambda: mux._pending)
                wait_until(lambda: not mux._pending)
        finally:
            raw.close()
