import queue
import time

import pytest

from fake_firmware import FakeFirmware, MemoryTransport
from navbot_link import navbot_link_pb2 as pb
from navbot_link.framing import FrameDecoder, Text
from navbot_link.link import Disconnected, FramesLost, Link, LogEvent, Restarted, Row, SubscriptionEnded
from navbot_link.protocol import Arg, Entry, Kind, LinkError, Type


class RecordingTransport(MemoryTransport):
    def __init__(self, firmware):
        super().__init__(firmware)
        self.written = []

    def write(self, data: bytes) -> None:
        self.written.append(data)
        super().write(data)


class LossyTransport(MemoryTransport):
    """Drops the PC's frames whose body is in `lose`, as if the wire lost them."""

    def __init__(self, firmware, lose):
        super().__init__(firmware)
        self.lose = set(lose)

    def write(self, data: bytes) -> None:
        msg = pb.Message()
        msg.ParseFromString(FrameDecoder().feed(data)[0])
        if msg.WhichOneof("body") not in self.lose:
            super().write(data)


def open_link(fw, **kwargs):
    kwargs.setdefault("clock", lambda: 0.0)
    return Link(MemoryTransport(fw), **kwargs)


def next_events(link, count, kind=object, timeout_s=2.0):
    out, deadline = [], time.monotonic() + timeout_s
    while len(out) < count:
        event = link.events.get(timeout=max(0.0, deadline - time.monotonic()))
        if isinstance(event, kind):
            out.append(event)
    return out


def test_catalog_is_read_entry_by_entry_starting_with_the_literal_request_frame():
    fw = FakeFirmware()
    transport = RecordingTransport(fw)
    with Link(transport, clock=lambda: 0.0) as link:
        catalog = link.catalog()
    # Message{seq: 1, get_catalog: {index: 0}}, CRC 0xC63B, COBS, 0x00
    assert transport.written[0] == bytes.fromhex("05 08 01 b2 01 03 3b c6 00")
    assert len(transport.written) == 14
    assert catalog.build == "fake-build-1"
    assert catalog.entries[9] == Entry(9, "speed_kp", "rad/(m/s)", Kind.PARAM, Type.F32)
    assert catalog.entries[13] == Entry(13, "motor_current", "", Kind.ACTION, None, (
        Arg("left_a", -2.0, 2.0), Arg("right_a", -2.0, 2.0), Arg("lease_ms", 1.0, 1000.0)))


def test_get_set_and_call_send_literal_requests_and_return_robot_values():
    fw = FakeFirmware()
    with open_link(fw) as link:
        link.catalog()  # seq 1 to 14
        assert link.set("speed_kp", 0.12) == 0.11999999731779099
        assert link.get("speed_i_on") is True
        assert link.get("wheel_offset") == -3
        link.call("motor_current", left_a=0.5, right_a=-0.5, lease_ms=200)
    assert fw.received[14:] == [
        # seq 15, set_param{id: 9, value: f32 0.12}
        bytes.fromhex("08 0f d2 01 08 08 09 12 04 8f c2 f5 3d"),
        # seq 16, get_param{id: 11}
        bytes.fromhex("08 10 ca 01 02 08 0b"),
        # seq 17, get_param{id: 8}
        bytes.fromhex("08 11 ca 01 02 08 08"),
        # seq 18, call{id: 13, args: [0.5, -0.5, 200.0]}
        bytes.fromhex("08 12 da 01 10 08 0d 12 0c 00 00 00 3f 00 00 00 bf 00 00 48 43"),
    ]
    assert fw.calls == [("motor_current", [0.5, -0.5, 200.0])]


def test_robot_refusal_raises_with_its_reason():
    fw = FakeFirmware()
    with open_link(fw) as link:
        link.catalog()
        fw.entries[9].kind = 1  # the robot now treats speed_kp as a signal
        with pytest.raises(LinkError, match="^set_param refused: speed_kp is not a parameter$"):
            link.set("speed_kp", 0.2)


def test_call_checks_names_and_bounds_before_sending():
    fw = FakeFirmware()
    with open_link(fw) as link:
        link.catalog()
        with pytest.raises(LinkError, match=r"^left_a=3 is outside \[-2, 2\]$"):
            link.call("motor_current", left_a=3, right_a=0, lease_ms=10)
        with pytest.raises(LinkError, match="^motor_current takes left_a, right_a, lease_ms; unknown: left; "
                                            "missing: left_a$"):
            link.call("motor_current", left=1, right_a=0, lease_ms=10)
        with pytest.raises(LinkError, match="^unknown action motor_off$"):
            link.call("motor_off")
        link.call("motor_reset")
    assert fw.calls == [("motor_reset", [])]


def test_unknown_signal_is_refused_before_subscribing():
    fw = FakeFirmware()
    with open_link(fw) as link:
        with pytest.raises(LinkError, match="^unknown signal k58_drive_mode$"):
            link.subscribe(["speed_target", "k58_drive_mode"], 10)
    assert fw.subscribed_slots() == []


def test_subscription_streams_typed_rows_at_the_requested_period():
    fw = FakeFirmware()
    with open_link(fw) as link:
        sub = link.subscribe(["speed_measured", "tilt_clipped", "loop_count", "wheel_offset", "battery_min"], 50)
        rows = next_events(link, 3, Row)
    assert (sub.slot, sub.period_us) == (1, 20_000)
    assert rows == [
        Row(1, 1_010_000, (0.125, True, 101, -3, 7.5)),
        Row(1, 1_030_000, (0.375, True, 103, -3, 7.5)),
        Row(1, 1_050_000, (0.125, True, 105, -3, 7.5)),
    ]


def test_two_subscriptions_get_their_own_slots_and_unsubscribe_stops_one():
    fw = FakeFirmware()
    with open_link(fw) as link:
        fast = link.subscribe(["loop_count"], 0)
        slow = link.subscribe(["speed_target"], 10)
        assert (fast.slot, slow.slot) == (1, 2)
        link.unsubscribe(fast.slot)
        assert fw.subscribed_slots() == [2]


def test_the_robot_assigns_slots_across_ports_and_each_port_keeps_its_own():
    fw = FakeFirmware()
    uart = fw.add_port()
    with open_link(fw) as usb_link, Link(MemoryTransport(fw, uart), clock=lambda: 0.0) as uart_link:
        on_usb = usb_link.subscribe(["loop_count"], 100)
        on_uart = uart_link.subscribe(["speed_target"], 100)
        assert (on_usb.slot, on_uart.slot) == (1, 2)
        uart_row = next_events(uart_link, 1, Row)[0]
        assert (uart_row.slot, uart_row.values) == (2, (0.5,))
        with pytest.raises(LinkError, match="^unsubscribe refused: slot in use$"):
            uart_link.unsubscribe(1)
        assert fw.subscribed_slots() == [1, 2]
    # seq 15, subscribe{slot: 0, ids: [6], period_us: 10000}: slot 0 asks the robot to pick
    assert fw.received[14] == bytes.fromhex("08 0f ba 01 06 12 01 06 18 90 4e")


def test_lease_renewal_keeps_a_stream_alive_past_the_firmware_lease():
    renewing, idle = FakeFirmware(lease_s=0.3), FakeFirmware(lease_s=0.3)
    with open_link(renewing, renew_s=0.1) as a, open_link(idle, renew_s=60) as b:
        a.subscribe(["loop_count"], 100)
        b.subscribe(["loop_count"], 100)
        time.sleep(0.8)
        assert renewing.subscribed_slots() == [1]
        assert idle.subscribed_slots() == []


def test_an_unconfirmed_unsubscribe_keeps_the_slot_until_its_lease_has_run_out():
    fw = FakeFirmware(lease_s=0.5)
    transport = LossyTransport(fw, lose={"unsubscribe"})
    with Link(transport, clock=lambda: 0.0, timeout_s=0.05, renew_s=0.05, lease_s=0.5) as link:
        link.subscribe(["loop_count"], 100)
        with pytest.raises(LinkError, match="^no reply to unsubscribe$"):
            link.unsubscribe(1)
        assert link.slots == (1,)
        while not link.events.empty():
            link.events.get_nowait()
        row = next_events(link, 1, Row)[0]  # the robot still streams the slot
        assert (row.slot, row.values) == (1, (row.t_us // 10_000,))
        time.sleep(0.8)
        assert (link.slots, fw.subscribed_slots()) == ((), [])


def test_seq_gap_is_reported_and_a_restart_ends_subscriptions():
    fw = FakeFirmware()
    with open_link(fw) as link:
        link.subscribe(["loop_count"], 100)
        fw.drop_frames(2)
        lost = next_events(link, 1, FramesLost)
        assert lost[0].count == 2
        assert link.lost_frames == 2

        fw.restart()
        fw.log("boot", "setup")
        events = next_events(link, 3, (Restarted, SubscriptionEnded, LogEvent))
        assert events == [Restarted(), SubscriptionEnded(1, "robot restarted"),
                          LogEvent(t_ms=fw.t_us // 1000, level="INFO", code=0, text="boot", tag="setup")]
        before = len(fw.received)
        assert link.catalog().build == "fake-build-1"
        assert len(fw.received) - before == 14  # catalog read again


def test_a_restart_during_a_call_fails_the_call():
    class RestartingTransport(MemoryTransport):
        restart_before_write = False

        def write(self, data):
            if self.restart_before_write:
                self.firmware.restart()
            super().write(data)

    fw = FakeFirmware()
    transport = RestartingTransport(fw)
    with Link(transport, clock=lambda: 0.0) as link:
        link.catalog()
        transport.restart_before_write = True
        with pytest.raises(LinkError, match="^robot restarted$"):
            link.call("motor_reset")
        assert next_events(link, 1, Restarted) == [Restarted()]
    assert fw.calls == [("motor_reset", [])]


def test_usb_port_passes_boot_text_through_between_frames():
    fw = FakeFirmware(usb=True)
    with open_link(fw) as link:
        assert link.get("speed_kp") == pytest.approx(0.08)
        fw.write_text(b"E (1234) task_wdt: Task watchdog got triggered\r\n")
        text = next_events(link, 1, Text)
        assert link.get("speed_ki") == 0.5
    assert text == [Text("E (1234) task_wdt: Task watchdog got triggered\r\n")]
    assert link.bad_frames == 0


def test_a_log_level_the_schema_lacks_is_reported_by_number():
    fw = FakeFirmware()
    with open_link(fw) as link:
        fw.log("from a newer build", "x", level=9)
        assert next_events(link, 1, LogEvent) == [
            LogEvent(t_ms=1000, level="9", code=0, text="from a newer build", tag="x")]
        assert link.get("wheel_offset") == -3


def test_a_reply_value_of_the_wrong_size_raises():
    class ShortValues(FakeFirmware):
        def _handle(self, msg, port):
            if msg.WhichOneof("body") == "get_param":
                return self._ack(msg.seq, port, value=b"\x01\x02")
            super()._handle(msg, port)

    with open_link(ShortValues()) as link:
        with pytest.raises(LinkError, match="^value of 2 bytes, expected 4$"):
            link.get("speed_kp")


def test_an_error_on_the_io_thread_disconnects_instead_of_hanging(monkeypatch):
    def broken(types, data):
        raise RuntimeError("boom")

    monkeypatch.setattr("navbot_link.link.unpack_rows", broken)
    fw = FakeFirmware()
    with open_link(fw) as link:
        link.subscribe(["loop_count"], 100)
        assert next_events(link, 1, Disconnected) == [Disconnected("I/O thread failed: RuntimeError: boom")]
        started = time.monotonic()
        with pytest.raises(LinkError, match="^I/O thread failed: RuntimeError: boom$"):
            link.get("speed_kp")
        assert time.monotonic() - started < 0.1


def test_no_reply_raises_after_retries():
    class Silent:
        def read(self):
            time.sleep(0.001)
            return b""

        def write(self, data):
            pass

        def close(self):
            pass

    with Link(Silent(), timeout_s=0.05) as link:
        with pytest.raises(LinkError, match="^no reply to get_catalog$"):
            link.catalog()
    with pytest.raises(queue.Empty):
        link.events.get_nowait()
