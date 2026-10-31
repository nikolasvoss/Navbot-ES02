import importlib.util
import io
import json
import subprocess
import sys
import time
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "src"))
sys.path.insert(0, str(ROOT / "tests" / "navbot_link"))
SPEC = importlib.util.spec_from_file_location("navlink", ROOT / "scripts" / "navlink.py")
navlink = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(navlink)

from fake_firmware import FakeFirmware, MemoryTransport, PtyFirmware  # noqa: E402
from navbot_link import navbot_link_pb2 as pb  # noqa: E402
from navbot_link.link import Link  # noqa: E402
from navbot_link.protocol import Arg, Entry, Kind  # noqa: E402

SPEED_LOOP_SIGNALS = "speed_target,speed_measured,speed_p,speed_i,tilt_ref,tilt_clipped"
SPEED_LOOP_CSV = """\
# build: fake-build-1
# rate: 100 Hz
# param speed_kp=0.08
# param speed_ki=0.5
# param speed_i_on=1
t_us,speed_target,speed_measured,speed_p,speed_i,tilt_ref,tilt_clipped
1010000,0.5,0.125,-0.03125,0.001,0,1
# log t_ms=1020 INFO [param] speed_kp: 0.08 -> 0.12
1020000,0.5,0.25,-0.03125,0.002,0,0
1030000,0.5,0.375,-0.03125,0.003,0,1
1040000,0.5,0,-0.03125,0,0,0
1050000,0.5,0.125,-0.03125,0.001,0,1
"""


def run(argv, fw, stdin=None, opened=None):
    def open_link(port, baud):
        link = Link(MemoryTransport(fw))
        if opened is not None:
            opened.append(link)
        return link

    out = io.StringIO()
    rc = navlink.main(argv, open_link=open_link, stdin=stdin, stdout=out)
    return rc, out.getvalue()


def test_capture_with_the_shipped_preset_writes_header_rows_and_log_comments(tmp_path):
    fw = FakeFirmware()
    fw.log_at(1_020_000, "speed_kp: 0.08 -> 0.12", "param")
    csv = tmp_path / "speed-01.csv"
    rc, out = run(["capture", "--port", "/dev/ttyAMA3", "--signals", SPEED_LOOP_SIGNALS, "--rate", "100",
                   "--seconds", "0.05", "-o", str(csv)], fw)
    assert (rc, out) == (0, f"5 rows, 6 signals @ 100 Hz -> {csv}\n")
    assert csv.read_text() == SPEED_LOOP_CSV
    assert fw.subscribed_slots() == []  # unsubscribed at the end


def test_capture_prints_raw_text_from_the_robot_and_keeps_it_out_of_the_file(tmp_path, capsys):
    fw = FakeFirmware(usb=True)
    fw.text_at(1_030_000, b"I (5) stray print\r\n")
    csv = tmp_path / "speed-02.csv"
    rc, out = run(["capture", "--port", "/dev/ttyUSB0", "--signals", SPEED_LOOP_SIGNALS, "--rate", "100",
                   "--seconds", "0.05", "-o", str(csv)], fw)
    assert (rc, out) == (0, f"5 rows, 6 signals @ 100 Hz -> {csv}\n")
    assert capsys.readouterr().err == "I (5) stray print\n"
    assert csv.read_text() == SPEED_LOOP_CSV.replace("# log t_ms=1020 INFO [param] speed_kp: 0.08 -> 0.12\n", "")


def test_capture_fails_on_a_signal_the_firmware_lacks(tmp_path, capsys):
    presets = tmp_path / "presets.yaml"
    presets.write_text("drive:\n  rate_hz: 50\n  signals: [speed_target, k58_drive_mode]\n")
    csv = tmp_path / "drive.csv"
    rc, _ = run(["capture", "--port", "/dev/ttyAMA3", "--presets", str(presets), "--preset", "drive",
                 "--seconds", "1", "-o", str(csv)], FakeFirmware())
    assert rc == 2
    assert capsys.readouterr().err == "navlink: error: unknown signal k58_drive_mode\n"
    assert csv.read_text() == ""  # the output is opened before the link, so it exists and stays empty


def test_capture_output_that_cannot_be_written_is_a_usage_error_before_the_link_opens(tmp_path, capsys):
    opened = []
    missing = tmp_path / "no-such-dir" / "out.csv"
    rc, out = run(["capture", "--port", "/dev/ttyAMA3", "--signals", "speed_target", "--rate", "100",
                   "--seconds", "0.05", "-o", str(missing)], FakeFirmware(), opened=opened)
    assert (rc, out) == (2, "")
    assert capsys.readouterr().err == f"navlink: error: cannot write {missing}: No such file or directory\n"
    assert opened == []
    rc, _ = run(["capture", "--port", "/dev/ttyAMA3", "--signals", "speed_target", "--rate", "100",
                 "--seconds", "0.05", "-o", str(tmp_path)], FakeFirmware())
    assert rc == 2
    assert capsys.readouterr().err == f"navlink: error: cannot write {tmp_path}: Is a directory\n"


class RestartAfterSubscribe(MemoryTransport):
    """The robot restarts as soon as the subscription is in place."""

    restarted = False

    def read(self):
        if not self.restarted and self.firmware.subscribed_slots():
            self.restarted = True
            self.firmware.restart()
        return super().read()


def test_capture_ends_with_no_connection_when_the_robot_restarts(tmp_path, capsys):
    fw = FakeFirmware()
    csv = tmp_path / "restart.csv"
    code = navlink.main(["capture", "--port", "/dev/ttyAMA3", "--signals", "speed_target", "--rate", "100",
                         "--seconds", "5", "-o", str(csv)],
                        open_link=lambda port, baud: Link(RestartAfterSubscribe(fw)), stdout=io.StringIO())
    assert code == 3
    assert capsys.readouterr().err == "navlink: error: robot restarted\n"


def test_ctrl_c_exits_130_with_one_line(capsys):
    def open_link(port, baud):
        raise KeyboardInterrupt

    assert run_with(["info"], open_link) == 130
    assert capsys.readouterr().err == "navlink: interrupted\n"


def run_with(argv, open_link):
    return navlink.main(argv, open_link=open_link, stdout=io.StringIO())


def test_open_ready_closes_the_link_on_any_exception():
    class Interrupted:
        closed = False

        def catalog(self):
            raise KeyboardInterrupt

        def close(self):
            self.closed = True

    link = Interrupted()
    with pytest.raises(KeyboardInterrupt):
        navlink.open_ready(lambda port, baud: link, "port", 1, 0)
    assert link.closed


def test_a_link_that_dies_while_opening_is_no_connection(capsys):
    class ResetTransport:
        def read(self):
            raise OSError("connection reset")

        def write(self, data):
            pass

        def close(self):
            pass

    def open_link(port, baud):
        return Link(ResetTransport())

    for argv in (["info"], ["get", "speed_kp", "--wait", "0.3"]):
        assert run_with(argv, open_link) == 3
        assert capsys.readouterr().err == "navlink: error: connection reset\n"


def test_a_failing_close_does_not_change_the_exit_code(capsys):
    fw = FakeFirmware()

    def open_link(port, baud):
        link = Link(MemoryTransport(fw))
        close_link = link.close

        def close_then_fail():
            close_link()
            raise OSError("port already closed")

        link.close = close_then_fail
        return link

    assert run_with(["get", "nope"], open_link) == 2
    err = capsys.readouterr().err
    assert "navlink: error: unknown signal nope\n" in err
    assert "navlink: warning: closing the link failed: port already closed\n" in err


class Script:
    """Shell stdin: strings are lines, callables run before the next line is read."""

    def __init__(self, *steps):
        self.steps = list(steps)

    def readline(self):
        while self.steps:
            step = self.steps.pop(0)
            if callable(step):
                step()
                continue
            return step + "\n"
        return ""


def wait_until(predicate, timeout_s=2.0):
    deadline = time.monotonic() + timeout_s
    while not predicate():
        assert time.monotonic() < deadline, "timed out"
        time.sleep(0.005)


def test_shell_session(tmp_path):
    fw = FakeFirmware(stop_us=1_090_000)
    csv = tmp_path / "sub.csv"
    opened = []

    def all_rows_queued_and_taken():
        wait_until(lambda: fw.t_us == fw.stop_us)
        time.sleep(0.02)  # the link reads and queues the last row within a few ms
        wait_until(opened[0].events.empty)

    stdin = Script(
        "log off",
        "list speed_k*",
        "list motor_current",
        "get speed_kp speed_i_on",
        "set speed_kp 0.12",
        "set speed_target 1",
        "call motor_current left_a=0.5 right_a=0 lease_ms=200",
        "y",
        "call motor_reset",
        "n",
        "call motor_current left_a=5 right_a=0 lease_ms=200",
        f"sub speed_target loop_count @50 -o {csv}",
        all_rows_queued_and_taken,
        "unsub 1",
        "sub nope",
        "quit",
    )
    rc, out = run(["shell", "--port", "/dev/ttyAMA3"], fw, stdin=stdin, opened=opened)
    assert rc == 0
    assert out == """\
speed_kp                 param      f32  rad/(m/s) live ±0.05
speed_ki                 param      f32  rad/(m/s*s) live ±0.02
motor_current            action          left_a [-2, 2], right_a [-2, 2], lease_ms [1, 1000]
speed_kp = 0.08
speed_i_on = 1
speed_kp: 0.08 -> 0.12
error: speed_target is not a parameter (kind signal)
motor_current(left_a=0.5 [-2, 2], right_a=0 [-2, 2], lease_ms=200 [1, 1000])? [y/N] motor_current: started
motor_reset()? [y/N] not sent
error: left_a=5 is outside [-2, 2]
slot 1: 2 entries @ 50 Hz, 1.9 kB/s -> {csv}
slot 1 closed
error: unknown signal nope
""".format(csv=csv)
    assert fw.calls == [("motor_current", [0.5, 0.0, 200.0])]
    assert csv.read_text() == """\
# build: fake-build-1
# rate: 50 Hz
# param speed_kp=0.12
# param speed_ki=0.5
# param speed_i_on=1
t_us,speed_target,loop_count
1010000,0.5,101
1030000,0.5,103
1050000,0.5,105
1070000,0.5,107
1090000,0.5,109
"""


def test_shell_quit_stops_its_subscriptions(tmp_path):
    fw = FakeFirmware()
    csv = tmp_path / "quit.csv"
    rc, out = run(["shell", "--port", "/dev/ttyAMA3"], fw,
                  stdin=Script("log off", f"sub loop_count @10 -o {csv}", "quit"))
    assert (rc, out) == (0, f"slot 1: 1 entries @ 10 Hz, 0.3 kB/s -> {csv}\n")
    assert fw.subscribed_slots() == []


def test_capture_over_a_serial_port(tmp_path):
    pty = PtyFirmware(FakeFirmware())
    csv = tmp_path / "pty.csv"
    try:
        result = subprocess.run(
            [sys.executable, str(ROOT / "scripts" / "navlink.py"), "capture", "--port", pty.path,
             "--signals", "speed_target,loop_count", "--rate", "50", "--seconds", "0.1", "-o", str(csv)],
            capture_output=True, text=True, timeout=30,
        )
    finally:
        pty.close()
    assert (result.returncode, result.stdout, result.stderr) == (
        0, f"5 rows, 2 signals @ 50 Hz -> {csv}\n", "")
    assert csv.read_text() == """\
# build: fake-build-1
# rate: 50 Hz
# param speed_kp=0.08
# param speed_ki=0.5
# param speed_i_on=1
t_us,speed_target,loop_count
1010000,0.5,101
1030000,0.5,103
1050000,0.5,105
1070000,0.5,107
1090000,0.5,109
"""


def test_info_is_text_and_json():
    rc, out = run(["info"], FakeFirmware())
    assert (rc, out) == (0, "build fake-build-1\nentries 14\nport tcp:127.0.0.1:7650\n")
    rc, out = run(["info", "--json"], FakeFirmware())
    assert (rc, out) == (0, '{"build": "fake-build-1", "entries": 14, "port": "tcp:127.0.0.1:7650"}\n')


def test_list_text_and_json():
    rc, out = run(["list", "speed_k*"], FakeFirmware())
    assert (rc, out) == (0, "speed_kp                 param      f32  rad/(m/s) live ±0.05\n"
                            "speed_ki                 param      f32  rad/(m/s*s) live ±0.02\n")
    rc, out = run(["list", "speed_k*", "--json"], FakeFirmware())
    assert json.loads(out) == [
        {"name": "speed_kp", "kind": "PARAM", "type": "F32", "unit": "rad/(m/s)", "live_step": 0.05},
        {"name": "speed_ki", "kind": "PARAM", "type": "F32", "unit": "rad/(m/s*s)", "live_step": 0.02},
    ]
    rc, out = run(["list", "motor_current", "--json"], FakeFirmware())
    assert (rc, out) == (0, '[{"name": "motor_current", "kind": "ACTION", "unit": "", "args": ['
                            '{"name": "left_a", "min": -2.0, "max": 2.0}, '
                            '{"name": "right_a", "min": -2.0, "max": 2.0}, '
                            '{"name": "lease_ms", "min": 1.0, "max": 1000.0}]}]\n')


def test_get_set_and_call_in_json():
    fw = FakeFirmware()
    rc, out = run(["get", "speed_kp", "speed_i_on", "--json"], fw)
    assert (rc, out) == (0, '{"speed_kp": 0.08, "speed_i_on": true}\n')
    rc, out = run(["set", "speed_kp", "0.12", "--json"], fw)
    assert (rc, out) == (0, '{"name": "speed_kp", "old": 0.08, "new": 0.12}\n')
    rc, out = run(["call", "motor_current", "left_a=0.5", "right_a=0", "lease_ms=200", "--yes", "--json"], fw)
    assert (rc, out) == (0, '{"name": "motor_current", "args": {"left_a": 0.5, "right_a": 0.0, '
                            '"lease_ms": 200.0}, "accepted": true}\n')
    assert fw.calls == [("motor_current", [0.5, 0.0, 200.0])]


def test_call_without_yes_is_a_usage_error_and_sends_nothing(capsys):
    fw = FakeFirmware()
    rc, out = run(["call", "motor_reset"], fw)
    assert (rc, out) == (2, "")
    assert capsys.readouterr().err == "navlink: error: call needs --yes\n"
    assert fw.calls == []


def test_exit_codes_for_usage_refusal_and_no_connection(capsys):
    fw = FakeFirmware()
    fw.entries[9].limits = (0.0, 1.0)
    assert run(["get", "nope"], fw) == (2, "")
    assert capsys.readouterr().err == "navlink: error: unknown signal nope\n"
    assert run(["set", "speed_kp", "2"], fw) == (1, "")
    assert capsys.readouterr().err == "navlink: error: set_param refused: speed_kp must be within [0, 1]\n"

    class Silent:
        def read(self):
            time.sleep(0.01)
            return b""

        def write(self, data):
            pass

        def close(self):
            pass

    code = navlink.main(["info"], open_link=lambda port, baud: Link(Silent(), timeout_s=0.05, retries=1),
                        stdout=io.StringIO())
    assert code == 3
    assert capsys.readouterr().err == "navlink: error: no reply to get_catalog\n"


class LateTransport:
    """Drops what the PC writes until `answer_at`, like a robot that boots late."""

    def __init__(self, firmware, answer_at):
        self.inner = MemoryTransport(firmware)
        self.answer_at = answer_at

    def read(self):
        return self.inner.read()

    def write(self, data):
        if time.monotonic() >= self.answer_at:
            self.inner.write(data)

    def close(self):
        self.inner.close()


def test_wait_retries_until_the_robot_answers():
    fw = FakeFirmware()
    answer_at = time.monotonic() + 0.3
    out = io.StringIO()
    code = navlink.main(["info", "--wait", "3"],
                        open_link=lambda port, baud: Link(LateTransport(fw, answer_at), timeout_s=0.1, retries=1),
                        stdout=out)
    assert (code, out.getvalue()) == (0, "build fake-build-1\nentries 14\nport tcp:127.0.0.1:7650\n")


def test_log_prints_events_at_or_above_the_minimum_level():
    fw = FakeFirmware()
    fw.log("speed_kp: 0.08 -> 0.12", "param", pb.Log.INFO)
    fw.log("noisy", "imu", pb.Log.DEBUG)
    rc, out = run(["log", "--seconds", "0.3", "--min-level", "INFO"], fw)
    assert (rc, out) == (0, "log t_ms=1000 INFO [param] speed_kp: 0.08 -> 0.12\n")
    fw.log("again", "param", pb.Log.WARN)
    rc, out = run(["log", "--seconds", "0.3", "--min-level", "WARN", "--json"], fw)
    assert (rc, out) == (0, '{"t_ms": 1000, "level": "WARN", "tag": "param", "text": "again"}\n')


def test_set_follows_the_live_step_rule_while_balancing(capsys):
    fw = FakeFirmware(balancing=True)
    assert run(["set", "speed_kp", "0.12"], fw) == (0, "speed_kp: 0.08 -> 0.12\n")
    assert run(["set", "speed_kp", "0.2"], fw) == (1, "")  # from 0.12: a step of 0.08
    assert capsys.readouterr().err == "navlink: error: set_param refused: speed_kp: step 0.08 over 0.05 while balancing\n"
    assert run(["set", "speed_i_on", "0"], fw) == (1, "")
    assert capsys.readouterr().err == "navlink: error: set_param refused: speed_i_on: refused while balancing\n"


def test_wait_gives_up_before_its_deadline_and_closes_every_link(capsys):
    class Silent:
        closed = False

        def read(self):
            time.sleep(0.01)
            return b""

        def write(self, data):
            pass

        def close(self):
            self.closed = True

    transports = []

    def open_link(port, baud):
        transports.append(Silent())
        return Link(transports[-1], timeout_s=0.05, retries=1)

    started = time.monotonic()
    code = run_with(["info", "--wait", "0.5"], open_link)
    assert code == 3
    assert time.monotonic() - started < 2.0
    assert len(transports) == 2  # a third attempt could not finish before the deadline
    assert all(transport.closed for transport in transports)
    assert capsys.readouterr().err == "navlink: error: no reply to get_catalog\n"


def test_log_json_is_one_object_per_event_at_or_above_the_minimum_level():
    fw = FakeFirmware()
    fw.log("first", "a", pb.Log.WARN)
    fw.log("skipped", "b", pb.Log.DEBUG)
    fw.log("second", "c", pb.Log.ERROR)
    rc, out = run(["log", "--seconds", "0.3", "--min-level", "WARN", "--json"], fw)
    assert rc == 0
    assert [json.loads(line) for line in out.splitlines()] == [
        {"t_ms": 1000, "level": "WARN", "tag": "a", "text": "first"},
        {"t_ms": 1000, "level": "ERROR", "tag": "c", "text": "second"},
    ]


def test_action_argument_bounds_in_json_go_through_the_float32_rounding():
    entry = Entry(0, "tune", "", Kind.ACTION, None, (Arg("gain", 0.1, 0.3),))
    assert json.dumps(navlink.entry_record(entry)["args"]) == '[{"name": "gain", "min": 0.1, "max": 0.3}]'
