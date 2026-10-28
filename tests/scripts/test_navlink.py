import importlib.util
import io
import subprocess
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "src"))
sys.path.insert(0, str(ROOT / "tests" / "navbot_link"))
SPEC = importlib.util.spec_from_file_location("navlink", ROOT / "scripts" / "navlink.py")
navlink = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(navlink)

from fake_firmware import FakeFirmware, MemoryTransport, PtyFirmware  # noqa: E402
from navbot_link.link import Link  # noqa: E402

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
    assert rc == 1
    assert capsys.readouterr().err == "navlink: error: unknown signal k58_drive_mode\n"
    assert not csv.exists()


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
speed_kp                 param      f32  rad/(m/s)
speed_ki                 param      f32  rad/(m/s*s)
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
