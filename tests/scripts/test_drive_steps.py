"""drive_steps: the step analysis on a synthetic capture, the precondition refusal and the command loop."""

import io
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "scripts"))
sys.path.insert(0, str(ROOT / "src"))
sys.path.insert(0, str(ROOT / "tests" / "navbot_link"))

import drive_steps  # noqa: E402
import navlink  # noqa: E402
from drive_steps import Step, analyze_steps, drive, main, read_capture, smallest_moving_speed  # noqa: E402
from fake_firmware import FakeEntry, FakeFirmware, MemoryTransport  # noqa: E402
from navbot_link import navbot_link_pb2 as pb  # noqa: E402
from navbot_link.link import Link  # noqa: E402

FIXTURE = ROOT / "tests" / "scripts" / "fixtures" / "drive-steps-analysis.csv"
STEPS = [Step(0.5, 0.0, 1.0), Step(0.0, 0.0, 1.0)]  # the fixture: 0.5 m/s for 1 s, then zero
HEADER = ("t_us,ch5_position,balance_state,cmd_speed_mps,cmd_turn_rad_s,cmd_age_ms,motor1_velocity_f,"
          "motor2_velocity_f,roll_deg,gyro_z_rad_s,voltage_filtered_v")
PRECONDITION_SIGNALS = ("ch5_position", "balance_state")


def steps_firmware(ch5: int, balance: int) -> FakeFirmware:
    """A robot with the signals of the `steps` preset; ch5 and balance are the two preconditions."""
    values = {"ch5_position": ch5, "balance_state": balance}
    names = navlink.resolve_signals(["@steps"], navlink.DEFAULT_PRESETS)[0]
    entries = []
    for name in names:
        if name in PRECONDITION_SIGNALS:
            entries.append(FakeEntry(name, pb.CatalogEntry.SIGNAL, pb.CatalogEntry.U32, "", values[name]))
        else:
            entries.append(FakeEntry(name, pb.CatalogEntry.SIGNAL, pb.CatalogEntry.F32, "", 0.0))
    return FakeFirmware(entries=entries, link_position=True)


def write_steps(tmp_path: Path, text: str) -> Path:
    path = tmp_path / "steps.yaml"
    path.write_text(text, encoding="utf-8")
    return path


def test_the_fixture_gives_the_literal_step_results():
    results = analyze_steps(read_capture(FIXTURE), STEPS, wheel_radius_m=1.0)
    assert results == [
        {"speed_mps": 0.5, "turn_rad_s": 0.0, "seconds": 1.0, "t90_s": 0.6, "steady_error_mps": -0.022,
         "peak_roll_deg": 2.4, "stop_distance_m": 0.045, "rollback_m": 0.006, "moved": True},
        {"speed_mps": 0.0, "turn_rad_s": 0.0, "seconds": 1.0, "t90_s": None, "steady_error_mps": -0.01,
         "peak_roll_deg": 0.0, "stop_distance_m": None, "rollback_m": None, "moved": False},
    ]
    assert smallest_moving_speed(results) == 0.5


def test_the_wheel_radius_scales_the_speed_and_the_distances():
    results = analyze_steps(read_capture(FIXTURE), STEPS, wheel_radius_m=0.5)
    # Half the speed: the peak is 0.25 m/s, below 90 % of 0.5 m/s, so the step never reaches its target.
    assert results[0]["t90_s"] is None
    assert results[0]["rollback_m"] == 0.003
    assert results[0]["moved"] is True


def test_the_steps_must_start_with_a_command_the_robot_took():
    with pytest.raises(ValueError, match="never took the first step's command"):
        analyze_steps(read_capture(FIXTURE), [Step(0.7, 0.0, 1.0)], wheel_radius_m=1.0)


def test_a_robot_not_in_link_mode_with_balance_running_is_refused_before_any_command(tmp_path, capsys):
    fw = steps_firmware(ch5=2, balance=2)  # CH5 in RC balance, not link mode
    steps = write_steps(tmp_path, "rate_hz: 20\nsteps: [{speed_mps: 0.1, seconds: 1.0}]\n")
    out = tmp_path / "drive.csv"
    rc = main([str(steps), "-o", str(out)], open_link=lambda port, baud: Link(MemoryTransport(fw)))
    assert rc == 4
    assert "drive_steps: error: CH5 must be in position 1 with balance running" in capsys.readouterr().err
    assert fw.commands == []


def test_the_command_loop_sends_the_step_then_zero_and_captures_the_steps_preset(monkeypatch):
    monkeypatch.setattr(drive_steps, "TAIL_S", 0.1)
    fw = steps_firmware(ch5=1, balance=2)
    link = Link(MemoryTransport(fw))
    output = io.StringIO()
    try:
        drive(link, [Step(0.2, 0.0, 0.2)], 20, output, navlink.DEFAULT_PRESETS, io.StringIO())
    finally:
        link.close()
    sent = [(round(speed, 3), round(turn, 3)) for speed, turn in fw.commands]
    # Repeats of the same command are collapsed; the run is the step, then zero.
    assert [c for i, c in enumerate(sent) if i == 0 or sent[i - 1] != c] == [(0.2, 0.0), (0.0, 0.0)]
    assert sent[-1] == (0.0, 0.0)
    lines = output.getvalue().splitlines()
    assert lines[lines.index(HEADER) + 1:] != []  # rows were written under the header
    assert fw.subscribed_slots() == []  # unsubscribed at the end


def test_a_steps_file_without_a_rate_is_a_usage_error(tmp_path, capsys):
    steps = write_steps(tmp_path, "steps: [{speed_mps: 0.1, seconds: 1.0}]\n")
    fw = steps_firmware(ch5=1, balance=2)
    rc = main([str(steps), "-o", str(tmp_path / "drive.csv")], open_link=lambda port, baud: Link(MemoryTransport(fw)))
    assert rc == 2
    assert "rate_hz must be a finite number" in capsys.readouterr().err
