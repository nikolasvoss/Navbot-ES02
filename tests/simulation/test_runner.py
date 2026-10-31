"""The batch runner: the CSV layout, the exit codes and the determinism of navbot_sim."""

import subprocess
import tempfile
from pathlib import Path

from conftest import run, run_sim

CSV_HEADER = (
    "t_us,x_m,v_mps,pitch_rad,pitch_rate_rad_s,yaw_rad,yaw_rate_rad_s,left_wheel_rad_s,right_wheel_rad_s,bus_v,"
    "left_current_a,right_current_a,tilt_meas_rad,balance_state,balance_fault,motor1_target_a,motor2_target_a,"
    "motors_enabled,overspeed_tripped,lifted,fallen"
)


def test_csv_header_word_for_word(sim_binary):
    result = run(sim_binary, "0 end\n")
    assert result.returncode == 0
    assert result.stdout.splitlines() == [CSV_HEADER]


def test_empty_scenario_exits_zero(sim_binary):
    assert run(sim_binary, "0 end\n").returncode == 0


def test_bad_line_exits_two_with_line_number(sim_binary):
    result = run(sim_binary, "0 bogus 1\n0 end\n")
    assert result.returncode == 2
    assert "line 1:" in result.stderr


def test_unknown_set_exits_two(sim_binary):
    result = run(sim_binary, "0 end\n", "no_such_parameter=1")
    assert result.returncode == 2


def test_ch5_off_keeps_balance_disabled(sim_binary):
    rows, _ = run_sim(sim_binary, "1 end\n")
    assert len(rows) > 0
    assert all(row["balance_state"] == 0.0 for row in rows)
    assert all(row["motors_enabled"] == 0.0 for row in rows)


def test_same_seed_gives_identical_csv(sim_binary):
    scenario = "0 rc 5 192\n0.5 rc 5 1792\n1.5 end\n"
    first = run(sim_binary, scenario, "seed=7")
    second = run(sim_binary, scenario, "seed=7")
    assert first.returncode == 0
    assert first.stdout == second.stdout
    assert len(first.stdout.splitlines()) > 1


def test_output_file_matches_stdout(sim_binary):
    scenario = "0.2 end\n"
    stdout_csv = run(sim_binary, scenario).stdout
    with tempfile.TemporaryDirectory() as folder:
        out = Path(folder) / "run.csv"
        scenario_file = Path(folder) / "scenario.txt"
        scenario_file.write_text(scenario)
        subprocess.run([str(sim_binary), "--scenario", str(scenario_file), "--out", str(out)], check=True)
        assert out.read_text() == stdout_csv
