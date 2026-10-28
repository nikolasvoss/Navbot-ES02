"""The analysis tools read navlink CSV captures (t_us column header, `#` comments)."""

import subprocess
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "scripts"))
sys.path.insert(0, str(ROOT / "src"))
from analyze_drive_trace import REQUIRED, analyze  # noqa: E402
from navbot_link.capture import load_presets  # noqa: E402
from summarize_balance_trace import read_trace, summarize  # noqa: E402

DRIVE_SIGNALS = set(load_presets(ROOT / "scripts" / "navlink_presets.yaml")["drive"].signals)

FIXTURES = ROOT / "tests" / "scripts" / "fixtures"
SPEED_LOOP = FIXTURES / "speed-loop-navlink.csv"
DRIVE = FIXTURES / "drive-navlink-synthetic.csv"

SPEED_LOOP_SUMMARY = f"""\
### `{SPEED_LOOP}`

- Trace-Modus: **navlink**; gültige Datensätze: **5**; beschädigte/ungültige Datensätze: **0**
- Zeitspanne der gültigen Datensätze: **0.05 s**; Medianabstand zwischen gültigen Datensätzen: **10.0 ms**
- Zeitbasis: `t_us` (Roboter-`micros()`); Zeilen sind nicht gleichmäßig verteilt; Firmware: **synthetic-1**; Rate: **100 Hz**
- Parameter bei Start: `speed_kp=0.08`, `speed_ki=0.5`, `speed_i_on=1`
- Verlorene Frames (`# lost`): **3**
- Log-Ereignisse: **1**
  - `log t_ms=1030 INFO [param] speed_kp: 0.08 -> 0.12`

| Messgröße | Min | Max | Mittel | Anfang → Ende |
|---|---:|---:|---:|---:|
| `speed_target` | 0.5 | 0.5 | 0.5 | 0.5 → 0.5 |
| `speed_measured` | 0.125 | 0.5 | 0.35 | 0.125 → 0.5 |
| `speed_p` | -0.062 | -0.031 | -0.044 | -0.031 → -0.062 |
| `speed_i` | 0.001 | 0.005 | 0.003 | 0.001 → 0.005 |
| `tilt_ref` | 0 | 0.02 | 0.01 | 0 → 0.02 |
| `tilt_clipped` | 0 | 1 | 0.6 | 1 → 1 |
"""

DRIVE_RESULT = {
    "file": str(DRIVE),
    "valid_rows": 26,
    "malformed_rows": 0,
    "bad_fraction": 0.0,
    "capture_ok": True,
    "duration_s": 0.51,
    "median_interval_ms": 20.5,
    "active_rows": 26,
    "activation_roll_deg": 0.2,
    "command_rows": 4,
    "releases": [{
        "at_ms": 5222.7,
        "command_direction": "vorwaerts",
        "command_peak": 5.0,
        "command_duration_s": 0.06,
        "speed_before_release": 4.0,
        "opposite_speed_peak": 7.0,
        "neutral_samples": 16,
    }],
    "opposite_speed_peak": 7.0,
    "motor_target_abs_peak": 2.5,
    "voltage_raw_min_v": 7.65,
    "imu_rate_hz": 342.9,
    "imu_mean_interval_s": 0.001,
    "imu_long_gaps": 2,
    "imu_long_gap_fraction": 0.0114,
    "imu_max_interval_us": 4100,
    "control_execution_us_peak": 325,
    "loop_entry_interval_us_peak": 1250,
    "motor_current_sp_abs_peak": 0.25,
    "motor_voltage_q_abs_peak": 0.5,
    "battery_raw_instant_min_v": 7.4,
    "stop_available": True,
}


def test_summary_of_a_speed_loop_capture_lists_every_signal_and_the_comment_lines():
    assert "\n".join(summarize(SPEED_LOOP)) == SPEED_LOOP_SUMMARY


def test_summary_cli_prints_the_same_report():
    result = subprocess.run([sys.executable, str(ROOT / "scripts" / "summarize_balance_trace.py"), str(SPEED_LOOP)],
                            capture_output=True, text=True, check=False)
    assert result.returncode == 0, result.stderr
    assert result.stdout == "# Balance-Trace: Run-Zusammenfassung\n\n" + SPEED_LOOP_SUMMARY


def test_drive_analysis_of_a_navlink_capture_with_uneven_rows():
    assert analyze(DRIVE) == DRIVE_RESULT


def test_stop_window_ends_by_t_us_not_by_row_count():
    result = analyze(FIXTURES / "drive-navlink-stop-window.csv")
    assert result["releases"] == [{
        "at_ms": 1300.0,
        "command_direction": "vorwaerts",
        "command_peak": 5.0,
        "command_duration_s": 0.2,
        "speed_before_release": 1.0,
        "opposite_speed_peak": 3.0,
        "neutral_samples": 3,
    }]


def test_drive_analysis_cli_reports_the_stop_from_t_us():
    result = subprocess.run([sys.executable, str(ROOT / "scripts" / "analyze_drive_trace.py"), str(DRIVE),
                             "--require-stop"], capture_output=True, text=True, check=False)
    assert result.returncode == 0, result.stderr
    assert f"| `{DRIVE}` | 26/0 | 26 | 0.2° | 4 | 1 | 7.0 | 2.5 | 7.65 V |" in result.stdout


def test_analysis_of_a_navlink_capture_without_drive_columns_names_the_missing_ones():
    with pytest.raises(ValueError, match="fehlende DRIVE-Spalten: battery_raw_instant_v, ch3_target, "):
        analyze(SPEED_LOOP)


def test_t_us_is_unwrapped_past_2_pow_32(tmp_path):
    log = tmp_path / "wrap.csv"
    log.write_text("t_us,x\n4294960000,1\n4294966000,2\n4000,3\n10000,4\n", encoding="utf-8")
    trace = read_trace(log)
    assert [row["time_ms"] for _, row in trace.rows] == [4294960.0, 4294966.0, 4294971.296, 4294977.296]


def test_malformed_navlink_rows_are_counted_not_parsed(tmp_path):
    log = tmp_path / "bad.csv"
    log.write_text("t_us,x,y\n1000,1,2\n2000,1\n3000,nan,2\n4000,a,2\nnoise before comma\n5000,3,4\n", encoding="utf-8")
    trace = read_trace(log)
    assert [row for _, row in trace.rows] == [
        {"t_us": 1000.0, "time_ms": 1.0, "x": 1.0, "y": 2.0},
        {"t_us": 5000.0, "time_ms": 5.0, "x": 3.0, "y": 4.0},
    ]
    assert trace.malformed == 4


def test_navlink_drive_columns_are_the_drive_preset_signals():
    columns = read_trace(DRIVE).header
    assert set(columns) - {"t_us", "time_ms"} <= DRIVE_SIGNALS
    assert REQUIRED - {"time_ms"} <= set(columns)
