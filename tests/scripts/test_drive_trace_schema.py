#!/usr/bin/env python3
"""DRIVE header contract, header-driven reading, and summary output for the new telemetry schema."""

import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "scripts"))
from capture_balance_trace import DRIVE_COLUMNS, TRACE_HEADERS
from summarize_balance_trace import read_trace, summarize

FIXTURE = ROOT / "tests" / "scripts" / "fixtures" / "drive-k58-synthetic.txt"

# Field order of the firmware DRIVE row (LogFormat.cpp, Logging::DriveSample) after the tag.
FIRMWARE_DRIVE_ORDER = [
    "time_ms", "ch5_mode", "voltage_min_raw_v", "voltage_filtered_v",
    "imu_sequence", "imu_read_time_us", "imu_dt_s", "imu_accepted_samples",
    "imu_long_gap_count", "imu_max_interval_us", "control_start_us", "control_execution_us",
    "max_loop_entry_interval_us", "ch3_target", "motor1_velocity_f", "motor2_velocity_f",
    "tick_dt_s", "body_pitching_f", "roll_ok", "angle_output",
    "angle_out_p", "angle_out_i", "angle_out_d",
    "motor1_target", "motor2_target", "motor1_current_sp", "motor2_current_sp",
    "motor1_voltage_q", "motor2_voltage_q", "motor1_shaft_velocity", "motor2_shaft_velocity",
    "battery_raw_instant_v", "battery_filtered_instant_v", "top_ball_x", "body_pitching",
    "max_servo_range_deg", "sequence", "logger_dropped", "uart_write_failures",
]


def drive_line(time_ms, **overrides):
    values = {name: "0" for name in DRIVE_COLUMNS}
    values.update(time_ms=str(time_ms), ch5_mode="1", voltage_min_raw_v="7.9", voltage_filtered_v="7.9",
                  imu_dt_s="0.007", motor1_target="0.05", motor2_target="-0.05")
    values.update({name: str(value) for name, value in overrides.items()})
    return "DRIVE," + ",".join(values[name] for name in DRIVE_COLUMNS)


class DriveSchemaTest(unittest.TestCase):
    def test_drive_columns_match_firmware_order_exactly(self):
        self.assertEqual(list(DRIVE_COLUMNS), FIRMWARE_DRIVE_ORDER)
        self.assertEqual(len(DRIVE_COLUMNS), 39)

    def test_capture_writes_the_same_drive_header(self):
        self.assertEqual(TRACE_HEADERS[58], "DRIVE," + ",".join(FIRMWARE_DRIVE_ORDER))

    def test_header_field_count_mismatch_is_malformed(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "mismatch.log"
            path.write_text("# DRIVE," + ",".join(FIRMWARE_DRIVE_ORDER) + "\n"
                            + drive_line(0) + "\n"
                            + drive_line(7).rsplit(",", 1)[0] + "\n",
                            encoding="utf-8")
            kind, header, rows, malformed = read_trace(path)
        self.assertEqual(kind, "DRIVE")
        self.assertEqual(len(rows), 1)
        self.assertEqual(malformed, 1)

    def test_nonfinite_value_is_malformed(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "nonfinite.log"
            path.write_text("# DRIVE," + ",".join(FIRMWARE_DRIVE_ORDER) + "\n"
                            + drive_line(0) + "\n"
                            + drive_line(7, motor1_current_sp="inf") + "\n"
                            + drive_line(14, battery_raw_instant_v="nan") + "\n",
                            encoding="utf-8")
            _, _, rows, malformed = read_trace(path)
        self.assertEqual(len(rows), 1)
        self.assertEqual(malformed, 2)

    def test_trace_capture_doc_lists_the_same_drive_columns(self):
        doc = (ROOT / "docs" / "robot" / "software" / "trace-capture.md").read_text(encoding="utf-8")
        start = doc.index("`time_ms,ch5_mode,voltage_min_raw_v,voltage_filtered_v,imu_sequence")
        documented = doc[start + 1:doc.index("`", start + 1)].split(",")
        self.assertEqual(documented, list(DRIVE_COLUMNS))

    def test_summary_reports_new_measurements(self):
        text = "".join(summarize(FIXTURE))
        self.assertIn("`control_execution_us`", text)
        self.assertIn("`motor1_current_sp`", text)
        self.assertIn("`battery_raw_instant_v`", text)
        self.assertIn("IMU-Abtastrate aus `imu_sequence`", text)

    def test_summary_cli_runs_on_fixture(self):
        result = subprocess.run([sys.executable, str(ROOT / "scripts" / "summarize_balance_trace.py"), str(FIXTURE)],
                                capture_output=True, text=True, check=False)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("Trace-Modus: **DRIVE**", result.stdout)


if __name__ == "__main__":
    unittest.main()
