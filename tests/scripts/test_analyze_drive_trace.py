#!/usr/bin/env python3
"""Focused checks for DRIVE release detection, capture-quality gating, and new measurements."""

import tempfile
import unittest
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "scripts"))
from analyze_drive_trace import analyze
from capture_balance_trace import DRIVE_COLUMNS


OLD_HEADER = ("DRIVE,time_ms,ch5_mode,voltage_min_raw_v,ch3_target,"
              "motor1_velocity_f,motor2_velocity_f,roll_ok,motor1_target,motor2_target,angle_out_i")


def drive_row(time_ms, command, speed, *, roll=0, ch5=1, **fields):
    values = {name: 0 for name in DRIVE_COLUMNS}
    values.update(time_ms=time_ms, ch5_mode=ch5, voltage_min_raw_v=7.9, voltage_filtered_v=7.9,
                  imu_sequence=time_ms, imu_read_time_us=time_ms * 1000, imu_dt_s=0.007,
                  imu_accepted_samples=time_ms,
                  imu_max_interval_us=1200, control_execution_us=500, max_loop_entry_interval_us=1000,
                  ch3_target=command, effective_speed_target=command, motor1_velocity_f=speed,
                  motor2_velocity_f=speed, roll_ok=roll, motor1_target=2, motor2_target=-2,
                  motor1_current_sp=0.1, motor2_current_sp=-0.1, motor1_voltage_q=0.2,
                  motor2_voltage_q=-0.2, battery_raw_instant_v=7.9, battery_filtered_instant_v=7.9,
                  angle_out_i=3)
    values.update(fields)
    return "DRIVE," + ",".join(str(values[name]) for name in DRIVE_COLUMNS)


class DriveTraceTest(unittest.TestCase):
    def write_log(self, directory, lines, header=None):
        path = Path(directory) / "drive.log"
        header = header or "DRIVE," + ",".join(DRIVE_COLUMNS)
        path.write_text("# " + header + "\n" + "\n".join(lines) + "\n", encoding="utf-8")
        return path

    def test_release_peak_uses_neutral_window_only(self):
        lines = [drive_row(i * 50, 0, 0) for i in range(10)]
        lines += [drive_row(i * 50, 5, 4) for i in range(10, 14)]
        lines += [drive_row(700, 0, -2), drive_row(750, 0, -7), drive_row(800, 0, -3)]
        lines += [drive_row(i * 50, -5, -20) for i in range(17, 25)]
        with tempfile.TemporaryDirectory() as directory:
            result = analyze(self.write_log(directory, lines))
        self.assertTrue(result["stop_available"])
        self.assertEqual(len(result["releases"]), 1)
        self.assertEqual(result["opposite_speed_peak"], 7)
        self.assertEqual(result["releases"][0]["command_peak"], 5)

    def test_neutral_ch3_and_tilt_do_not_pass_stop_gate(self):
        lines = [drive_row(i * 50, 0, 0, roll=-23) for i in range(24)]
        with tempfile.TemporaryDirectory() as directory:
            result = analyze(self.write_log(directory, lines))
        self.assertTrue(result["capture_ok"])
        self.assertFalse(result["stop_available"])
        self.assertEqual(result["command_rows"], 0)
        self.assertEqual(result["activation_roll_deg"], -23)

    def test_corrupted_capture_is_rejected(self):
        lines = [drive_row(i * 50, 0, 0) for i in range(20)] + ["DRIVE,broken"] * 3
        with tempfile.TemporaryDirectory() as directory:
            result = analyze(self.write_log(directory, lines))
        self.assertEqual(result["malformed_rows"], 3)
        self.assertFalse(result["capture_ok"])

    def test_timing_and_motor_measurements_are_reported(self):
        lines = []
        for index in range(24):
            extra = {}
            if index == 3:
                extra = dict(control_execution_us=1234, max_loop_entry_interval_us=2345,
                             motor1_current_sp=-0.21, motor2_voltage_q=0.37,
                             battery_raw_instant_v=7.25, imu_dt_s=0.0065)
            if index == 20:
                extra = dict(motor2_current_sp=0.15, motor1_voltage_q=-0.41)
            if index == 23:
                extra = dict(imu_long_gap_count=4, imu_max_interval_us=9100)
            lines.append(drive_row(index * 7, 0, 0, **extra))
        with tempfile.TemporaryDirectory() as directory:
            result = analyze(self.write_log(directory, lines))
        self.assertEqual(result["control_execution_us_peak"], 1234)
        self.assertEqual(result["loop_entry_interval_us_peak"], 2345)
        self.assertEqual(result["motor_current_sp_abs_peak"], 0.21)
        self.assertEqual(result["motor_voltage_q_abs_peak"], 0.41)
        self.assertEqual(result["battery_raw_instant_min_v"], 7.25)
        self.assertEqual(result["imu_long_gaps"], 4)
        self.assertEqual(result["imu_long_gap_fraction"], 0.0248)
        self.assertEqual(result["imu_max_interval_us"], 9100)
        self.assertEqual(result["imu_mean_interval_s"], 0.001)
        self.assertEqual(result["imu_rate_hz"], 1000.0)

    def test_imu_mean_interval_comes_from_read_time_not_row_phase(self):
        lines = []
        for index in range(24):
            lines.append(drive_row(index * 7, 0, 0, imu_sequence=index * 4, imu_accepted_samples=index * 4,
                                   imu_read_time_us=index * 9440 + 17, imu_dt_s=0.001845,
                                   imu_long_gap_count=0 if index < 23 else 5))
        with tempfile.TemporaryDirectory() as directory:
            result = analyze(self.write_log(directory, lines))
        self.assertEqual(result["imu_mean_interval_s"], 0.00236)
        self.assertEqual(result["imu_long_gaps"], 5)
        self.assertEqual(result["imu_long_gap_fraction"], 0.0543)

    def test_old_33_column_schema_is_rejected_without_fallback(self):
        lines = [f"DRIVE,{i * 50},1,7.9,0,0,0,0,2,-2,3" for i in range(24)]
        with tempfile.TemporaryDirectory() as directory:
            with self.assertRaisesRegex(ValueError, "fehlende DRIVE-Spalten"):
                analyze(self.write_log(directory, lines, header=OLD_HEADER))

    def test_short_row_is_malformed_not_parsed(self):
        lines = [drive_row(i * 50, 0, 0) for i in range(24)]
        lines.append(lines[0].rsplit(",", 1)[0])
        with tempfile.TemporaryDirectory() as directory:
            result = analyze(self.write_log(directory, lines))
        self.assertEqual(result["valid_rows"], 24)
        self.assertEqual(result["malformed_rows"], 1)

    def test_nonfinite_value_is_malformed_not_parsed(self):
        lines = [drive_row(i * 50, 0, 0) for i in range(24)]
        lines.append(drive_row(1200, 0, 0, imu_dt_s="nan"))
        with tempfile.TemporaryDirectory() as directory:
            result = analyze(self.write_log(directory, lines))
        self.assertEqual(result["valid_rows"], 24)
        self.assertEqual(result["malformed_rows"], 1)


if __name__ == "__main__":
    unittest.main()
