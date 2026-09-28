#!/usr/bin/env python3
"""Focused checks for DRIVE release detection and capture-quality gating."""

import tempfile
import unittest
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from analyze_drive_trace import analyze


HEADER = ("DRIVE,time_ms,ch5_mode,voltage_min_raw_v,ch3_target,"
          "motor1_velocity_f,motor2_velocity_f,roll_ok,motor1_target,motor2_target,angle_out_i")


def row(time_ms, command, speed, *, roll=0, ch5=1):
    return f"DRIVE,{time_ms},{ch5},7.9,{command},{speed},{speed},{roll},2,-2,3"


class DriveTraceTest(unittest.TestCase):
    def write_log(self, directory, lines):
        path = Path(directory) / "drive.log"
        path.write_text("# " + HEADER + "\n" + "\n".join(lines) + "\n", encoding="utf-8")
        return path

    def test_release_peak_uses_neutral_window_only(self):
        lines = [row(i * 50, 0, 0) for i in range(10)]
        lines += [row(i * 50, 5, 4) for i in range(10, 14)]
        lines += [row(700, 0, -2), row(750, 0, -7), row(800, 0, -3)]
        lines += [row(i * 50, -5, -20) for i in range(17, 25)]
        with tempfile.TemporaryDirectory() as directory:
            result = analyze(self.write_log(directory, lines))
        self.assertTrue(result["stop_available"])
        self.assertEqual(len(result["releases"]), 1)
        self.assertEqual(result["opposite_speed_peak"], 7)
        self.assertEqual(result["releases"][0]["command_peak"], 5)

    def test_neutral_ch3_and_tilt_do_not_pass_stop_gate(self):
        lines = [row(i * 50, 0, 0, roll=-23) for i in range(24)]
        with tempfile.TemporaryDirectory() as directory:
            result = analyze(self.write_log(directory, lines))
        self.assertTrue(result["capture_ok"])
        self.assertFalse(result["stop_available"])
        self.assertEqual(result["command_rows"], 0)
        self.assertEqual(result["activation_roll_deg"], -23)

    def test_corrupted_capture_is_rejected(self):
        lines = [row(i * 50, 0, 0) for i in range(20)] + ["DRIVE,broken"] * 3
        with tempfile.TemporaryDirectory() as directory:
            result = analyze(self.write_log(directory, lines))
        self.assertEqual(result["malformed_rows"], 3)
        self.assertFalse(result["capture_ok"])


if __name__ == "__main__":
    unittest.main()
