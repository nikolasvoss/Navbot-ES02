#!/usr/bin/env python3
"""Checks the turn-test summary: track width, left/right mapping and gyro sign."""

import math
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "scripts"))
from odometry_turn_test import summarize, unwrap_step


class SummarizeTest(unittest.TestCase):
    def test_wheel_yaw_too_large_means_track_width_too_small(self):
        result = summarize(3, math.radians(1188.0), math.radians(1083.6), 0.113)
        self.assertEqual(result["true_yaw_deg"], 1080.0)
        self.assertEqual(result["track_width_m"], 0.1243)
        self.assertEqual(result["left_right_mapping"], "ok")
        self.assertEqual(result["gyro_yaw_sign"], 1)
        self.assertEqual(result["gyro_scale"], 1.0033)

    def test_negative_wheel_yaw_on_ccw_turn_means_swapped_wheels(self):
        result = summarize(2, math.radians(-720.0), math.radians(-720.0), 0.113)
        self.assertEqual(result["left_right_mapping"], "swapped")
        self.assertEqual(result["gyro_yaw_sign"], -1)
        self.assertEqual(result["track_width_m"], 0.113)

    def test_clockwise_turns(self):
        result = summarize(-1, math.radians(-360.0), math.radians(360.0), 0.113)
        self.assertEqual(result["left_right_mapping"], "ok")
        self.assertEqual(result["gyro_yaw_sign"], -1)

    def test_unwrap_step_crosses_pi(self):
        self.assertAlmostEqual(unwrap_step(math.radians(179.0), math.radians(-179.0)), math.radians(2.0))


if __name__ == "__main__":
    unittest.main()
