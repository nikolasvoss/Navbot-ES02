#!/usr/bin/env python3
"""Whitelist for capture_balance_trace.py --interactive, including bounded T/M motor commands."""

import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "scripts"))
from capture_balance_trace import (MAX_CURRENT_A, MAX_LEASE_MS, MAX_VOLTAGE_V, interactive_command)


class InteractiveWhitelistTest(unittest.TestCase):
    def test_firmware_caps_are_mirrored(self):
        self.assertEqual(MAX_CURRENT_A, 0.35)
        self.assertEqual(MAX_VOLTAGE_V, 2.0)
        self.assertEqual(MAX_LEASE_MS, 5000)

    def test_bounded_current_commands_are_sent_uppercase_and_trimmed(self):
        self.assertEqual(interactive_command("T 0.01 -0.02 500\n"), "T 0.01 -0.02 500")
        self.assertEqual(interactive_command("t .02 -0.02 1"), "T .02 -0.02 1")
        self.assertEqual(interactive_command("T 0 0 2000"), "T 0 0 2000")
        self.assertEqual(interactive_command("T +0.01 0 100"), "T +0.01 0 100")

    def test_bounded_voltage_commands_are_accepted_for_either_wheel(self):
        self.assertEqual(interactive_command("M 1 0.5 2000"), "M 1 0.5 2000")
        self.assertEqual(interactive_command("m 2 -0.25 1"), "M 2 -0.25 1")
        self.assertEqual(interactive_command("M 2 0 10"), "M 2 0 10")
        self.assertEqual(interactive_command("M 1 2.0 5000"), "M 1 2.0 5000")
        self.assertEqual(interactive_command("M 2 -2 5000"), "M 2 -2 5000")

    def test_stop_and_reset_are_accepted_for_both_motion_commands(self):
        for command in ("T STOP", "T RESET", "M STOP", "M RESET", " m reset \r\n"):
            with self.subTest(command=command):
                self.assertEqual(interactive_command(command), command.strip().upper())

    def test_current_over_cap_is_rejected(self):
        for command in ("T 0.36 0 500", "T 0 -0.351 500"):
            with self.subTest(command=command):
                self.assertIsNone(interactive_command(command))

    def test_voltage_over_cap_is_rejected(self):
        for command in ("M 1 2.01 500", "M 2 -2.1 500"):
            with self.subTest(command=command):
                self.assertIsNone(interactive_command(command))

    def test_lease_must_be_whole_milliseconds_within_firmware_range(self):
        for command in ("T 0 0 0", "T 0 0 5001", "T 0 0 500.5", "T 0 0 -5", "M 1 0.1 0", "M 1 0.1 5001",
                        "M 1 0.1 1.0"):
            with self.subTest(command=command):
                self.assertIsNone(interactive_command(command))

    def test_grammar_is_strict(self):
        for command in ("T 0 0", "T 0 0 500 extra", "T  0 0 500", "T 0,0 500", "T 1e-1 0 100",
                        "T nan 0 100", "T inf 0 100", "M 3 0.1 100", "M 0 0.1 100", "M STOP now",
                        "T STOPX", "T", "M", "TSTOP", "T RESET 1"):
            with self.subTest(command=command):
                self.assertIsNone(interactive_command(command))

    def test_non_motor_commands_keep_the_existing_whitelist(self):
        self.assertEqual(interactive_command("K0"), "K0")
        self.assertEqual(interactive_command("K58"), "K58")
        self.assertEqual(interactive_command("yp0.04"), "YP0.04")
        self.assertIsNone(interactive_command("E1"))
        self.assertIsNone(interactive_command("K1"))
        self.assertIsNone(interactive_command(""))


if __name__ == "__main__":
    unittest.main()
