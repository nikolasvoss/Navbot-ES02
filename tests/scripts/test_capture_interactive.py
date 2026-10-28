import importlib.util
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
SPEC = importlib.util.spec_from_file_location("capture_balance_trace", ROOT / "scripts" / "capture_balance_trace.py")
capture_balance_trace = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(capture_balance_trace)
interactive_command = capture_balance_trace.interactive_command


class InteractiveCommandTest(unittest.TestCase):
    def test_trace_selections_pass_in_either_case_with_line_ends(self):
        self.assertEqual(interactive_command("K0"), "K0")
        self.assertEqual(interactive_command("k58\r\n"), "K58")
        self.assertEqual(interactive_command(" K55 "), "K55")
        self.assertEqual(interactive_command("K56"), "K56")
        self.assertEqual(interactive_command("K57"), "K57")

    def test_motor_letters_are_gone(self):
        # Motor tests are navlink actions now (motor_current, motor_voltage, motor_stop, motor_reset). The letter
        # commands are not forwarded from here, so no motor command can reach the wheels through this script.
        for command in ("T 0.01 -0.02 500", "M 1 0.5 2000", "T STOP", "M RESET", "t .02 -0.02 1", "M 2 0 10"):
            with self.subTest(command=command):
                self.assertIsNone(interactive_command(command))

    def test_other_selections_and_the_empty_line_are_refused(self):
        for command in ("K9", "K1", "K", "KI", "YP0.04", "E1", "Q0.4", "A1.5", ""):
            with self.subTest(command=command):
                self.assertIsNone(interactive_command(command))


if __name__ == "__main__":
    unittest.main()
