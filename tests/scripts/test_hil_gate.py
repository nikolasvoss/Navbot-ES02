"""The HIL gate decision and the skip rules of tests/conftest.py.

The skip rules run in a generated test file under pytester, with the real tests/conftest.py copied in.
"""

import importlib.util
from pathlib import Path
from types import SimpleNamespace

ROOT = Path(__file__).resolve().parents[2]
pytest_plugins = ["pytester"]


def load_hil_conftest():
    spec = importlib.util.spec_from_file_location("hil_conftest", ROOT / "tests" / "hil" / "conftest.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def item(*markers):
    return SimpleNamespace(keywords={name: True for name in markers})


def test_gate_runs_when_not_guided_even_if_every_test_is_guided():
    hil = load_hil_conftest()
    assert hil.gate_needed([item("guided")], guided=False) is True


def test_gate_is_skipped_when_guided_and_every_test_is_guided():
    hil = load_hil_conftest()
    assert hil.gate_needed([item("guided", "robot"), item("guided")], guided=True) is False


def test_gate_runs_when_guided_but_an_unguided_test_is_collected():
    hil = load_hil_conftest()
    assert hil.gate_needed([item("guided"), item("robot")], guided=True) is True


def test_gate_runs_for_unattended_tests_in_a_guided_session():
    hil = load_hil_conftest()
    assert hil.gate_needed([item("robot")], guided=True) is True


def test_pytester_skips_guided_without_guided_and_robot_without_robot(pytester):
    pytester.makeconftest((ROOT / "tests" / "conftest.py").read_text())
    pytester.makepyfile(
        test_areas="""
        import pytest

        @pytest.mark.guided
        def test_guided():
            pass

        @pytest.mark.robot
        def test_robot():
            pass

        def test_plain():
            pass
        """
    )
    result = pytester.runpytest("-rs")
    result.assert_outcomes(passed=1, skipped=2)
    result.stdout.fnmatch_lines(["*guided test: run with --guided*", "*needs a robot: run with --robot PORT*"])


def test_pytester_runs_guided_with_guided_and_robot_with_robot(pytester):
    pytester.makeconftest((ROOT / "tests" / "conftest.py").read_text())
    pytester.makepyfile(
        test_areas="""
        import pytest

        @pytest.mark.guided
        def test_guided():
            pass

        @pytest.mark.robot
        def test_robot():
            pass
        """
    )
    result = pytester.runpytest("--guided", "--robot", "fake")
    result.assert_outcomes(passed=2)
