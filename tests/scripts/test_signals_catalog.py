import importlib.util
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[2]
SPEC = importlib.util.spec_from_file_location("signals_catalog", ROOT / "scripts" / "signals_catalog.py")
signals_catalog = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(signals_catalog)

SAMPLE = """\
// A comment line.

SIGNAL    (speed,        "m/s", balanceOutput.diag.shapedSpeedMps)
SIGNAL_MIN(battery_min,  "V",   batteryInstantRawV)   // trailing comment
SIGNAL_MAX(loop_max_us,  "us",  loopEntryIntervalUs)
SIGNAL(flag, "", imu::latest().valid)
"""


def test_parse_lists_name_unit_and_kind_in_file_order():
    entries = signals_catalog.parse(SAMPLE)
    assert [(e.index, e.name, e.unit, e.kind) for e in entries] == [
        (0, "speed", "m/s", "SIGNAL"),
        (1, "battery_min", "V", "SIGNAL_MIN"),
        (2, "loop_max_us", "us", "SIGNAL_MAX"),
        (3, "flag", "", "SIGNAL"),
    ]


def test_parse_rejects_a_duplicate_name():
    with pytest.raises(signals_catalog.DefError, match="line 2: duplicate name speed"):
        signals_catalog.parse('SIGNAL(speed, "", a)\nSIGNAL_MAX(speed, "", b)\n')


@pytest.mark.parametrize("line", [
    'PROBE(speed, "", balanceSettings, speedKpRadPerMps)',  # a macro the tool does not know
    'SIGNAL(speed, m/s, a)',                                # unit without quotes
    'SIGNAL(speed, "m/s")',                                 # no expression
    'PARAM(speed_kp, "", balanceSettings)',                 # no field
    'ACTION_NO_ARGS(motor_reset)',                          # no handler
    'int x;',
])
def test_parse_stops_on_a_line_it_does_not_understand(line):
    with pytest.raises(signals_catalog.DefError, match="line 1: not an entry this tool knows"):
        signals_catalog.parse(line + "\n")


PARAM_AND_ACTION = """\
SIGNAL    (speed,        "m/s", a)
PARAM     (speed_kp,     "rad/(m/s)", BalanceGroup, speedKpRadPerMps)
ACTION    (motor_current, startMotorCurrent, ARG(left_a, -MotorDiagnostic::kMaximumCurrentA, MotorDiagnostic::kMaximumCurrentA), ARG(lease_ms, 1, MotorDiagnostic::kMaximumLeaseMs))
ACTION_NO_ARGS(motor_reset, resetMotorFault)
"""


def test_parse_lists_params_and_actions_with_their_kinds():
    entries = signals_catalog.parse(PARAM_AND_ACTION)
    assert [(e.index, e.name, e.unit, e.kind) for e in entries] == [
        (0, "speed", "m/s", "SIGNAL"),
        (1, "speed_kp", "rad/(m/s)", "PARAM"),
        (2, "motor_current", "", "ACTION"),
        (3, "motor_reset", "", "ACTION"),
    ]


def test_an_action_needs_its_arguments_in_arg_form():
    with pytest.raises(signals_catalog.DefError, match="line 1: not an ARG\\(name, min, max\\) list"):
        signals_catalog.parse('ACTION(go, startGo, ARG(left_a, 0), ARG(b, 1, 2))\n')
    with pytest.raises(signals_catalog.DefError, match="line 1: not an entry this tool knows"):
        signals_catalog.parse('ACTION(go, startGo, left_a)\n')


def test_a_duplicate_param_or_action_name_is_refused():
    with pytest.raises(signals_catalog.DefError, match="line 2: duplicate name go"):
        signals_catalog.parse('ACTION_NO_ARGS(go, startGo)\nACTION_NO_ARGS(go, startGo)\n')


def test_command_line_prints_tab_separated_rows(capsys, tmp_path):
    path = tmp_path / "entries.def.h"
    path.write_text(SAMPLE, encoding="utf-8")
    assert signals_catalog.main([str(path)]) == 0
    assert capsys.readouterr().out == (
        "0\tspeed\tm/s\tSIGNAL\n1\tbattery_min\tV\tSIGNAL_MIN\n2\tloop_max_us\tus\tSIGNAL_MAX\n3\tflag\t\tSIGNAL\n"
    )


def test_command_line_reports_a_bad_file(capsys, tmp_path):
    path = tmp_path / "entries.def.h"
    path.write_text("nonsense\n", encoding="utf-8")
    assert signals_catalog.main([str(path)]) == 1
    assert "not an entry this tool knows: nonsense" in capsys.readouterr().err


def test_the_firmware_file_parses():
    entries = signals_catalog.parse(signals_catalog.DEFAULT_FILE.read_text(encoding="utf-8"))
    assert entries[0].name == "ch5_mode"
    assert len(entries) == len({e.name for e in entries})

