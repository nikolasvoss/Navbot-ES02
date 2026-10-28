"""The shipped presets against the firmware's entry table and the K55 to K58 log profiles."""

import importlib.util
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "src"))


def load(name):
    spec = importlib.util.spec_from_file_location(name, ROOT / "scripts" / f"{name}.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


signals_catalog = load("signals_catalog")
capture_balance_trace = load("capture_balance_trace")

from navbot_link.capture import load_presets  # noqa: E402
from navbot_link.protocol import Catalog, Entry, Kind, Type, UnknownName  # noqa: E402

PRESETS = load_presets(ROOT / "scripts" / "navlink_presets.yaml")
DEF_TEXT = signals_catalog.DEFAULT_FILE.read_text(encoding="utf-8")

# Presets for signals the firmware does not have yet. They resolve only against tests/navbot_link/fake_firmware.py.
PENDING = {"speed_loop"}
MAX_ENTRIES = 48  # one subscription row has to fit one frame

K_PRESET = {55: "trace", 56: "ctrl", 57: "bal", 58: "drive"}

# A K column that the link replaces by t_us and its frame numbers, or that repeats another column.
NOT_A_SIGNAL = {
    "time_ms", "sequence", "logger_dropped", "uart_write_failures",
    "battery_filtered_instant_v",  # the same product as voltage_filtered_v
}
# roll_ok is roll_deg in the other profiles; max_loop_entry_interval_us has 26 characters, 2 over the catalog limit.
RENAMED = {
    "roll_ok": "roll_deg",
    "max_loop_entry_interval_us": "loop_interval_max_us",
}
# The K range columns are the highest minus the lowest servo command since the previous row. The servos all take
# the same leg pose, so its running minimum and maximum carry the same information.
LEG_POSE = ["leg_pose_min_deg", "leg_pose_max_deg"]
RANGE = {
    "servo1_range_deg": LEG_POSE, "servo2_range_deg": [], "servo3_range_deg": [], "servo4_range_deg": [],
    "max_servo_range_deg": LEG_POSE,
}


def catalog_of(def_text):
    entries = signals_catalog.parse(def_text)
    # The expected catalog knows names, units and kinds. Types do not matter for resolving a name.
    return Catalog("expected", tuple(Entry(e.index, e.name, e.unit, Kind[e.kind], Type.F32) for e in entries))


def k_columns(selector):
    header = capture_balance_trace.TRACE_HEADERS[selector]
    return header.split(",")[1:]


def signals_for(columns):
    names = []
    for column in columns:
        if column in NOT_A_SIGNAL:
            continue
        names += RANGE.get(column) if column in RANGE else [RENAMED.get(column, column)]
    return names


@pytest.mark.parametrize("name", sorted(set(PRESETS) - PENDING))
def test_every_preset_resolves_against_the_firmware_catalog(name):
    catalog = catalog_of(DEF_TEXT)
    signals = PRESETS[name].signals
    for signal in signals:
        catalog.signal(signal)
    assert len(signals) == len(set(signals)), "a repeated name would give two CSV columns the same header"
    assert len(signals) <= MAX_ENTRIES


@pytest.mark.parametrize("name", sorted(PENDING))
def test_pending_presets_still_name_signals_the_firmware_lacks(name):
    catalog = catalog_of(DEF_TEXT)
    missing = []
    for signal in PRESETS[name].signals:
        try:
            catalog.signal(signal)
        except UnknownName:
            missing.append(signal)
    assert missing, f"{name} now resolves against the firmware: remove it from PENDING"


@pytest.mark.parametrize("selector", sorted(K_PRESET))
def test_preset_has_the_columns_of_its_k_profile(selector):
    assert list(PRESETS[K_PRESET[selector]].signals) == signals_for(k_columns(selector))


def test_a_signal_renamed_in_the_firmware_breaks_the_presets_that_use_it():
    drifted = DEF_TEXT.replace("(roll_deg,", "(roll_degrees,")
    assert drifted != DEF_TEXT
    catalog = catalog_of(drifted)
    with pytest.raises(UnknownName, match="unknown signal roll_deg"):
        for signal in PRESETS["bal"].signals:
            catalog.signal(signal)


def test_the_rate_of_a_k_preset_is_the_k_row_rate():
    assert {PRESETS[name].rate_hz for name in K_PRESET.values()} == {100.0}
