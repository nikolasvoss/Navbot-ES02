"""The shipped presets against the firmware's entry table."""

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

from navbot_link.capture import load_presets  # noqa: E402
from navbot_link.protocol import Catalog, Entry, Kind, Type, UnknownName  # noqa: E402

PRESETS = load_presets(ROOT / "scripts" / "navlink_presets.yaml")
DEF_TEXT = signals_catalog.DEFAULT_FILE.read_text(encoding="utf-8")
MAX_ENTRIES = 48  # one subscription row has to fit one frame


def catalog_of(def_text):
    entries = signals_catalog.parse(def_text)
    # The expected catalog knows names, units and kinds. Types do not matter for resolving a name.
    return Catalog("expected", tuple(Entry(e.index, e.name, e.unit, Kind[e.kind], Type.F32) for e in entries))


@pytest.mark.parametrize("name", sorted(PRESETS))
def test_every_preset_resolves_against_the_firmware_catalog(name):
    catalog = catalog_of(DEF_TEXT)
    signals = PRESETS[name].signals
    for signal in signals:
        catalog.signal(signal)
    assert len(signals) == len(set(signals)), "a repeated name would give two CSV columns the same header"
    assert len(signals) <= MAX_ENTRIES


def test_a_signal_renamed_in_the_firmware_breaks_the_presets_that_use_it():
    drifted = DEF_TEXT.replace("(roll_deg,", "(roll_degrees,")
    assert drifted != DEF_TEXT
    catalog = catalog_of(drifted)
    with pytest.raises(UnknownName, match="unknown signal roll_deg"):
        for signal in PRESETS["bal"].signals:
            catalog.signal(signal)
