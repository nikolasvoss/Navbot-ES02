"""Fixtures for the host simulation: the batch binary and a helper that runs a scenario."""

import subprocess
import tempfile
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[2]


@pytest.fixture(scope="session")
def sim_binary(tmp_path_factory):
    folder = tmp_path_factory.mktemp("navbot_sim")
    subprocess.run([str(REPO / "src/sim/build.sh"), str(folder)], check=True)
    return folder / "navbot_sim"


def run(binary, scenario_text, *sets):
    """Runs one scenario in batch mode; returns the CompletedProcess (stdout is the CSV)."""
    with tempfile.TemporaryDirectory() as folder:
        scenario = Path(folder) / "scenario.txt"
        scenario.write_text(scenario_text)
        command = [str(binary), "--scenario", str(scenario)]
        for item in sets:
            command += ["--set", item]
        return subprocess.run(command, capture_output=True, text=True)


def run_sim(binary, scenario_text, *sets):
    """Runs a scenario that must succeed. Returns (rows, stderr); rows are dicts of floats, in CSV column order."""
    result = run(binary, scenario_text, *sets)
    assert result.returncode == 0, result.stderr
    lines = result.stdout.splitlines()
    header = lines[0].split(",")
    rows = [dict(zip(header, map(float, line.split(",")))) for line in lines[1:]]
    return rows, result.stderr
