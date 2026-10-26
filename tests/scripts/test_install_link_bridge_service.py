from __future__ import annotations

import importlib.util
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[2]
SCRIPTS = ROOT / "scripts"
sys.path.insert(0, str(SCRIPTS))
SPEC = importlib.util.spec_from_file_location("install_link_bridge_service", SCRIPTS / "install_link_bridge_service.py")
INSTALLER = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(INSTALLER)


def test_unit_runs_the_bridge_with_port_and_measured_wheel_geometry(tmp_path, monkeypatch):
    ros_setup = tmp_path / "ros_setup.bash"
    workspace_setup = tmp_path / "workspace_setup.bash"
    ros_setup.write_text("")
    workspace_setup.write_text("")
    monkeypatch.setattr(INSTALLER, "ROS_SETUP", ros_setup)
    monkeypatch.setattr(INSTALLER, "WORKSPACE_SETUP", workspace_setup)

    unit = INSTALLER.render_unit()

    assert "SupplementaryGroups=dialout" in unit
    assert "@" not in unit
    for expected in ("port:=/dev/ttyAMA3", "wheel_radius_m:=0.01915", "wheel_separation_m:=0.113"):
        assert expected in unit
    assert f"source {ros_setup}" in unit and f"source {workspace_setup}" in unit


def test_unit_refuses_to_render_when_the_package_is_not_built(tmp_path, monkeypatch):
    monkeypatch.setattr(INSTALLER, "WORKSPACE_SETUP", tmp_path / "missing.bash")
    with pytest.raises(FileNotFoundError, match="build the package first"):
        INSTALLER.render_unit()
