#!/usr/bin/env python3
from __future__ import annotations

import os
import shlex
import subprocess
import sys
import tempfile
from pathlib import Path

from install_lidar_service import ROS_SETUP, systemd_quote

REPO_ROOT = Path(__file__).resolve().parents[1]
TEMPLATE = REPO_ROOT / "src/ros2_pc/systemd/navbot-link-bridge.service.in"
WORKSPACE_SETUP = REPO_ROOT / "src/ros2_pc/ros2/install/setup.bash"
UNIT_PATH = "/etc/systemd/system/navbot-link-bridge.service"
LINK_PORT = "/dev/ttyAMA3"
WHEEL_RADIUS_M = "0.01915"  # 38.3 mm wheel diameter
WHEEL_SEPARATION_M = "0.113"  # measured track width


def render_unit() -> str:
    uid, gid = os.getuid(), os.getgid()
    required = (TEMPLATE, ROS_SETUP, WORKSPACE_SETUP)
    missing = [str(path) for path in required if not path.is_file()]
    if missing:
        raise FileNotFoundError(
            "Required link bridge startup files are missing (build the package first, see "
            "docs/integration/software/navbot-link-bridge.md): " + ", ".join(missing)
        )

    shell_command = (
        f"source {shlex.quote(str(ROS_SETUP))} && source {shlex.quote(str(WORKSPACE_SETUP))} && exec "
        + shlex.join([
            "ros2", "run", "navbot_link_bridge", "navbot_link_bridge", "--ros-args",
            "-p", f"port:={LINK_PORT}",
            "-p", f"wheel_radius_m:={WHEEL_RADIUS_M}",
            "-p", f"wheel_separation_m:={WHEEL_SEPARATION_M}",
        ])
    )
    values = {
        "@UID@": str(uid),
        "@GID@": str(gid),
        "@EXEC_START@": "/bin/bash -c " + systemd_quote(shell_command),
    }
    rendered = TEMPLATE.read_text()
    for placeholder, value in values.items():
        rendered = rendered.replace(placeholder, value)
    return rendered


def main() -> int:
    if len(sys.argv) != 1:
        print("Usage: python3 scripts/install_link_bridge_service.py", file=sys.stderr)
        return 2
    if os.getuid() == 0:
        print("Run this installer as the link bridge service user, without sudo.", file=sys.stderr)
        return 2

    try:
        unit = render_unit()
        with tempfile.NamedTemporaryFile(mode="w", encoding="utf-8") as temporary:
            temporary.write(unit)
            temporary.flush()
            subprocess.run(["sudo", "install", "-m", "644", temporary.name, UNIT_PATH], check=True)
        subprocess.run(["sudo", "systemctl", "daemon-reload"], check=True)
        subprocess.run(["sudo", "systemctl", "enable", "navbot-link-bridge.service"], check=True)
        subprocess.run(["sudo", "systemctl", "restart", "navbot-link-bridge.service"], check=True)
    except (FileNotFoundError, ValueError, subprocess.CalledProcessError) as error:
        print(f"Link bridge service installation failed: {error}", file=sys.stderr)
        return 1

    print("Link bridge service enabled and restarted.")
    print("Check with: sudo systemctl status navbot-link-bridge.service")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
