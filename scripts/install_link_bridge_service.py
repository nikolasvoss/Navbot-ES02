#!/usr/bin/env python3
from __future__ import annotations

import argparse
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


def render_unit(heading_source: str = "wheels", gyro_yaw_sign: int | None = None) -> str:
    uid, gid = os.getuid(), os.getgid()
    required = (TEMPLATE, ROS_SETUP, WORKSPACE_SETUP)
    missing = [str(path) for path in required if not path.is_file()]
    if missing:
        raise FileNotFoundError(
            "Required link bridge startup files are missing (build the package first, see "
            "docs/integration/software/navbot-link-bridge.md): " + ", ".join(missing)
        )

    heading_args = []
    if heading_source == "gyro":
        if gyro_yaw_sign not in (1, -1):
            raise ValueError("--heading-source gyro requires --gyro-yaw-sign 1 or -1")
        heading_args = ["-p", "heading_source:=gyro", "-p", f"gyro_yaw_sign:={gyro_yaw_sign}"]
    elif heading_source != "wheels":
        raise ValueError(f"unknown heading source {heading_source!r}")

    shell_command = (
        f"source {shlex.quote(str(ROS_SETUP))} && source {shlex.quote(str(WORKSPACE_SETUP))} && exec "
        + shlex.join([
            "ros2", "run", "navbot_link_bridge", "navbot_link_bridge", "--ros-args",
            "-p", f"port:={LINK_PORT}",
            "-p", f"wheel_radius_m:={WHEEL_RADIUS_M}",
            "-p", f"wheel_separation_m:={WHEEL_SEPARATION_M}",
            *heading_args,
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
    parser = argparse.ArgumentParser(description="Install and start the navbot_link_bridge systemd service.")
    parser.add_argument("--heading-source", choices=("wheels", "gyro"), default="wheels",
                        help="yaw from wheel differential (default) or from the IMU gyro")
    parser.add_argument("--gyro-yaw-sign", type=int, choices=(1, -1),
                        help="required with --heading-source gyro: maps IMU gz to ROS +yaw (CCW from above)")
    args = parser.parse_args()
    if args.heading_source == "gyro" and args.gyro_yaw_sign is None:
        parser.error("--heading-source gyro requires --gyro-yaw-sign 1 or -1")
    if os.getuid() == 0:
        print("Run this installer as the link bridge service user, without sudo.", file=sys.stderr)
        return 2

    try:
        unit = render_unit(args.heading_source, args.gyro_yaw_sign)
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
