#!/usr/bin/env python3
from __future__ import annotations

import os
import shlex
import subprocess
import sys
import tempfile
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
TEMPLATE = REPO_ROOT / "src/ros2_pc/systemd/navbot-lidar.service.in"
UNIT_PATH = "/etc/systemd/system/navbot-lidar.service"
ROS_SETUP = Path("/opt/ros/jazzy/setup.bash")


def systemd_quote(value: str) -> str:
    if "\n" in value or "\r" in value:
        raise ValueError("Paths containing newlines cannot be used in a systemd unit")
    escaped = value.replace("%", "%%").replace("$", "$$").replace("\\", "\\\\").replace('"', '\\"')
    return f'"{escaped}"'


def render_unit() -> str:
    uid, gid = os.getuid(), os.getgid()
    python = Path.home() / ".venvs/navbot-lidar/bin/python"
    launcher = REPO_ROOT / "scripts/start_lidar.py"
    required = (TEMPLATE, ROS_SETUP, python, launcher)
    missing = [str(path) for path in required if not path.is_file()]
    if missing:
        raise FileNotFoundError("Required LiDAR startup files are missing: " + ", ".join(missing))

    shell_command = (
        f"source {shlex.quote(str(ROS_SETUP))} && exec "
        + shlex.join([
            str(python), str(launcher), "--device", "/dev/ttyAMA0",
            "--ros", "on", "--host", "0.0.0.0",
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
        print("Usage: python3 scripts/install_lidar_service.py", file=sys.stderr)
        return 2
    if os.getuid() == 0:
        print("Run this installer as the LiDAR service user, without sudo.", file=sys.stderr)
        return 2

    try:
        unit = render_unit()
        with tempfile.NamedTemporaryFile(mode="w", encoding="utf-8") as temporary:
            temporary.write(unit)
            temporary.flush()
            subprocess.run(["sudo", "install", "-m", "644", temporary.name, UNIT_PATH], check=True)
        subprocess.run(["sudo", "systemctl", "daemon-reload"], check=True)
        subprocess.run(["sudo", "systemctl", "enable", "navbot-lidar.service"], check=True)
        subprocess.run(["sudo", "systemctl", "restart", "navbot-lidar.service"], check=True)
    except (FileNotFoundError, ValueError, subprocess.CalledProcessError) as error:
        print(f"LiDAR service installation failed: {error}", file=sys.stderr)
        return 1

    print("LiDAR service enabled and restarted.")
    print("Check with: sudo systemctl status navbot-lidar.service")
    print("Page: http://<ros2_pc-host>:8081/lidar")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
