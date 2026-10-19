#!/usr/bin/env python3
from __future__ import annotations

import shlex
import socket
import subprocess
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
PACKAGE_ROOT = REPO_ROOT / "src/ros2_pc/ros2/src/lds08rr_lidar"
sys.path.insert(0, str(PACKAGE_ROOT))

from lds08rr_lidar.service import build_parser, parse_args, run

RADAR_BRIDGE_PORT = 9090


def local_port_free(port: int) -> bool:
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as probe:
        try:
            probe.bind(("127.0.0.1", port))
        except OSError:
            return False
    return True


def ssh_invocation(
    host: str, remote_path: str, service_args: list[str], forward_radar: bool = False
) -> tuple[list[str], str]:
    args = parse_args(service_args)
    if args.host not in ("127.0.0.1", "localhost"):
        raise ValueError("SSH mode binds the remote service to loopback for the SSH tunnel")
    remote_args = list(service_args)
    for index, value in enumerate(remote_args):
        if value == "--host":
            del remote_args[index:index + 2]
            break
        if value.startswith("--host="):
            del remote_args[index]
            break
    remote_args.extend(["--host", "127.0.0.1"])
    remote_python = '"$HOME/.venvs/navbot-lidar/bin/python"'
    remote_shell = (
        f"cd {shlex.quote(remote_path)} && source /opt/ros/jazzy/setup.bash "
        f"&& exec {remote_python} {shlex.join(['scripts/start_lidar.py', *remote_args])}"
    )
    command = f"bash -lc {shlex.quote(remote_shell)}"
    ssh = ["ssh", "-o", "ExitOnForwardFailure=yes", "-L", f"{args.port}:127.0.0.1:{args.port}"]
    if forward_radar:
        # The radar tab connects the browser to rosbridge on 127.0.0.1:9090 of this machine.
        ssh.extend(["-L", f"{RADAR_BRIDGE_PORT}:127.0.0.1:{RADAR_BRIDGE_PORT}"])
    ssh.extend(["--", host, command])
    return ssh, f"http://127.0.0.1:{args.port}/lidar"


def main(argv=None) -> int:
    values = list(sys.argv[1:] if argv is None else argv)
    parser = build_parser(include_ssh=True)
    options = parser.parse_args(values)
    service_args = ["--demo"] if options.demo else ["--device", options.device]
    service_args.extend([
        "--baud", str(options.baud), "--host", options.host,
        "--port", str(options.port), "--ros", options.ros,
    ])
    if options.ssh is None:
        if options.remote_path is not None:
            parser.error("--remote-path requires --ssh")
        return run(service_args)
    if not options.ssh:
        parser.error("--ssh must be a host name or user@host")
    if options.ssh.startswith("-"):
        parser.error("--ssh must be a host name or user@host")
    if not options.remote_path:
        parser.error("--ssh requires --remote-path")
    try:
        # An HMMD tunnel may already forward the bridge port; then the forward is skipped.
        command, url = ssh_invocation(
            options.ssh, options.remote_path, service_args,
            forward_radar=local_port_free(RADAR_BRIDGE_PORT),
        )
    except ValueError as exc:
        parser.error(str(exc))
    print(f"LiDAR page through SSH tunnel: {url}", flush=True)
    return subprocess.run(command, check=False).returncode


if __name__ == "__main__":
    raise SystemExit(main())
