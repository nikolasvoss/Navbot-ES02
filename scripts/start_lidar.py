#!/usr/bin/env python3
from __future__ import annotations

import shlex
import subprocess
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
PACKAGE_ROOT = REPO_ROOT / "src/ros2_pc/ros2/src/lds08rr_lidar"
sys.path.insert(0, str(PACKAGE_ROOT))

from lds08rr_lidar.service import build_parser, parse_args, run


def ssh_invocation(host: str, remote_path: str, service_args: list[str]) -> tuple[list[str], str]:
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
    ssh = ["ssh", "-o", "ExitOnForwardFailure=yes", "-L", f"{args.port}:127.0.0.1:{args.port}", "--", host, command]
    return ssh, f"http://127.0.0.1:{args.port}/lidar"


def main(argv=None) -> int:
    values = list(sys.argv[1:] if argv is None else argv)
    parser = build_parser(include_ssh=True)
    options = parser.parse_args(values)
    service_args = ["--demo"] if options.demo else ["--device", options.device]
    if options.protocol is not None:
        service_args.extend(["--protocol", options.protocol])
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
        command, url = ssh_invocation(options.ssh, options.remote_path, service_args)
    except ValueError as exc:
        parser.error(str(exc))
    print(f"LiDAR page through SSH tunnel: {url}", flush=True)
    return subprocess.run(command, check=False).returncode


if __name__ == "__main__":
    raise SystemExit(main())
