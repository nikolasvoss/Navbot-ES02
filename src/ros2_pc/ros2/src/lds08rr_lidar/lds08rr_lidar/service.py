from __future__ import annotations

import argparse
import socket
import sys
import threading
from pathlib import Path


def build_parser(include_ssh: bool = False):
    parser = argparse.ArgumentParser(description="Run the LDS08RR status page and optional ROS publisher.")
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument("--demo", action="store_true", help="show a synthetic scan without hardware")
    source.add_argument("--device", help="serial device, for example /dev/ttyAMA0")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--host", default="127.0.0.1", help="HTTP bind address; use 0.0.0.0 for LAN access")
    parser.add_argument("--port", type=int, default=8081)
    parser.add_argument("--ros", choices=("auto", "on", "off"), default="auto")
    if include_ssh:
        parser.add_argument("--ssh", metavar="HOST", help="run remotely and forward the LiDAR page")
        parser.add_argument("--remote-path", help="absolute project path on the remote host")
    return parser


def parse_args(argv=None):
    parser = build_parser()
    args = parser.parse_args(argv)
    if args.baud <= 0:
        parser.error("--baud must be positive")
    return args


def page_host(bind_host: str) -> str:
    if bind_host != "0.0.0.0":
        return bind_host
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as route_probe:
            route_probe.connect(("192.0.2.1", 9))
            return route_probe.getsockname()[0]
    except OSError:
        return "127.0.0.1"


def run(argv=None) -> int:
    args = parse_args(argv)
    from werkzeug.serving import make_server

    from .source import SourceConfig
    from .web import make_lidar_app

    config = SourceConfig(
        mode="demo" if args.demo else "serial",
        device=None if args.demo else Path(args.device),
        baud=args.baud,
    )
    runtime = None
    node = None
    rclpy = None
    shutdown_exceptions = (KeyboardInterrupt,)
    ros_requested = args.ros == "on"
    if args.ros != "off":
        try:
            import rclpy as ros_client
            from .ros_node import LidarNode

            rclpy = ros_client
        except ImportError as exc:
            if ros_requested:
                raise RuntimeError(f"ROS 2 is unavailable: {exc}") from exc
            print(f"ROS 2 unavailable, serving the browser only: {exc}", file=sys.stderr)
        else:
            from rclpy.executors import ExternalShutdownException

            shutdown_exceptions = (KeyboardInterrupt, ExternalShutdownException)
            rclpy.init()
            node = LidarNode(config)
            runtime = node.runtime
    if runtime is None:
        from .runtime import LidarRuntime

        runtime = LidarRuntime(config)
    try:
        server = make_server(args.host, args.port, make_lidar_app(runtime), threaded=True)
    except Exception:
        if node is not None:
            node.destroy_node()
            rclpy.try_shutdown()
        raise
    runtime.start()
    thread = threading.Thread(target=server.serve_forever, name="lds08rr-http", daemon=True)
    thread.start()
    url_host = page_host(args.host)
    print(f"LiDAR page: http://{url_host}:{args.port}/lidar", flush=True)
    if args.host == "0.0.0.0":
        print(f"HTTP listens on all interfaces at port {args.port}.", flush=True)
    try:
        if node is not None:
            rclpy.spin(node)
        else:
            threading.Event().wait()
    except shutdown_exceptions:
        pass
    finally:
        runtime.stop()
        server.shutdown()
        server.server_close()
        thread.join(timeout=2.0)
        if node is not None:
            node.destroy_node()
            rclpy.try_shutdown()
    return 0


def main() -> None:
    try:
        raise SystemExit(run())
    except OSError as exc:
        raise SystemExit(f"Could not start LiDAR service: {exc}") from exc
    except RuntimeError as exc:
        raise SystemExit(str(exc)) from exc
