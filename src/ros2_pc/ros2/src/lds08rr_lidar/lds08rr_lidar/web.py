from __future__ import annotations

from lds2d.viz import ScanBuffer


class FreshScanBuffer:
    def __init__(self, runtime) -> None:
        self.runtime = runtime
        self.buffer = runtime.scan_buffer
        self.empty = ScanBuffer().latest()

    def latest(self) -> dict:
        status = self.runtime.snapshot()
        if status.state not in ("live", "demo") or status.scans_received == 0:
            return self.empty
        return self.buffer.latest()


def make_lidar_app(runtime):
    from flask import Flask, jsonify, render_template

    app = Flask(__name__)
    scan_buffer = FreshScanBuffer(runtime)

    @app.get("/")
    @app.get("/lidar")
    def lidar_page():
        return render_template("lidar.html")

    @app.get("/scan.json")
    def scan_json():
        return jsonify(scan_buffer.latest())

    @app.get("/status.json")
    def status_json():
        return jsonify(runtime.status_dict())

    return app
