from __future__ import annotations

from pathlib import Path

from lds2d.viz import ScanBuffer, make_app


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
    from flask import jsonify, render_template

    app = make_app(FreshScanBuffer(runtime), model="Navbot LDS08RR", rotation="ccw")
    template_root = Path(__file__).parent / "templates"
    app.template_folder = str(template_root)

    @app.get("/lidar")
    def lidar_page():
        return render_template("lidar.html")

    @app.get("/status.json")
    def status_json():
        return jsonify(runtime.status_dict())

    return app
