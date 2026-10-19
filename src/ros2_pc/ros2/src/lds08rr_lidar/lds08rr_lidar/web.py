from __future__ import annotations

from pathlib import Path

from lds2d.viz import ScanBuffer

from .slam import SlamState

DEFAULT_RADAR_WEB = Path(__file__).resolve().parents[2] / "hmmd_radar" / "web"


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


def make_lidar_app(runtime, slam: SlamState | None = None, control=None, radar_web: Path | None = DEFAULT_RADAR_WEB):
    from flask import Flask, abort, jsonify, render_template, send_from_directory

    app = Flask(__name__)
    scan_buffer = FreshScanBuffer(runtime)
    slam = slam if slam is not None else SlamState()
    radar_dir = radar_web if radar_web is not None and (radar_web / "index.html").is_file() else None

    @app.get("/")
    @app.get("/lidar")
    def lidar_page():
        return render_template("lidar.html", active="lidar")

    @app.get("/slam")
    def slam_page():
        return render_template("slam.html", active="slam")

    @app.get("/map.json")
    def map_json():
        return jsonify({**slam.snapshot(), "control": control.status() if control is not None else None})

    @app.post("/slam/save")
    def slam_save():
        if control is None:
            return jsonify(ok=False, error="SLAM is not running on this service"), 409
        if slam.snapshot()["map"] is None:
            return jsonify(ok=False, error="No map to save yet"), 409
        error = control.save()
        if error is not None:
            return jsonify(ok=False, error=error), 500
        return jsonify(ok=True)

    @app.post("/slam/delete")
    def slam_delete():
        if control is None:
            return jsonify(ok=False, error="SLAM is not running on this service"), 409
        control.delete()
        slam.clear()
        return jsonify(ok=True)

    @app.get("/radar/")
    def radar_page():
        # The radar page is served unchanged; only the hub navigation is injected.
        if radar_dir is None:
            abort(404)
        html = (radar_dir / "index.html").read_text(encoding="utf-8")
        nav = render_template("_nav.html", active="radar")
        return html.replace("<body>", "<body>" + nav, 1)

    @app.get("/radar/<path:name>")
    def radar_file(name):
        if radar_dir is None:
            abort(404)
        return send_from_directory(radar_dir, name)

    @app.get("/scan.json")
    def scan_json():
        return jsonify(scan_buffer.latest())

    @app.get("/status.json")
    def status_json():
        return jsonify(runtime.status_dict())

    return app
