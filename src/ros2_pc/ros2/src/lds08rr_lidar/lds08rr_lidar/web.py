from __future__ import annotations

from pathlib import Path

from lds2d.viz import ScanBuffer

from .health import ServiceMonitor, lidar_view
from .mount import robot_bearing_deg
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
        scan = self.buffer.latest()
        # The page plots bearings from the robot's front, not from the sensor's own zero.
        points = [{**point, "angle": robot_bearing_deg(point["angle"])} for point in scan["points"]]
        return {**scan, "points": points}


def make_lidar_app(
    runtime, slam: SlamState | None = None, control=None, radar_web: Path | None = DEFAULT_RADAR_WEB, tilt_gate=None, ros=None,
):
    from flask import Flask, abort, jsonify, render_template, request, send_from_directory
    from urllib.parse import urlsplit

    app = Flask(__name__)

    @app.before_request
    def refuse_cross_site_posts():
        # A browser sends Origin on every cross-site POST; scripts without one (curl) pass.
        if request.method != "POST":
            return None
        origin = request.headers.get("Origin")
        if request.headers.get("Sec-Fetch-Site") == "cross-site" or (
            origin is not None and urlsplit(origin).netloc != request.host
        ):
            return jsonify(ok=False, error="Cross-site request refused"), 403
        return None
    scan_buffer = FreshScanBuffer(runtime)
    slam = slam if slam is not None else SlamState()
    monitor = ServiceMonitor(runtime, control=control, ros=ros, slam=slam)
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
        status = control.status() if control is not None else None
        return jsonify({
            **slam.snapshot(),
            "control": status,
            "mode": None if status is None else status["mode"],
            "localization": None if status is None else status["localization"],
            "tilt_gate": tilt_gate.snapshot() if tilt_gate is not None else None,
        })

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

    @app.post("/slam/reset")
    def slam_reset():
        if control is None:
            return jsonify(ok=False, error="SLAM is not running on this service"), 409
        error = control.reload()
        if error is not None:
            return jsonify(ok=False, error=error), 409
        slam.clear()
        return jsonify(ok=True)

    @app.post("/slam/mode")
    def slam_mode():
        if control is None:
            return jsonify(ok=False, error="SLAM is not running on this service"), 409
        body = request.get_json(silent=True)
        error = control.set_mode(body.get("mode") if isinstance(body, dict) else None)
        if error is not None:
            return jsonify(ok=False, error=error), 409
        return jsonify(ok=True)

    @app.post("/slam/locate")
    def slam_locate():
        if control is None:
            return jsonify(ok=False, error="SLAM is not running on this service"), 409
        error = control.locate()
        if error is not None:
            return jsonify(ok=False, error=error), 409
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
        status = runtime.status_dict()
        level, label = lidar_view(status["state"])
        return jsonify({**status, "level": level, "label": label})

    @app.get("/services.json")
    def services_json():
        return jsonify(monitor.snapshot())

    return app
