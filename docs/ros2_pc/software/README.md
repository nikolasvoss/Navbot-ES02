# ros2_pc-Software

This directory documents software that runs on the ros2_pc, including its operating system, ROS 2 packages, services, applications, and deployment steps.

The [ROS 2 roadmap](../ROS_ROADMAP.md) tracks future work and live acceptance status.

- [HMMD ROS 2 developer guide](hmmd-ros2.md) for the current build, startup, browser, topic, and recording workflows.

The HMMD ROS 2 workspace is under `src/ros2_pc/ros2/`.

## Web hub and SLAM

The LiDAR service on port 8081 is the single browser entry point with three tabs: `/lidar`, `/slam`, and `/radar/`. The radar tab serves the unchanged `hmmd_radar/web` page from the same server. It still needs the HMMD launcher's rosbridge on `ws://127.0.0.1:9090` and shows a connecting state while no radar is installed. Radar and SLAM stay optional; the LiDAR page works without them.

SLAM starts with the LiDAR service when ROS 2 is active and `slam_toolbox` is installed (`sudo apt install ros-jazzy-slam-toolbox`). No second terminal is needed. The service supervises `slam_toolbox`, restarts it when it exits, and shows SLAM as unavailable when the package is missing. Parameters are in `src/ros2_pc/ros2/src/lds08rr_lidar/lds08rr_lidar/slam_toolbox.yaml`.

The service keeps one map in `~/.local/share/navbot/slam/` (`map.posegraph` and `map.data`). Every start loads that map when it exists and mapping continues on it, assuming the robot starts where the saved map began. On the `/slam` tab, **Save map** overwrites the saved map with the current one. **Delete map and start over** asks for confirmation, removes the saved map, and restarts mapping from an empty map. The SLAM log is `~/.local/share/navbot/slam/slam.log`.

The `odom` frame is fixed to the `lds08rr` sensor frame because the robot has no wheel odometry yet. Scan matching alone tracks motion, so move the sensor slowly (well under 0.5 m between scans). The `/slam` tab shows `/map`, the pose from TF `map` → `lds08rr`, the process state, and the saved map. It reports "SLAM not updating" when the map is older than 10 s. The map fits the window; **+** and **−**, the mouse wheel, or a two-finger pinch zoom, **Fit** shows the whole map again, and **Center arrow** keeps the pose arrow in the middle. With centering off, drag the map with the left mouse button or a finger to move the view. A scale bar in the lower left shows a round distance (for example 50 cm or 1 m) at the current zoom.

The ROS adapter negates native LiDAR bearings before binning them into CCW-positive LaserScan angles. This corrects the sign mismatch behind the reported reversed SLAM heading. The user confirmed correct clockwise and counterclockwise pose-arrow rotation after the fix. Left/right and angle-zero calibration are tracked in the [roadmap](../ROS_ROADMAP.md#3-lidar-only-slam-im-web-hub). Start a fresh SLAM map after changing angle conventions; do not combine scans from the previous convention.

`start_lidar.py --ssh` also forwards port 9090 for the radar tab when that local port is free. If it is taken, for example by an HMMD tunnel, the forward is skipped. Over plain LAN the radar tab still needs rosbridge reachable, but the HMMD launcher keeps rosbridge bound to 127.0.0.1.

On 15 October 2026, the CM5 verified live UART0 scans, automatic `slam_toolbox` lifecycle activation, `/map`, TF, map growth during sensor movement, and the browser's Mapping state and pose arrow. That check used the former `slam.launch.py` and a separate `navbot-slam.service`; both are replaced by the supervised start inside the LiDAR service. Measurements and limitations are in the [hardware observations](../../../agent_notes/ros2_pc/sensors/lds08rr/observations.md).

On 19 October 2026, the CM5 verified the supervised start, map save, loading the saved map on restart, map delete with a fresh start, map growth while the sensor moves, restart after the SLAM node is killed, the stale state of a hung node, and a clean stop with no leftover processes ([observations](../../../agent_notes/ros2_pc/slam/observations.md)). A hung node shows as "SLAM not updating" but is not restarted. Quantitative trajectory accuracy, complete room coverage, and physical scan handedness also still require verification.

## LDS08RR LiDAR

The `lds08rr_lidar` package uses `lds2d[viz]==0.7.0` for the confirmed LDS08RR Camsense driver and polar plot. The local launcher adds an operational status page at `http://127.0.0.1:8081/lidar`. It works in demo mode without a serial device or ROS. When ROS 2 is available, the same reader publishes `/scan` and `/lidar/status`.

Create a Python environment that can import the ROS 2 system packages. Run these commands from the project root. The example uses ROS 2 Jazzy and Python 3.12.

```bash
source /opt/ros/jazzy/setup.bash
python3 -m venv --system-site-packages "$HOME/.venvs/navbot-lidar"
source "$HOME/.venvs/navbot-lidar/bin/activate"
python -m pip install 'lds2d[viz]==0.7.0'
rosdep install --from-paths src/ros2_pc/ros2/src/lds08rr_lidar --ignore-src -r -y --skip-keys=ament_python
python "$(command -v colcon)" --log-base src/ros2_pc/ros2/log build --base-paths src/ros2_pc/ros2/src --packages-select lds08rr_lidar --build-base src/ros2_pc/ros2/build --install-base src/ros2_pc/ros2/install
source src/ros2_pc/ros2/install/setup.bash
```

Run colcon with the environment's Python so the installed executable can import the pinned driver.

Start a synthetic scan to check the page without hardware.

```bash
python3 scripts/start_lidar.py --demo
```

The page labels generated data as `DEMO DATA`. To run the page without ROS, add `--ros off`. The default `--ros auto` starts ROS when its Python modules are available and keeps the HTTP page up when they are not.

LiDAR TX uses UART0 RX on J8 pin 10 / GPIO15, received as `/dev/ttyAMA0`. Leave LiDAR RX disconnected; J8 pin 8 / GPIO14 is host TX. A 15 October 2026 capture confirmed Camsense at 115200 baud, 8N1. Live browser scans and ROS `/scan` were verified at about 5 Hz. Verify electrical conditions before changing wiring and ensure no other process owns the UART.

```bash
python3 scripts/start_lidar.py --device /dev/ttyAMA0
```

Camsense is the sole serial protocol; no selection flag is needed. Use `--host 0.0.0.0` to accept browser connections from the LAN. Use `--ssh user@host --remote-path /absolute/project/path` to run on the ros2_pc and forward port 8081 to local loopback. SSH mode runs `$HOME/.venvs/navbot-lidar/bin/python` on the remote host, so create that environment there with the commands above. Keep the SSH command open while you use the page. A port conflict exits with an error instead of starting a second reader.

### Start LiDAR automatically at boot

On the ros2_pc, stop any manually launched LiDAR process before setup so only one process owns the UART. This requires the existing ROS 2 Jazzy installation, `$HOME/.venvs/navbot-lidar/bin/python`, and access to `/dev/ttyAMA0` through the `dialout` group. From the repository root, run as your normal user. The installer uses this checkout and existing environment, runs the service as your user, and writes `/etc/systemd/system/navbot-lidar.service` from `config/systemd/navbot-lidar.service.in`. It starts at boot without an interactive login and restarts after process failures. Rerun the command after moving the checkout to update and restart the service.

```bash
python3 scripts/install_lidar_service.py
```

Open `http://<ros2_pc-host>:8081/lidar` from a browser on the LAN. Manage the service with:

```bash
sudo systemctl status navbot-lidar.service
sudo journalctl -u navbot-lidar.service -f
sudo systemctl restart navbot-lidar.service
```

To disable startup and remove the unit:

```bash
sudo systemctl disable --now navbot-lidar.service
sudo rm /etc/systemd/system/navbot-lidar.service
sudo systemctl daemon-reload
```

`/status.json` reports waiting, invalid bytes, live, stale, and error states. The browser hides the polar plot when serial data is stale or the status server is unavailable. The same responsive status page is available at `/` and `/lidar`, with its JSON scan feed at `/scan.json`.

Run the protocol, runtime, ROS conversion, launcher, and optional Playwright browser checks with the development environment active.

```bash
python3 -m pytest -q src/ros2_pc/ros2/src/lds08rr_lidar/test scripts/test_start_lidar.py
```

The ROS acceptance test publishes demo scans on `/scan`. Stop live LiDAR and SLAM before running it, or prefix the command with an unused domain such as `ROS_DOMAIN_ID=83` to isolate the test from live mapping.

Install `pytest` and `playwright` in that environment, then run `python -m playwright install chromium` to include the browser acceptance check. ROS checks require the Jazzy environment above. Serial tests use Linux pseudo-terminals and recorded or constructed packet bytes.

Automated checks use recorded and synthetic packets; live acceptance is recorded separately in the [LDS08RR hardware reference](../hardware/lds08rr-lidar.md), together with wiring, electrical limits, and remaining calibration checks.
