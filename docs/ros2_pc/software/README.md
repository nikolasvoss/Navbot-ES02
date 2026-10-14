# ros2_pc-Software

This directory documents software that runs on the ros2_pc, including its operating system, ROS 2 packages, services, applications, and deployment steps.

The [ROS 2 roadmap](../ROS_ROADMAP.md) tracks future work and live acceptance status.

- [HMMD ROS 2 developer guide](hmmd-ros2.md) for the current build, startup, browser, topic, and recording workflows.

The HMMD ROS 2 workspace is under `src/ros2_pc/ros2/`.

## LDS08RR LiDAR

The `lds08rr_lidar` package uses `lds2d[viz]==0.7.0` for the LDS08RR protocol drivers and polar plot. The local launcher adds an operational status page at `http://127.0.0.1:8081/lidar`. It works in demo mode without a serial device or ROS. When ROS 2 is available, the same reader publishes `/scan` and `/lidar/status`.

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

The live serial command requires confirmed signal levels, UART2 pin mux, and a raw capture that identifies the protocol. The initial device path is provisional.

```bash
python3 scripts/start_lidar.py --device /dev/ttyAMA2 --protocol camsense
```

Choose `delta` or `camsense` explicitly. The launcher does not detect a protocol from arbitrary bytes. Use `--host 0.0.0.0` to accept browser connections from the LAN. Use `--ssh user@host --remote-path /absolute/project/path` to run on the ros2_pc and forward port 8081 to local loopback. SSH mode runs `$HOME/.venvs/navbot-lidar/bin/python` on the remote host, so create that environment there with the commands above. Keep the SSH command open while you use the page. A port conflict exits with an error instead of starting a second reader.

`/status.json` reports waiting, invalid bytes, live, stale, and error states. The browser hides the polar plot when serial data is stale or the status server is unavailable. The upstream plot remains at `/`, and its JSON feed remains at `/scan.json`.

Run the protocol, runtime, ROS conversion, launcher, and optional Playwright browser checks with the development environment active.

```bash
python3 -m pytest -q src/ros2_pc/ros2/src/lds08rr_lidar/test scripts/test_start_lidar.py
```

Install `pytest` and `playwright` in that environment, then run `python -m playwright install chromium` to include the browser acceptance check. ROS checks require the Jazzy environment above. Serial tests use Linux pseudo-terminals and recorded or constructed packet bytes.

The checks use synthetic protocol fixtures and do not prove a live sensor connection. The [LDS08RR hardware reference](../hardware/lds08rr-lidar.md) records the UART assignment, current host observations, electrical checks, ROS field limits, and remaining hardware acceptance.
