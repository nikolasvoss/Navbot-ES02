# ros2_pc-Software

This directory documents software that runs on the ros2_pc, including its operating system, ROS 2 packages, services, applications, and deployment steps.

The [ROS 2 roadmap](../ROS_ROADMAP.md) tracks future work and live acceptance status.

- [HMMD ROS 2 developer guide](hmmd-ros2.md) for the current build, startup, browser, topic, and recording workflows.

The HMMD ROS 2 workspace is under `src/ros2_pc/ros2/`.

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

Install `pytest` and `playwright` in that environment, then run `python -m playwright install chromium` to include the browser acceptance check. ROS checks require the Jazzy environment above. Serial tests use Linux pseudo-terminals and recorded or constructed packet bytes.

Automated checks use recorded and synthetic packets; live acceptance is recorded separately in the [LDS08RR hardware reference](../hardware/lds08rr-lidar.md), together with wiring, electrical limits, and remaining calibration checks.

## NavBot link odometry bridge

The `navbot_link_bridge` package reads the ESP32 link ([protocol](../../robot/software/interfaces/ros2-uart-protocol.md)) from a serial port. It publishes `nav_msgs/Odometry` on `/odom`, the `odom` to `base_link` TF (unless `publish_tf` is false), and `sensor_msgs/Imu` on `/imu/data`. The ros2_pc UART for the ESP32 link is not chosen or wired yet, so the port below is a placeholder. On 19 October 2026 the bridge was built on the ros2_pc and fed firmware-format frames through a pseudo-terminal: `/odom` arrived at 50 Hz with the expected pose, and the TF and `/imu/data` were published. It has not run against the robot.

Build with the system ROS 2 Jazzy install. The package needs `python3-serial` and `python3-protobuf` 3.21 from apt, which the generated `navbot_link_pb2.py` requires.

```bash
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src/ros2_pc/ros2/src/navbot_link_bridge --ignore-src -r -y --skip-keys=ament_python
colcon --log-base src/ros2_pc/ros2/log build --base-paths src/ros2_pc/ros2/src --packages-select navbot_link_bridge --build-base src/ros2_pc/ros2/build --install-base src/ros2_pc/ros2/install
source src/ros2_pc/ros2/install/setup.bash
```

Run it with the serial port and the measured wheel geometry. `port`, `wheel_radius_m` and `wheel_separation_m` have no usable default, and the node refuses to start without `port`. Odometry needs both radius and separation above zero.

```bash
ros2 run navbot_link_bridge navbot_link_bridge --ros-args -p port:=<serial-port> -p wheel_radius_m:=<measured-radius-m> -p wheel_separation_m:=<measured-separation-m>
```

Optional parameters: `baud` (500000), `odom_frame` (`odom`), `base_frame` (`base_link`), `imu_frame` (`imu_link`), `publish_tf` (`true`). The node reopens the port every second after an error, so a power-cycled robot reconnects.

Unverified, before relying on the output:

- Wheel radius and track width are unknown. Measure them; the bridge does not guess.
- The forward direction and the left/right mapping of the wheel angles are unverified. Roll the robot forward and check the signs of the `/odom` twist.
- The IMU frame `imu_link` has no `base_link` to `imu_link` TF yet.
- Stamps are the ROS receive time, not the robot's sample time. Covariances are zero.

Test the framing, decoding and odometry logic without ROS. The tests need `pytest` and `protobuf`.

```bash
PYTHONPATH=src/ros2_pc/ros2/src/navbot_link_bridge python3 -m pytest -q src/ros2_pc/ros2/src/navbot_link_bridge/test
```
