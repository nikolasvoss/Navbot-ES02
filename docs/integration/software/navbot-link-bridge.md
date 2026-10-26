# NavBot link bridge

The `navbot_link_bridge` package reads the ESP32 link ([protocol](ros2-uart-protocol.md)) from a serial port. It publishes `nav_msgs/Odometry` on `/odom`, the `odom` to `base_link` TF (unless `publish_tf` is false), and `sensor_msgs/Imu` on `/imu/data`. On 19 October 2026 the bridge was built on the ros2_pc and fed firmware-format frames through a pseudo-terminal: `/odom` arrived at 50 Hz with the expected pose, and the TF and `/imu/data` were published. It has not run against the robot. The ros2_pc side of the link uses GPIO8/GPIO9 ([J8 allocation](../../ros2_pc/hardware/system-overview.md#j8-pin-allocation)) and is wired; `/dev/ttyAMA3` carried the robot's frames on 2026-10-26.

Configure the ros2_pc first with the [setup page](ros2-pc-setup.md): packages, the boot overlay for the link UART, and serial permissions.

Build with the system ROS 2 Jazzy install. The package needs `python3-serial` and `python3-protobuf` 3.21 from apt, which the generated `navbot_link_pb2.py` requires.

```bash
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src/ros2_pc/ros2/src/navbot_link_bridge --ignore-src -r -y --skip-keys=ament_python
colcon --log-base src/ros2_pc/ros2/log build --base-paths src/ros2_pc/ros2/src --packages-select navbot_link_bridge --build-base src/ros2_pc/ros2/build --install-base src/ros2_pc/ros2/install
source src/ros2_pc/ros2/install/setup.bash
```

Run it with the serial port and the measured wheel geometry. `port`, `wheel_radius_m` and `wheel_separation_m` have no usable default, and the node refuses to start without `port`. Odometry needs both radius and separation above zero.

```bash
ros2 run navbot_link_bridge navbot_link_bridge --ros-args -p port:=/dev/ttyAMA3 -p wheel_radius_m:=0.01915 -p wheel_separation_m:=0.113
```

`port` is the serial device on the ros2_pc that carries the ESP32 link, here `/dev/ttyAMA3` (GPIO8/GPIO9 on J8), not the LiDAR's `/dev/ttyAMA0` or the radar's UART2. Check the name before use, as described in the [setup page](ros2-pc-setup.md).

To start the bridge at boot like the LiDAR service, run `python3 scripts/install_link_bridge_service.py` as the service user, without sudo. It needs the build above and writes `navbot-link-bridge.service` with `/dev/ttyAMA3`, `wheel_radius_m:=0.01915` and `wheel_separation_m:=0.113`; edit the constants in the script to change them. The `/slam` tab depends on this service.

Optional parameters: `baud` (500000), `odom_frame` (`odom`), `base_frame` (`base_link`), `imu_frame` (`imu_link`), `publish_tf` (`true`). The node reopens the port every second after an error, so a power-cycled robot reconnects.

Unverified, before relying on the output:

- The track width is 113 mm and the wheel diameter 38.3 mm ([robot hardware overview](../../robot/hardware/system-overview.md#key-findings-and-consequences)). The bridge has no defaults and does not guess them: pass `-p wheel_radius_m:=0.01915 -p wheel_separation_m:=0.113`. The track width is a hand measurement; an odometry turn test on the floor would confirm it.
- The forward direction of the wheel angles was checked on the link on 2026-10-25: both angles grow when the wheels roll forward ([lab note](../../../lab/integration/2026-10-25-odometry-signs-and-rate.md)). The left/right mapping is not verified (both wheels were rolled together), and neither are the signs of the `/odom` twist. Roll one wheel at a time to check the mapping.
- The IMU frame `imu_link` has no `base_link` to `imu_link` TF yet.
- Stamps are the ROS receive time, not the robot's sample time. The robot's IMU read time (`Imu.t_us`) is decoded but not published, because nothing converts robot time into ROS time. Covariances are zero.

Test the framing, decoding and odometry logic without ROS. The tests need `pytest` and `protobuf`.

```bash
PYTHONPATH=src/ros2_pc/ros2/src/navbot_link_bridge python3 -m pytest -q src/ros2_pc/ros2/src/navbot_link_bridge/test
```
