# NavBot link bridge

The `navbot_link_bridge` package reads the ESP32 link ([protocol](ros2-uart-protocol.md)) from a serial port. It publishes `nav_msgs/Odometry` on `/odom`, the `odom` to `base_link` TF (unless `publish_tf` is false), and `sensor_msgs/Imu` on `/imu/data`. On 19 October 2026 the bridge was built on the ros2_pc and fed firmware-format frames through a pseudo-terminal: `/odom` arrived at 50 Hz with the expected pose, and the TF and `/imu/data` were published. It has not run against the robot. The ros2_pc side of the link is planned on GPIO8/GPIO9 ([J8 allocation](../../ros2_pc/hardware/system-overview.md#j8-pin-allocation)); it is not wired or enabled, and its device path is not confirmed, so the port below is a placeholder.

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
ros2 run navbot_link_bridge navbot_link_bridge --ros-args -p port:=<serial-port> -p wheel_radius_m:=<measured-radius-m> -p wheel_separation_m:=<measured-separation-m>
```

Optional parameters: `baud` (500000), `odom_frame` (`odom`), `base_frame` (`base_link`), `imu_frame` (`imu_link`), `publish_tf` (`true`). The node reopens the port every second after an error, so a power-cycled robot reconnects.

Unverified, before relying on the output:

- The track width is not measured. The bridge does not guess it.
- The forward direction and the left/right mapping of the wheel angles are not verified against this firmware. Roll the robot forward and check the signs of the `/odom` twist.
- The IMU frame `imu_link` has no `base_link` to `imu_link` TF yet.
- Stamps are the ROS receive time, not the robot's sample time. Covariances are zero.

Test the framing, decoding and odometry logic without ROS. The tests need `pytest` and `protobuf`.

```bash
PYTHONPATH=src/ros2_pc/ros2/src/navbot_link_bridge python3 -m pytest -q src/ros2_pc/ros2/src/navbot_link_bridge/test
```
