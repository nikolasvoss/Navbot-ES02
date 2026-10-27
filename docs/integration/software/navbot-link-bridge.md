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

To start the bridge at boot like the LiDAR service, run `python3 scripts/install_link_bridge_service.py` as the service user, without sudo. It needs the build above and writes `navbot-link-bridge.service` with `/dev/ttyAMA3`, `wheel_radius_m:=0.01915` and `wheel_separation_m:=0.113`; edit the constants in the script to change them. Pass `--heading-source gyro --gyro-yaw-sign <1|-1>` to install it with gyro heading (see below). The `/slam` tab depends on this service.

Optional parameters: `baud` (2000000), `odom_frame` (`odom`), `base_frame` (`base_link`), `imu_frame` (`imu_link`), `publish_tf` (`true`). The node reopens the port every second after an error, so a power-cycled robot reconnects.

## Heading source

`heading_source` selects where the `/odom` yaw comes from:

- `wheels` (default): yaw change is the wheel differential divided by `wheel_separation_m`. Wheel slip and the hand-measured track width make this yaw poor on the balancing robot.
- `gyro`: yaw change comes from the IMU gyro z rate; distance still comes from the wheels. Requires `gyro_yaw_sign` (`1` or `-1`), which maps the IMU `gz` to ROS +yaw (counter-clockwise seen from above, REP-103). The node has no default for it and refuses to start without it, like `port`. Unknown `heading_source` values also stop the node.

```bash
ros2 run navbot_link_bridge navbot_link_bridge --ros-args -p port:=/dev/ttyAMA3 -p wheel_radius_m:=0.01915 -p wheel_separation_m:=0.113 -p heading_source:=gyro -p gyro_yaw_sign:=1
```

How gyro heading works (`odometry.py`):

- Each `Imu` frame adds `gyro_yaw_sign * gz` integrated with the trapezoid rule over the difference of the robot's `Imu.t_us` (wrap-safe at 2^32). An interval longer than 60 ms (three link periods) is not integrated.
- At each wheel sample the bridge uses the gyro yaw collected since the previous wheel sample instead of the wheel yaw. The position still integrates the wheel distance at the mid-step yaw, and the `/odom` angular rate is the latest signed gyro rate. The firmware sends the wheel frame before the IMU frame of the same 20 ms tick, so the gyro yaw lags the wheel distance by one tick.
- When no gap-free IMU interval covers a step (before the first two IMU samples, after a gap, or when IMU frames stop), that step uses the wheel yaw. The node counts these fallbacks and warns at most every 5 s. One fallback at start and after each robot restart is expected.
- A robot restart (seq reset) keeps the pose and drops both the wheel and the gyro baselines.

The gyro rate on the link is `filteredAngularVelocityRadPerSec.z` (`src/ES-02/OllieFOCdrive/imu.cpp`): the firmware subtracts the stored gyro bias from the raw counts, then scales and low-pass filters them. The bias is measured once by the explicit gyro calibration (refused while balancing) and loaded from the calibration store at boot; it does not track temperature. A remaining bias shows as a slow, steady `/odom` yaw drift while the robot stands still; recalibrate the gyro if the drift is visible over a minute.

## Turn test results

A 3-turn counter-clockwise spin on 2026-10-26 ([lab note](../../../lab/integration/2026-10-26-odometry-turn-test.md)) gave:

- `gyro_yaw_sign` is **+1**: positive `gz` is counter-clockwise seen from above. The gyro read 1080.4° for 1080°.
- The left/right wheel mapping is correct: the wheel yaw is positive for a counter-clockwise turn.
- Wheel yaw over-reads by a factor of 1.135 with `wheel_radius_m:=0.01915` and `wheel_separation_m:=0.113`. Wheel yaw scales with radius / separation, so the effective ratio is 0.149; with the measured radius that is an effective separation of 0.128 m. Gyro heading avoids this error. The boot service keeps the measured 0.113 m, which only affects the wheel-yaw fallback.

Repeat the test with `python3 scripts/odometry_turn_test.py --turns <n>` while the bridge runs in wheel heading mode: mark the floor, turn the robot on the spot exactly `n` turns (positive counter-clockwise seen from above), stop on the mark and press Ctrl-C. The JSON gives `track_width_m`, `left_right_mapping`, `gyro_yaw_sign` and `gyro_scale`. Stop on the mark before Ctrl-C; any movement after it falsifies the result.

Unverified, before relying on the output:

- The wheel diameter is 38.3 mm and the hand-measured track width 113 mm ([robot hardware overview](../../robot/hardware/system-overview.md#key-findings-and-consequences)). The bridge has no defaults and does not guess them: pass `-p wheel_radius_m:=0.01915 -p wheel_separation_m:=0.113`. The turn test shows that their ratio is 13.5 % off for yaw (see above). The radius alone, which sets the `/odom` distance, is not checked; push the robot straight along a tape measure and compare `/odom` x.
- The signs of the `/odom` twist are not checked on the robot.
- The IMU frame `imu_link` has no `base_link` to `imu_link` TF yet.
- Stamps are the ROS receive time, not the robot's sample time. The robot's IMU read time (`Imu.t_us`) is decoded but not published, because nothing converts robot time into ROS time. Covariances are zero.

Test the framing, decoding and odometry logic without ROS. The tests need `pytest` and `protobuf`.

```bash
PYTHONPATH=src/ros2_pc/ros2/src/navbot_link_bridge python3 -m pytest -q src/ros2_pc/ros2/src/navbot_link_bridge/test
```
