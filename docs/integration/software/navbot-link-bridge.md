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

The bridge reads the port through the shared [navbot_link](navlink.md#library) `Link`, which also carries the catalog, parameter and subscription messages (next sections). Only one program can hold the port, so stop the bridge before `navlink`.

Optional parameters: `baud` (2000000), `odom_frame` (`odom`), `base_frame` (`base_link`), `imu_frame` (`imu_link`), `publish_tf` (`true`). The node reopens the port every second after an error, so a power-cycled robot reconnects; the pose is kept and the odometry baseline is taken again.

## Robot log, signals and firmware parameters

Status: **written and tested only against the in-memory fake firmware** (`tests/navbot_link/fake_firmware.py`), plus a pseudo-terminal run of the node. The robot firmware does not answer the catalog messages yet ([navlink](navlink.md)), so none of this has run on the robot.

**Robot log.** Every `Log` frame goes to `/rosout` as `rcl_interfaces/msg/Log`. The `name` is the `Log` tag (`robot` when the tag is empty) and `msg` is the text, with ` (code N)` appended when the code is not 0. The level maps `DEBUG` 10, `INFO` 20, `WARN` 30, `ERROR` 40, `FATAL` 50; a level the bridge does not know is shown as `INFO`. The bridge publishes these messages itself, because rcl sends a logger's lines to `/rosout` only for node loggers. The text does not reach the console or journal of the bridge process. The `/rosout` publisher uses the rcl profile (reliable, transient local, depth 1000) with a 10 s lifespan, so a subscriber that starts later gets the lines of the last 10 s only. `msg.stamp` is the ROS time at which the bridge forwarded the line, not the robot's time.

**Signals.** Two parameters control the stream:

| Parameter | Default | Meaning |
| --- | --- | --- |
| `signals` | `[]` | Catalog names of signals or parameters to stream. An empty list stops the stream. |
| `signals_rate_hz` | `10.0` | Row rate. `0.0` streams every control tick. Negative values are refused. |

```bash
ros2 param set /navbot_link_bridge signals "['speed_target', 'speed_measured']"
ros2 param set /navbot_link_bridge signals_rate_hz 100.0
ros2 topic echo /navbot/signals
```

`/navbot/signals` is a `std_msgs/Float64MultiArray`, one message per robot row. `data` is `[t_us, value0, value1, ...]`. `layout.dim` has one entry whose `label` is the column header `t_us,name0,name1,...`, the same header as the [navlink](navlink.md#capture) CSV, with `size` and `stride` equal to `len(data)`. `t_us` is the robot's `micros()` at the sampled tick (wraps at 2^32); bool values are 0.0 or 1.0, and `i32`, `u32` and `f32` values are exact in a double. The bridge does not convert robot time into ROS time. A message has no header stamp.

The message type is a plain `Float64MultiArray`, not a custom message, because the rows differ per subscription and a custom message would need a CMake interface package next to this `ament_python` package.

A change of `signals` or `signals_rate_hz` subscribes the new list first and releases the old slot after; the robot's budget check can refuse it, and the set then fails with the robot's error text while the old stream keeps running. `Link` renews the 5 s lease every 2 s. After a robot restart or a refused renewal the bridge subscribes again from the current parameters.

**Firmware parameters.** Each `PARAM` entry of the catalog becomes a ROS parameter with the catalog name and the value read from the robot. The ROS parameter is dynamically typed, and the bridge applies the type rules below itself. `ACTION` entries are not exposed, and signals that are not parameters are only streamed.

| Robot type | Accepted in ROS | Stored as |
| --- | --- | --- |
| `f32` | a float, or an integer, which is taken as a float | double |
| `i32`, `u32` | an integer | integer |
| `bool` | a bool | bool |

A value of another type is refused before the robot is asked, with a reason such as `speed_kp takes a number, got 'fast'`. A value the robot refuses (range, cross-field check) fails the set with the robot's error text, for example `set_param refused: speed_kp must be within [0, 0.5]`, and the ROS value stays as it was. Notes:

- The ROS value is a copy: it is what ROS last set or read at connect. A change made through `navlink` or the other port does not update it, and a restart of the robot rebuilds all firmware parameters from the robot.
- A value given at startup (`-p speed_kp:=0.1` or a parameters file) is sent to the robot once the catalog is read. An integer for an `f32` parameter (`-p speed_kp:=1`) is sent and stored as `1.0`. If the robot or the type rule refuses it, the bridge logs `startup value of speed_kp not applied: <reason>` and keeps the robot's value.
- A `set_parameters_atomically` request (the rclpy call, or the `set_parameters_atomically` service) that sets several firmware parameters, or a parameter and the signal stream, is all or nothing. Before the first write the bridge reads the current robot values of that request. If the robot refuses a value, the bridge writes the earlier values back and the result carries the refusal text. If a write-back fails too, the reason names the parameter (`; could not restore speed_ki: ...`). `ros2 param set` sends one name per request and keeps its one-by-one results.
- ROS stores the value the robot reports in its `Ack`, not the requested one. A robot that clamps a value, or `f32` rounding (`0.1` becomes `0.10000000149011612`), shows in `ros2 param get`. The bridge corrects the stored value before the set call returns.
- A name rclpy refuses (an empty name) is skipped with a warning such as `firmware parameter '' not exposed: Invalid parameter name:`; the other parameters and the signal stream still start. A firmware parameter named like a bridge parameter (`port`, `baud`, `signals`, ...) is not exposed, with a warning.

**Waiting for the robot.** A `ros2 param set` waits for the robot's answer (about 1.5 s when the robot does not answer), and so does its caller. That wait is in a service thread. Odometry, TF, IMU and `/rosout` keep their rate: the node runs on a `MultiThreadedExecutor`, and the poll timer has a callback group of its own. The catalog reads, parameter exchanges and subscription changes run in one worker thread, in order, so a second set waits for the first.

**Firmware without the catalog** (the current robot firmware). The bridge asks for the catalog at connect and again after a robot restart, in the worker thread, so odometry, TF, IMU and `/rosout` start at once. When the robot does not answer within about 1.5 s, the bridge logs one warning (`the robot did not answer GetCatalog ...`) and continues without firmware parameters and signals. It does not ask again on the same connection; the next ask comes with a reconnect or a robot restart. A restart whose new build answers no catalog removes the firmware parameters of the old build. Setting `signals` is refused with `the robot sent no catalog`. If the link fails before the robot answers, the bridge does not report a missing answer; it reports the link failure (`link failed: <reason>; reopening`).

## Time stamps

`/odom`, the TF and `/imu/data` carry the robot's sample time mapped to ROS time, not the time the bridge read the frame. `RobotClock` (`bridge.py`) unwraps the 32-bit `t_us` and adds an offset that follows the lowest `ROS time - t_us` seen, moved up by 0.1 % of the excess on later samples to follow clock drift. A stamp is never later than the time the bridge received the sample, and stamps of one run strictly increase, also for a backlog that the bridge reads in one go. The offset starts again at a reconnect and at a robot restart. `/rosout` lines carry the ROS time at which the bridge forwarded them.

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
- Covariances are zero.

Test the decoding, odometry, log, signal and parameter logic without ROS. The tests need `pytest` and `protobuf`. The framing and `Link` live in the shared [navbot_link](../../../src/navbot_link/) package, which `setup.py` installs next to the bridge; its tests are in `tests/navbot_link`, and the bridge tests reuse its fake firmware.

```bash
PYTHONPATH=src:src/ros2_pc/ros2/src/navbot_link_bridge python3 -m pytest -q src/ros2_pc/ros2/src/navbot_link_bridge/test
```

`test_node.py` runs the node against the fake firmware. It checks `/rosout` (lines and their stamp), `/navbot/signals`, the type rules, atomic sets, parameter results, reconnect and restart behavior, the warnings, and that odometry keeps its cadence while the robot ignores GetCatalog or a parameter set. It is skipped unless ROS 2 is importable. With the system install, and the ROS pytest plugins switched off because they fail to load under the system pytest:

```bash
source /opt/ros/jazzy/setup.bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 PYTHONPATH=$PYTHONPATH:src:src/ros2_pc/ros2/src/navbot_link_bridge python3 -m pytest -q src/ros2_pc/ros2/src/navbot_link_bridge/test
```

Node tests need an isolated ROS 2 graph. The real bridge runs on this machine and publishes `/odom` and `/imu/data`; a test node in the same domain would receive them and publish into the same topics. `test/conftest.py` therefore sets `ROS_DOMAIN_ID=77` and `ROS_LOCALHOST_ONLY=1` before `rclpy.init()`, and `test_tests_run_in_their_own_ros_graph` fails if another bridge's publisher is visible. Every node test you add runs under these settings. When you run a node test by hand, do not unset them and do not run it in domain 0.
