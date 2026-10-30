# Integration roadmap

## ESP32 to ros2_pc link: remaining work

Status: the protocol (COBS, CRC-16, Protobuf), the firmware sender for `WheelOdometry` and `Imu` at 50 Hz, and the ROS 2 bridge `navbot_link_bridge` are implemented. The bridge ran on the ros2_pc only with simulated frames through a pseudo-terminal. Details: [protocol](../software/ros2-uart-protocol.md), [bridge](../software/navbot-link-bridge.md).

- [x] Merge the odometry bridge branch into `feature/esp32-ros2-uart-interface`.
- [x] Merge `main` (with the new repository layout) into `feature/esp32-ros2-uart-interface` and resolve the conflicts there, not on `main`.
- [x] Merge the robust control branch (`docs/robust-control-plan`) and attach the odometry call to its control tick. The sender now publishes forward-positive wheel angles: `motor.shaftAngle()` times `WheelBoundary::kLeftMotorSign` and `kRightMotorSign`, stamped with the wheel read time. The forward sign was checked on the robot on 2026-10-25.
- [x] Choose the ros2_pc UART: GPIO8/GPIO9 (J8 pins 24 and 21), recorded in the [J8 allocation](../../ros2_pc/hardware/system-overview.md#j8-pin-allocation) on 2026-10-25. The user called it UART4. The carrier datasheet lists GPIO8/9 as UART3 and UART4 as GPIO12/13, so the pins decide.
- [x] Enable the UART on the ros2_pc (boot overlay and reboot), confirm the `/dev/ttyAMA*` path, verify pins and 3.3 V levels on the board, wire it, and replace the `port` placeholder in the [bridge page](../software/navbot-link-bridge.md).
- [x] Flash the firmware and check the 500 kBd link on the real robot (2026-10-26): `WheelOdometry` and `Imu` at 50.0 Hz each on a 20 ms schedule, 12002 CRC-valid frames in 2 minutes with the robot balancing, no lost frames. Measured with a script, not with the bridge.
- [ ] Run the bridge on the ros2_pc against the robot: `/odom` and `/imu/data` at about 50 Hz, no `bad_frames` or `lost_frames` in the bridge log.
- [x] Track width 113 mm (2026-10-26) and wheel diameter 38.3 mm (2026-10-24) are documented in the [robot hardware overview](../../robot/hardware/system-overview.md#key-findings-and-consequences). The control firmware uses a radius of 0.019 m (exact: 0.01915 m). The bridge has no default for either parameter; pass `wheel_radius_m` and `wheel_separation_m`.
- [x] Left/right mapping checked with a 3-turn spin on 2026-10-26: correct ([lab note](../../../lab/integration/2026-10-26-odometry-turn-test.md)). The same test showed that the wheel yaw over-reads by 1.135 (effective separation 0.128 m with the measured radius).
- [ ] Check the wheel radius with a straight push along a tape measure, and the `/odom` twist signs on the robot.
- [x] Gyro heading in the bridge: `heading_source:=gyro` takes the `/odom` yaw from the IMU gyro z rate and the distance from the wheels, with wheel-yaw fallback when IMU samples are missing ([bridge page](../software/navbot-link-bridge.md#heading-source)). Tested with simulated samples only.
- [x] `gyro_yaw_sign` is +1 (turn test 2026-10-26: 1080.4° for 3 counter-clockwise turns, [lab note](../../../lab/integration/2026-10-26-odometry-turn-test.md)).
- [ ] Run gyro heading on the robot: standstill drift over a minute, and a SLAM map that is no longer rotated.
- [ ] Define the static TF `base_link` to `imu_link` from the IMU mounting. `/imu/data` is in the sensor frame.
- [x] Align the SLAM branch (`feat/slam`): the static `odom` to `lds08rr` TF is gone, the LiDAR service publishes a static TF `base_link` to `lds08rr`, and `slam_toolbox` runs with `base_frame: base_link`. Not yet run on the robot.
- [ ] Check the LiDAR yaw offset on the robot: `SENSOR_MOUNT_YAW_RAD` is −120° (ahead reads 240° clockwise on the LiDAR page). Push the robot straight forward and confirm the SLAM pose arrow points along the direction of travel. `SENSOR_MOUNT_X_M` and `SENSOR_MOUNT_Y_M` stay zero because the LiDAR sits at the robot's rotation centre.
- [x] Start `navbot_link_bridge` at boot with `scripts/install_link_bridge_service.py` (unit and installer written, test passes). Not yet installed or run on the robot.

## Link logs, signals and parameters on the robot

Context: the log frames, the signal table, the parameters and the actions are implemented and host-tested: [signal table](../software/signal-table.md), [navlink](../software/navlink.md), [bridge](../software/navbot-link-bridge.md) and [sensor diagnostic mode](../../robot/software/sensor-diagnostic-mode.md). None of the checks below has run on the robot. Run them with the robot supported, balance off and the motor outputs safe.

- [ ] Normal image, on the ros2_pc: `navlink capture --preset drive` runs through the link multiplexer (`/dev/ttyAMA3`), and `navlink shell` lists the catalog and reads the parameters.
- [ ] Normal image: `navlink shell` sets one parameter, then calls `motor_stop` and `motor_reset`.
- [ ] A `Log` from a `set` made on one port appears on the other port, and the USB port shows the same `Log` frames as UART1.
- [ ] Boot, both images: the setup lines (`system run.`, the calibration lines, the `Link` GPIO19 check) arrive on UART1 from the first boot, not only on USB.
- [ ] Diagnostic image: `sensor_monitor.py` prints the `IMU_TIMING` line in full; `navlink shell` reads the catalog and the live signals on the [diagnostic page](../../robot/software/sensor-diagnostic-mode.md#which-signals-are-live); `motor_current`, `motor_voltage` and `calibrate` are refused with their reasons.
- [ ] Bridge on the ros2_pc against the robot (so far run only against the fake firmware): `/rosout` shows the robot's `Log` lines, `/navbot/signals` streams a chosen set of signals, and the firmware parameters appear as ROS parameters.
## Link autonomy on the robot

Context: the CH5 link position, `balance` and `reboot` from the link, the drive `Command` and its timeout, the live parameters and their steps, the link multiplexer, the one-shot navlink commands, the build ID, `log_level`, the health signals and the bridge's `/cmd_vel`, `/diagnostics` and `/battery_state` are implemented and host-tested: [protocol](../software/ros2-uart-protocol.md#command), [navlink](../software/navlink.md), [bridge](../software/navbot-link-bridge.md#drive-and-health-topics) and [signal table](../software/signal-table.md#health-balance-state-and-link-signals). The checks below that are marked done ran on the robot on 2026-10-30 ([lab note](../../../lab/integration/2026-10-30-link-autonomy-first-robot-checks.md)); the others have not run. Run them with the robot supported, wheels free, and balance off unless a check says otherwise.

- [x] Build ID: `scripts/build_remote.sh`, flash, `navlink info`: the build ID starts with `git rev-parse --short=8 HEAD`. Checked on 2026-10-30 with 9c3c5531 ([lab note](../../../lab/integration/2026-10-30-link-autonomy-first-robot-checks.md)).
- [x] Multiplexer sharing: a health capture, a 100 Hz capture and a third client's parameter write ran at the same time through the multiplexer, with no lost rows ([lab note](../../../lab/integration/2026-10-30-link-autonomy-first-robot-checks.md)).
- [ ] Multiplexer and bridge: reinstall both services and start them, then a 10 s `navlink capture` while the bridge publishes `/odom`: no `FramesLost`.
- [x] CH5: move through positions 0, 1 and 2 and read `ch5_position` each time (checks the raw thresholds 592 and 1402). A quick move 0 to 2 starts nothing on the way through 1 ([lab note](../../../lab/integration/2026-10-30-link-autonomy-first-robot-checks.md#ch5-and-reboot-with-the-transmitter-2026-10-30)).
- [x] `navlink call reboot --yes` at CH5 position 0 is accepted, and `navlink call reboot` without `--yes` sends nothing. `navlink get --wait 60` reconnects with a new `uptime_s` ([lab note](../../../lab/integration/2026-10-30-link-autonomy-first-robot-checks.md)).
- [x] `navlink call reboot --yes` is refused in positions 1 and 2 ([lab note](../../../lab/integration/2026-10-30-link-autonomy-first-robot-checks.md#ch5-and-reboot-with-the-transmitter-2026-10-30)).
- [x] Health values: `navlink capture --preset health` shows plausible heap and stack values at boot, and zero link counters ([lab note](../../../lab/integration/2026-10-30-link-autonomy-first-robot-checks.md)).
- [ ] Bridge health topics: `/diagnostics` and `/battery_state` arrive at 1 Hz.
- [ ] `navlink set log_level 3`: no INFO lines in `navlink log --seconds 10`. Set it back to 2.
- [ ] With the user, balance in position 2: a live `set speed_kp` within its step is accepted, and one over it is refused. The steps in the [signal table](../software/signal-table.md#parameters-settings-groups) are confirmed or changed here.
- [ ] With the user: moving CH5 from 2 to 1 keeps balancing; `call balance on=0` and `on=1` in position 1.
- [ ] With the user: `/cmd_vel` at 0.1 m/s from `teleop_twist_keyboard`; stop publishing and the robot stops within the timeout.
- [ ] With the user: CH5 from 1 to 0 ends balance; 0 to 2 in one move starts balance only at 2; 2 to 0 in one move ends it. Record the results in a lab note.

## Optional: fault-triggered recording (black box)

Status: optional, not planned. Start only after the user's go-ahead. A balance fault (a fall, overspeed, a saturated tilt reference) develops within a few control ticks, and a subscription that streams every tick is limited by the link budget, so the ticks before a fault are usually missing from a capture.

- A ring buffer in RAM keeps the last 1.0 s of a fixed signal list at every control tick and freezes on the transition into `Fault` or an overspeed trip; the PC reads it out afterwards as a CSV file in the `navlink capture` format.
- Size: 16 signals × 4 bytes + 4 bytes `t_us` = 68 bytes per tick; at about 710 ticks per second, 1.0 s before and 0.25 s after the fault is about 61 kB of internal SRAM (the board has no PSRAM). The free heap decides whether it fits; half the sampling rate halves it.
- Work: a pure ring buffer class with host tests, a flag on the chosen entries in `signals.def.h`, an action to clear and rearm, a readout request that sends the frozen rows as `SignalBatch` frames while balance is off, and `navlink blackbox -o fall.csv`. About 250 lines of firmware and tests, 100 lines of Python, one proto message.

- [ ] Decide whether to build it, after the free heap has been measured on the robot.

## Decide the dashboard for logs and debug signals

Context: the [signal table](../software/signal-table.md) and the link logs go over the NavBot link on UART1 to the ros2_pc, and on the USB port as a backup. The ROS 2 bridge publishes them as `/rosout` and `/navbot/signals`. The protocol does not depend on this decision.

Decision: which tool shows the live debug signals (plots for control tuning), the log messages, and the parameter get/set. Choose one:

| Option | What it is | Runs on | Effort here |
| --- | --- | --- | --- |
| PlotJuggler | Desktop app for live time-series plots of ROS 2 topics; also opens CSV files. No logs or parameter panel. | Laptop with ROS 2, or the ros2_pc with a screen | Install only |
| Foxglove | Web or desktop app; plots, log list, 3D view of the map and scan, parameter panel. Connects to `foxglove_bridge`, a ROS 2 node on the ros2_pc. | Any browser; bridge on the ros2_pc | Install `foxglove_bridge`, save a layout |
| Own web page | Extend the LiDAR web service (`lds08rr_lidar/web.py`) with a signal plot, a log list and parameter fields. | Any browser | Write and maintain the page |

Criteria: plot rate needed for control tuning (up to the control tick, about 700 Hz, decimated), whether the tool works without a ROS 2 install on the laptop, and maintenance effort.

- [ ] Try PlotJuggler and Foxglove with the bridge's signal topic once it exists, then record the choice here.
