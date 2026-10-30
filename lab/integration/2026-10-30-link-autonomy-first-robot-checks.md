# First robot checks of the link autonomy features, 2026-10-30

Status: open

## Observation

Firmware 9c3c5531 (build ID "9c3c5531 2026-10-30 19:41") was flashed with `scripts/upload_firmware.py`. `navlink info` reported that build ID and 80 catalog entries. The checks ran over the ros2_pc UART `/dev/ttyAMA3` with the RC transmitter off and balance off.

- Health at boot: heap_free_b 298068, heap_min_free_b 297876, stack_free_loop_b 4600, stack_free_link_b 2988, all link counters 0, log_level 2, ch5_position 0, rc_valid false (transmitter off).
- `navlink call reboot` without `--yes` exited with 2 and sent nothing. `navlink call reboot --yes` at CH5 position 0 was accepted, uptime_s restarted, and `navlink get --wait 60` reconnected.
- The multiplexer, started by hand on 127.0.0.1:7650, served three clients at once: a health capture at 1 Hz (8 rows), a three-signal capture at 100 Hz (800 rows), and a third client that set log_level 2 to 3 and back to 2. Both captures completed with no `# lost` lines, and both files contain the two `[param] log_level` Log lines.
- Loop timing at 100 Hz over 8 s, `loop_interval_max_us`:
  - With the system-health measurement inside the control tick (build 19cc7605): p50 1848, p99 2343, max 2709, with a spike once per second.
  - After moving the measurement to its own task (build 9c3c5531): p50 1881, p99 1930, max 2118, no spike.
  - `control_execution_us` p99 fell from 1134 to 432.

## Evidence

The values above were read from the robot on 2026-10-30 with `navlink`, and the multiplexer ran from a shell on the ros2_pc. The raw capture files are not cited here.

Not yet checked: the CH5 thresholds 592 and 1402 and the reboot refusal in positions 1 and 2 (both need the RC transmitter), and all motor checks.

## Practical consequence

- The health values give the baseline for heap and stack on this build. A later build that drops the free stack or heap well below these numbers needs a look before it is flashed.
- Moving the health measurement out of the control tick removes the once-per-second loop spike and halves the p99 control execution time. Keep it in its own task.
- `navlink` can capture while the ROS bridge runs, because both use the multiplexer. Two captures and a parameter write at the same time did not lose rows.
- `reboot` is safe from the link only at CH5 position 0. The refusal in positions 1 and 2 must still be shown before the link drives the robot.
- The CH5 thresholds, the reboot refusal in positions 1 and 2, and all motor checks stay open in the [integration roadmap](../../docs/integration/plans/roadmap.md#link-autonomy-on-the-robot) until they are run with the transmitter and the robot secured.

## CH5 and reboot with the transmitter, 2026-10-30

Same build 9c3c5531, RC transmitter on (`rc_valid` 1 throughout), robot held upright with the wheels free. A 120 s capture at 10 Hz of `ch5_position`, `rc_valid`, `balance_state`, `balance_fault`, `link_balance` and `overspeed_tripped` while CH5 was moved by hand (`work/ch5-check.csv`, not kept):

- `ch5_position` read 0, 1 and 2 as separate stable values in each switch position, so the thresholds 592 and 1402 separate the three positions of this transmitter.
- Moving 0 to 2 slowly showed position 1 for 0.1 to 0.2 s on the way; `link_balance` stayed 0 and the controller stayed `Disabled` until position 2. A quick move 0 to 2 showed no sample at 1. `link_balance` was 0 for the whole capture.
- Every time CH5 reached position 2, the controller went `Disabled -> Arming -> Run` within 2 ms (the robot was upright, inside the 5 degree arming window). With the wheels free, about 50 to 70 ms later the wheel speed reached 75 to 78 rad/s, the overspeed guard (30 rad/s) disabled both drivers and the controller went back to `Disabled`. This happened five times. Each return to position 0 cleared `overspeed_tripped`.
- `navlink call reboot --yes` was refused in position 1 and in position 2 with `CH5 must be 0 and balance off` (exit 1); `uptime_s` kept counting.

Consequence: the CH5 positions and the reboot refusal behave as designed. Position 2 with free wheels and an upright robot spins the wheels up to the overspeed limit before the guard stops them; for bench checks that do not need balance, keep CH5 out of position 2. The motor checks with balance on the ground stay open.

## Multiplexer and bridge as services, 2026-10-30

Same build 9c3c5531. Rebuilt `navbot_link_bridge` with colcon and ran `scripts/install_link_bridge_service.py --heading-source gyro --gyro-yaw-sign 1`; both `navbot-link-mux.service` and `navbot-link-bridge.service` started and are enabled at boot. The bridge connects to `tcp:127.0.0.1:7650`. Robot on the bench, CH5 0, transmitter off (`rc_valid` 0).

- While the bridge ran, `ros2 topic hz` over 12 s measured `/odom` 50.0 Hz, `/imu/data` 50.0 Hz, `/diagnostics` 1.000 Hz and `/battery_state` 1.000 Hz.
- At the same time, `navlink capture --preset drive --seconds 10` through the multiplexer wrote 1000 rows at 100 Hz with no `FramesLost`.
- `/diagnostics` carries one status `navbot_firmware` with level OK, message `Disabled`, the build ID as `hardware_id` and all health keys (heap 294216 B, loop stack 4600 B, link stack 2988 B, all link counters 0). `/battery_state` carries `voltage` 7.55 V, `present` true, and NaN for the unknown fields.
- The bridge log showed only the single startup warning that one odometry step used wheel yaw before the first IMU sample.

Consequence: the services run the multiplexer and the bridge together as designed, and `navlink` can be used next to the running bridge.

## log_level, 2026-10-30

Same build 9c3c5531, CH5 0, balance off, robot resting on the bench. Nothing logs INFO text lines on its own, and a `servo1_trim` set logs only the `[param]` frame, so `navlink call calibrate step=1 --yes` (gyroscope bias, as at boot) was the trigger, with `navlink log --seconds 12` running:

- `log_level` 3: only `INFO [action] calibrate started step=1`.
- `log_level` 2: the same `[action]` line, then `INFO [IMU] Calibrating gyroscope...`, `INFO [IMU] Gyroscope calibration completed.` and `INFO [Calibration] gyroBiasX: 17.72, gyroBiasY: -13.91, gyroBiasZ: 24.20`.

`log_level` was set back to 2. Consequence: `log_level` 3 hides the INFO text lines, and the `param` and `action` frames go out at every level, as the [signal table](../../docs/integration/software/signal-table.md) states.
