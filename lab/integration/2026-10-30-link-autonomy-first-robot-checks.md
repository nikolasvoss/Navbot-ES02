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
