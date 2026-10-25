# Odometry wheel signs and send rate on the ros2_pc link, 2026-10-25

Status: open

## Observation

With the UART1 link working at 500000 baud (after the STC8H1K08 fix, see [the STC note](../robot/2026-10-25-stc8h1k08-backpowers-uart1.md)):

- Rolling both wheels forward by hand for about 3 s gave positive changes in both cumulative wheel angles: left +11.04 rad, right +9.96 rad. The totals differ by about 10%; the roll was by hand, so this is not evidence of a scale difference.
- `WheelOdometry` arrives every 28.16 ms on average (355 frames; min 28.05 ms, max 28.70 ms), about 35.5 Hz. `Imu` follows at the same rate. The documented rate is 50 Hz each.

## Evidence

- Roll capture: 539 CRC-valid `WheelOdometry` frames in 15 s on `/dev/ttyAMA3` at 500000 baud, decoded as COBS plus CRC-16/CCITT-FALSE.
- Rate: `t_us` deltas from a 10 s capture at 115200 baud (same firmware loop) give a mean frame interval of 28.16 ms, which is 20 control ticks of about 1.41 ms (710 Hz). A 500000 capture gave 358 frames in 10 s (35.8 Hz). The sequence numbers had no gaps, so the sender dropped no frames.
- Code: `runControlTick()` in `OllieFOCdrive.ino` sends odometry when `controlGateSequence % LINK_ODOMETRY_DIVIDER == 0` (divider 20). `loop()` starts a tick only when `controlTimestepSec >= 0.001`, so 1 ms is a minimum gap, not the period.

## Practical consequence

Positive wheel-angle change means forward rolling, as the protocol specifies. Backward rolling was not tested.

The link rate is about 35.5 Hz, not 50 Hz. The control loop does not run at 1 kHz: a tick takes about 1.41 ms. The ROS 2 bridge and any consumer must use `t_us` for time deltas and must not assume 50 Hz. The cause of the 1.41 ms period is not confirmed. It is probably the execution time of one `loop()` pass (motor FOC and sensor reads); `controlExecutionUs` and `maxLoopEntryIntervalUs` in the debug telemetry would show it, but the console was not connected during this test. Open items: read those two values on the CH340 console, and decide whether to keep the 20-tick divider or switch to a time-based 20 ms send interval.

## 2026-10-25 update

The firmware now sends `WheelOdometry` and `Imu` on a 20 ms time schedule instead of every 20th control tick: `runControlTick()` calls `LinkScheduleDue()` (`link_schedule.h`) with the tick start time and `LINK_ODOMETRY_PERIOD_US` (20000). The deadline advances by one period per send, so the expected average is 50 Hz with up to one tick (about 1.41 ms) of jitter per interval. Host tests in `tests/link/` cover the rate, the stall recovery and the `micros()` wrap. Not yet verified on hardware: the firmware was not built or flashed, and the rate was not measured again.

## 2026-10-26 update

Loop timing measured on the CH340 console (firmware before the 20 ms schedule, `K58` DRIVE rows, 6 s, robot idle on the stand): [capture](captures/2026-10-26-k58-loop-timing.log).

- IMU accepted reads: 680 Hz from the `imu_accepted_samples` counter; `imu_long_gap_count` rose by 126 per second (intervals above 1.5 ms). The IMU output data rate is 1 kHz, so about 30% of the sensor samples are not read.
- In the 42 rows whose fields were all plausible: `tick_dt` median 1372 µs (min 1358, max 1783), IMU `dt` median 1368 µs, `control_execution_us` median 364 µs (max 768), `max_loop_entry_interval_us` median 1764 µs (max 2146).
- So one `loop()` pass takes longer than 1 ms and every pass runs a control tick. About 1 ms of each pass is outside the tick (`loopFOC()` for both wheels, the IMU read, S.BUS, the serial command router); the tick itself takes about 0.36 ms.
- Only 42 of 523 DRIVE rows parsed cleanly at 576000 baud: the rest had missing commas or mixed bytes. The rates above come from monotone counters and are not affected, but the per-row figures rest on a small sample. `K58` output also adds load, so the idle loop may be slightly faster.

Consequence: the IMU biquad filters are designed for `kSampleRateHz` = 1000 but run at about 700 Hz, so their effective cutoffs are about 30% lower than configured. The Mahony filter and the balance controller use measured `dt` and are not affected by the rate itself. The loop timing needs its own fix; the link schedule change does not touch it.
