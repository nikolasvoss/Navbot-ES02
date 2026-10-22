# 2026-10-22 Package 1 motor diagnostic: first hardware run

Status: open

## Observation

Firmware: Package 1 working tree on top of c42a34b (uncommitted), flashed over `/dev/ttyUSB0`, robot secured with balance off (`ch5_mode` = 0 in every row). Wheel pose and payload were not recorded.

- The first flash produced DRIVE rows with 48 values against 49 header columns. A format string had one specifier too few, so `sequence` printed the dropped counter and `uart_write_failures` was missing. The fix and a header-width test are in the working tree; the second flash prints 49 columns. The widest DRIVE row is 524 bytes against the 384-byte sender buffer, which is widened to 576.
- `K58` starts the DRIVE trace. Captured rows are about 52 per second (median interval 17 ms), not the nominal 142.9 Hz. `logger_dropped` and `uart_write_failures` stay 0.
- `imu_long_gap_count` grows by about 2 per millisecond of trace and `imu_rate_hz` computes to about 394 Hz against a nominal 1 kHz; the counters need a reading before they are trusted.
- With drivers enabled and no command, `motor1_shaft_velocity` and `motor2_shaft_velocity` jitter by about ±0.8 rad/s and reach about ±3 rad/s. Single-row spikes reached -40 and -65 rad/s on motor 2. Velocity of a wheel with a disabled driver stays frozen at its last value.
- `T 0.01 0.01 500` was accepted, `current_sp` was 0.01 A and `voltage_q` about 0.22 V for roughly one row, then the diagnostic latched fault 5 (overspeed). Every `M` request at 0.05 V and 0.1 V on either wheel also ended within about one to two rows. `T RESET` and `M RESET` cleared the fault and left outputs off.

## Evidence

`captures/2026-10-22-package1-t1.log` (T test under K58), `captures/2026-10-22-package1-t2-idle-messages.log` (T test with message text: `fault=5`), `captures/2026-10-22-package1-t3.log` (M steps under K58).

## Practical consequence

The diagnostic safety path works on the robot: requests are accepted, faults disable both drivers, and reset leaves outputs off. Identification itself is blocked. The 3 rad/s overspeed limit trips on measured velocity noise and spikes before any plateau exists, so no R, KV or q-ceiling data was obtained, and all commissioning rows stay PENDING. Next decision: filter or reject single-sample velocity spikes before the overspeed check, or raise the cap with a reviewed recompile. The row rate and IMU interval counters also need an explanation before they are used as timing evidence.

## 2026-10-22 follow-up: 50 ms overspeed debounce

Firmware with the 50 ms overspeed debounce, CH5 = 0 in every row, robot secured.

- `T 0.01 0.01 500` and `M` at 0.05 V and 0.1 V on each wheel now run the full lease and end with `fault=1` (lease expiry), not overspeed. `shaft_velocity` stays near 0.8 rad/s, the idle noise level: the wheels do not turn at 0.1 V (`captures/2026-10-22-package1-t4-debounce-messages.log`).
- Voltage steps under K58 (`captures/2026-10-22-package1-t5-voltage-steps.log`), 1500 ms lease: motor 1 stays at rest at 0.2 V and 0.3 V and breaks away at 0.4 V; motor 2 stays at rest at 0.2 V and breaks away at 0.3 V. After breakaway the wheel passes 3 rad/s within a few rows (median 5.7 rad/s at 0.4 V and 8.2 rad/s at 0.5 V for motor 1; 5.5, 6.5, 8.9 rad/s at 0.3, 0.4, 0.5 V for motor 2) and the overspeed fault ends the step after 4 to 10 rows. Speed was still rising when the fault stopped it.

Consequence: the filter removes the false overspeed trips, but the 3 rad/s limit ends every step above breakaway before a speed plateau exists, so the KV fit and usable q ceiling remain unmeasured. A higher diagnostic overspeed cap needs a reviewed recompile. The breakaway voltage (about 0.3 to 0.4 V per wheel) is a candidate friction finding and is recorded once, not yet repeated.

## 2026-10-22 follow-up: 15 rad/s cap, voltage steps run to the end of the lease

Firmware commit 241bd11 (overspeed cap 15 rad/s, 50 ms debounce), CH5 = 0 in every row, wheels clear. `M <motor> <V> 2000` under K58; every step ran its full 2000 ms lease with no overspeed fault (`captures/2026-10-22-package1-t6-voltage-steps-15rads.log`). Shaft velocity over the last third of each step, rad/s (median, min to max):

| Motor | 0.3 V | 0.4 V | 0.5 V |
| --- | --- | --- | --- |
| 1 (left) | 0.62 (0.20 to 0.83), at rest | 5.42 (2.24 to 7.86) | 7.78 (4.95 to 10.52) |
| 2 (right) | 3.45 (0.84 to 7.37) | 6.91 (3.59 to 9.30) | 8.81 (7.22 to 11.07) |

Practical consequence: the speed does not settle. Within one step the velocity moves by several rad/s (stick-slip or cogging at these low voltages is a plausible cause, not established), so no step meets the plateau condition of the KV fit in [motor commissioning](../../docs/robot/software/motor-commissioning.md). A straight line through the medians gives roughly 24 rad/s per V on motor 1 (0.4 to 0.5 V) and 27 rad/s per V on motor 2 (0.3 to 0.5 V), about 230 to 260 rpm/V, which would put KV near 130 to 150 rpm/V under the plan's `rpm = KV·√3·V` convention. That is two or three unsteady points with a breakaway offset; it is indicative only and is not entered in the commissioning record. R, the usable q ceiling and the sign check remain pending. The 0.5 V cap is the highest voltage allowed, and speed was still rising with voltage there. Next decision: longer leases are capped at 2000 ms, so settling needs either a velocity filter in the analysis or a reviewed change to the lease or voltage cap.

## 2026-10-22 follow-up: phase resistance measured by hand

The operator measured 11.5 Ω phase to phase on a wheel motor. The wheel, phase pair, meter and winding connection were not recorded. Consequence: this is the first measured resistance. It is below the published 12.5 to 16.4 Ω and about half of the firmware constant (22 Ω). If the winding is star, the per-phase value is 5.75 Ω. Recorded in the [commissioning record](../../docs/robot/software/motor-commissioning.md#commissioning-record); per-wheel confirmation stays pending.

## 2026-10-22 follow-up: 2 V sweep reaches plateaus

Firmware `c309861` (caps 2 V, 5 s, 60 rad/s), CH5 = 0 in every row, wheels clear. `M <motor> <V> 4000` for 0.5, 1.0, 1.5, 2.0 V on each wheel (`captures/2026-10-22-package1-t7-voltage-sweep-2v.log`). All eight steps ran the full lease with no fault; `logger_dropped` and `uart_write_failures` stayed 0 over 3512 rows.

Plateau (median of the last second, rad/s; sd about 1 rad/s) and the 500 ms bin medians show the speed settles within the first 500 ms:

| Voltage | Motor 1 | Motor 2 |
| --- | --- | --- |
| 0.5 V | 8.7 | 8.7 |
| 1.0 V | 18.2 | 18.5 |
| 1.5 V | 25.3 | 25.0 |
| 2.0 V | 30.4 | 29.7 |

Linear fit over the four points with the plan's convention (`KV = slope / √3`): motor 1 79.5 rpm/V, motor 2 76.7 rpm/V, intercept about 2.6 to 3.1 rad/s, residual up to 1.5 rad/s. The curve flattens with voltage (slope 19, 14, 10 rad/s per V between successive points), so the linear KV depends on the range; it is below the 90 rpm/V rating. The cause of the flattening is not established (load, friction and resistive drop at higher current are candidates). Both wheels agree within 4 %. The usable q ceiling is not reached at the 2.0 V cap. Peak control execution was 690 µs and the peak loop interval 7.3 ms, taken without balance load.

## 2026-10-22 follow-up: current command sign check

Firmware `c309861`, CH5 = 0, wheels clear, `T <left_A> <right_A> 3000` under K58 (`captures/2026-10-22-package1-t8-current-test-signs.log`). Each run held its target for the full lease; `voltage_q` was ±0.44 V (0.02 A × the 22 Ω firmware constant). Median shaft velocity, left/right: `T 0.02 0.02` +7.09/+7.34 rad/s; `T -0.02 -0.02` -7.35/-7.44; `T 0.02 -0.02` +7.04/-7.49. Consequence: the current command produces the expected motor-local sign on each wheel independently, and its speed matches the voltage sweep (0.5 V gave 8.7 rad/s). The earlier runs with the 3 rad/s cap showed `fault=5` (overspeed) disabling both drivers, which covers the overspeed gating. The physical rolling direction was still not observed.

## 2026-10-22 follow-up: voltage ceiling found

Firmware `54c04ca` (caps 6 V, 5 s, 120 rad/s), CH5 = 0, wheels clear. `M <motor> <V> 4000` for 3, 4, 5, 6 V on each wheel, no faults (`captures/2026-10-22-package1-t9-voltage-sweep-6v.log`). Plateau median shaft velocity, rad/s (bus voltage 7.4 V at 3 V falling to 7.2 V at 6 V):

| Voltage | Motor 1 | Motor 2 |
| --- | --- | --- |
| 3 V | 39.5 | 37.5 |
| 4 V | 44.3 | 41.4 |
| 5 V | 47.1 | 44.1 |
| 6 V | 47.3 | 44.7 |

Consequence: the speed stops rising between 4 and 5 V; 5 to 6 V adds 0.4 % and 1.4 %. The plan's prediction was supply/√3, which is about 4.2 V at the measured 7.2 to 7.4 V bus and 4.85 V at the 8.4 V constant. The measurement fits the lower value, so `voltage_limit` for Package 2 should follow the real bus voltage, not 8.4 V. The speeds at 3 V and above exceed what the linear low-voltage trend predicts, so the earlier KV fit (77 to 80 rpm/V, 0.5 to 2 V) describes only that range.

## 2026-10-22 follow-up: loop rate and IMU counters explained

Analysis of the 2 V sweep capture (`captures/2026-10-22-package1-t7-voltage-sweep-2v.log`): the loop ran at 424 passes per second, not 1 kHz, so accepted IMU reads (one per pass) came at 424 Hz with a 2.36 ms mean interval and DRIVE rows (every 7th pass) at 60.6 per second. `imu_long_gap_count` is a correct count with a 1.5 ms threshold, and every interval exceeded it, so it carried no information. `imu_max_interval_us` is cumulative since boot. The `imu_dt_median_s` value was phase-locked to the logging gate (1.845 ms against the 2.36 ms mean). Each row window held one pass of 4.5 to 6.7 ms against about 1.9 ms for the others; the cause inferred from the code is the blocking UART write of a 314 byte row (5.4 ms on the wire at 576000 baud) holding the UART lock while `Serial.available()` runs each pass.

Fix: `Serial.setTxBufferSize(1024)` before `Serial.begin` (commit 66417e6). Recapture with the new firmware (`captures/2026-10-22-package1-t10-after-tx-buffer.log`, 446 rows, no faults): maximum loop-entry interval 2.47 ms (was 7.34 ms), maximum IMU interval 2.51 ms (was 7.46 ms), IMU rate 521 Hz, mean interval 1.92 ms, control execution peak 662 µs. The stall is gone, which supports the cause, though the 1 kHz design rate is still not reached: a normal pass takes about 1.9 ms, of which the control block is 0.6 ms. The other 1.3 ms is not profiled (candidates: two AS5600 I²C reads and the IMU SPI transfers). Consequence: filter coefficients designed for 1 kHz sit at about half their design cutoff. Package 7 (IMU filter alignment) must use the measured rate, and a later task has to profile the pass if 1 kHz is wanted.
