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
