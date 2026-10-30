# Signal table, parameters, actions and link logs

The firmware names every value that the NavBot link can read, set or call in one file, [signals.def.h](../../../src/ES-02/OllieFOCdrive/signals.def.h). The link serves the table on both ports: the ros2_pc UART1 and the robot's USB port. The wire messages are in the [protocol](ros2-uart-protocol.md#catalog-subscriptions-parameters-and-actions); the PC side is [navlink](navlink.md).

The `K` selectors, the CSV log profiles and the letter commands are gone. The sensor diagnostic image serves the same table; its live entries are on the [sensor diagnostic mode](../../robot/software/sensor-diagnostic-mode.md#which-signals-are-live) page.

## Threads and ownership

- **Receive:** one link task per port decodes frames and pushes requests into one request queue. It never touches robot state.
- **Apply:** the control loop takes the queued requests at the start of each tick and applies them (the sensor diagnostic loop does the same at the start of each pass). It is the only writer of controller state, config and motor outputs, so a tick never sees a half-applied change. Sampling happens in the same task, so sampled rows are never torn.
- **Transmit:** each port has one TX queue and one writer task. The loop, odometry and IMU only push into the queue (non-blocking; a full queue drops and counts). The writer sends a frame only when `availableForWrite()` allows all of it. The loop never writes to a UART. This avoids the stall measured on 2026-10-22, where a blocking 314-byte write gave 4.5 to 7.3 ms loop passes ([lab note](../../../lab/robot/2026-10-22-package1-motor-diagnostic-first-run.md)).

## The entry table

`signals.def.h` is the only place that names a loggable, settable or callable value. An X-macro expands it into the catalog and the sampler's entries (`signal_table.h`, `signal_table_expand.h`).

```cpp
SIGNAL    (roll_deg,         "deg",   imu::latest().correctedRollDegrees)
SIGNAL_MIN(voltage_min_raw_v, "V",    batteryInstantRawV)
SIGNAL_MAX(loop_interval_max_us, "us", loopEntryIntervalUs)

PARAM     (speed_i_on,       "",          BalanceGroup, speedIntegralEnabled)
PARAM_LIVE(speed_kp,         "rad/(m/s)", BalanceGroup, speedKpRadPerMps, 0.05)

ACTION    (motor_current,    startMotorCurrent, ARG(left_a, -MotorDiagnostic::kMaximumCurrentA, MotorDiagnostic::kMaximumCurrentA), ...)
ACTION_NO_ARGS(motor_reset,  resetMotorFault)
```

| Kind | Meaning |
| --- | --- |
| `SIGNAL` | Read-only, streamable. The value at the sampled tick. |
| `SIGNAL_MIN`, `SIGNAL_MAX` | The minimum or maximum since the previous row of the same subscription. The sampler keeps the running value per subscription and resets it on each row, so short spikes that a sampled stream misses are kept: loop-interval peaks, battery dips, servo range windows. |
| `PARAM` | A field of a settings group. Readable, streamable and settable through the group. Refused while balance is requested. |
| `PARAM_LIVE` | A `PARAM` that may also change while balance is requested, by at most its step per `SetParam` (`INFINITY` allows any change). The step is in the entry's units. |
| `ACTION` | A call with bounded arguments. The bounds refer to existing constants, so the table cannot drift from them. |

Rules:

- An entry points at state that already exists: a controller's output or state struct, a global, a settings group. A value that existed only as a local is a member of its controller's state or diagnostics struct.
- The expression of a `SIGNAL` reads existing state each time a subscription samples it. Some expressions are accessor results such as `imu::latest().correctedRollDegrees`, which have no stable address, so the table holds a small function per row, not a pointer.
- Duplicate names and names longer than 24 characters (units 16) fail the build (`constexpr` checks in `signal_table_expand.h`).
- Values are 4 bytes on the wire, typed by the catalog: `f32`, `i32`, `u32` and `bool`. Integers are never sent as `float`, which would lose precision above 2^24 (a `micros()` value after about 17 s).
- The wire ID of an entry is its position in the file. It shifts when a line is added or removed, so the PC resolves names against the catalog of each connection and stores only names.
- The file is named `signals.def.h`, because arduino-cli copies only recognised source and header extensions into the build.

## Parameters: settings groups

A `PARAM` names a field of a settings group. A group is a struct plus one apply function:

| Group | Struct | Apply |
| --- | --- | --- |
| `BalanceGroup` | `Config` | `ApplyBalanceSettings()`: checks the candidate with `configValid()` and swaps it in at the next tick start |
| `ImuGroup` | `imu::Settings` | `imu::setSettings()`, which re-initialises the filters |
| `ServoTrimGroup` | `ServoTrimsDegrees` | `ApplyServoTrims()`: a range check per trim |
| `LogGroup` | `LogSettings` (`minLevel`, 1 to 4) | `Logging::setMinLevel()`; other values are refused with `takes 1 (DEBUG) to 4 (ERROR)` |

`SetParam` copies the group's current struct, changes one field in the copy and calls the apply function. On failure the robot replies with an error and the running values stay as they were. A per-field minimum and maximum alone cannot express cross-field rules such as `trimEnvelopeRad <= tiltRefLimitRad` (`BalanceController.cpp`, `configValid`); without the copy, one bad value could latch `InvalidConfig`, switch the wheels off and drop a balancing robot.

Not exposed as parameters: the `Config` fields that copy motor constants SimpleFOC uses separately (`kvRating`, `phaseResistanceOhm`, the voltage and current limits, the wheel signs). Changing only the copy would make the effort envelope disagree with the drive.

A `PARAM` without a live step is refused while balance is requested, because it feeds the balance controller or a sensor filter. The live parameters (`PARAM_LIVE`) may change while balance is requested, by at most their step per `SetParam`. The steps were chosen by hand and are not yet confirmed on the robot ([roadmap](../plans/roadmap.md#link-autonomy-on-the-robot)).

| Parameter | Default | Live step while balancing |
| --- | --- | --- |
| `speed_kp` | 0.25 | 0.05 |
| `speed_ki` | 0.08 | 0.02 |
| `tilt_p` | 8.0 | 1.0 |
| `gyro_damping` | 0.4 | 0.1 |
| `yaw_p` | 0.6 | 0.15 |
| `speed_cap` | 0.3 | 0.1 |
| `yaw_rate_cap` | 1.0 | 0.25 |
| `log_level` | 2 (INFO) | any change |

All other parameters are refused while balance is requested. `configValid()` still checks every live change, so a live change that breaks a cross-field rule is refused too.

`log_level` (`LogGroup`, `u32`) sets the minimum level of the robot's text lines (`Logging::message`): 1 DEBUG, 2 INFO, 3 WARN, 4 ERROR. It filters those lines only; the `param`, `action` and `link` `Log` frames always go out. It is one global level.

## Actions and safety

- Safety stays in the firmware handlers: `MotorDiagnostic::start` refuses while balance or calibration is active, and every motor test has a lease of at most `kMaximumLeaseMs` (5 s). A test whose lease is not renewed stops, so a lost link stops it too.
- Confirmation is a user-interface step: `navlink shell` shows the checked arguments and asks `y/N` before it sends an action. The firmware has no confirmation tokens.
- `motor_reset` also re-arms the wheel overspeed guard.
- `balance on` (`ARG(on, 0, 1)`) starts or stops balance from the link. It works only with CH5 in position 1 and is refused otherwise, so a switch in another position never starts a robot. Its refusals are in [navlink](navlink.md#parameters-and-actions-on-the-robot).
- `reboot` restarts the robot 300 ms after the `Ack`, in both images. It is refused unless CH5 is at 0 and balance is off, and it is cancelled if balance or motor activity starts in the delay.
- The sensor diagnostic image refuses `motor_current`, `motor_voltage`, `calibrate` and `balance`; `motor_stop`, `motor_reset` and `reboot` run there.
- A started action replies with `Ack` at once and is logged as a `Log` event on each port. Calibration blocks the loop for at least 3 s, and longer when the robot moves; streams pause meanwhile. Calibration is refused while balancing.

## Subscriptions

A subscription is a list of entry IDs and a period in µs.

- Up to 8 subscriptions at the same time, on any port, also several on one port.
- The firmware owns the slots. `Subscribe{slot: 0}` asks for a free one, and the `Ack` returns it in `value`. A slot belongs to the port that subscribed it: only that port renews, replaces or unsubscribes it, and any other port gets `slot in use`.
- Up to 48 entries per subscription, so one row (196 bytes) fits in a frame.
- The period is set in time, not in ticks, because the tick rate is not fixed (424, 521 and about 710 passes per second were measured as the code changed). A row is taken on the first tick at or after the deadline, scheduled like odometry (`link_schedule.h`). `period_us = 0` means every tick.
- Each row carries `t_us`, the `micros()` value at the start of its tick. Rows from different subscriptions taken at the same tick carry the same `t_us`. Rows are not evenly spaced; analysis uses `t_us`.
- Lease: a subscription expires 5 s after its last renewal. A crashed capture stops streaming by itself. An expiry is logged as `slot <n> lease expired` on each port.
- Budget: on `Subscribe` the firmware adds up the subscriptions of each port, `(4 + 4 × entries + 26) × rate`, where 26 bytes is the frame cost of each row. It refuses a subscription that would take the port over 70 % of its line rate, with both numbers in the error. UART1 allows 135.0 kB/s after the 5 kB/s kept for odometry and `Imu` at 2000000 baud. USB allows 40.3 kB/s at 576000 baud. The rest of each line stays free for `Log` and `Ack`.

| Example | Load | UART1 | USB |
| --- | --- | --- | --- |
| 40 entries at 10 Hz (logging) | 1.9 kB/s | fits | fits |
| 8 entries every tick, about 710 Hz | 44.0 kB/s | fits | rejected |
| 20 entries every tick, about 710 Hz | 78.1 kB/s | fits | rejected; fits at a 3 ms period |
| 48 entries every tick, about 710 Hz | 157.6 kB/s | rejected; fits at a 2 ms period | rejected |

The USB port at 576000 baud is the tighter one. A 921600-baud CH340 capture on 2026-10-07 lost rows ([USB/serial facts](../../robot/software/usb-serial-flashing.md)), so the USB rate stays where it is.

## Logs and events

`Logging::message` (used by the firmware for every text line) sends one `Log` frame on UART1 and one on USB, at or above the `log_level` parameter. Each applied `SetParam`, each started action and each lapsed lease is a `Log` on both ports too, so a capture on either port records a change made over the other. The old `Link` stats line, which printed the drop counters every 100 sends, is gone; the counters are signals (below).

The USB port is a backup with the same functions:

- Same frames, same functions, 576000 baud.
- The writer sends a `0x00` before each frame, so no unframed bytes end up in front of a frame. ROM boot output and panic dumps are not framed.
- Opening the CH340 can reset the board ([lab note](../../../lab/robot/2026-09-27-diagnostic-mode-history.md), [lab note](../../../lab/robot/2026-10-25-uart1-ros2-link.md)). Use USB only on a supported or idle robot unless the auto-reset circuit is changed.
- A port held open by `navlink` blocks flashing; close it first.

## Health, balance state and link signals

The signals below are read-only and streamable like the others. The [bridge](navbot-link-bridge.md#drive-and-health-topics) publishes the health set on a 1 Hz subscription (`navbot_link.health.HEALTH_SIGNALS`).

| Signal | Unit | Meaning |
| --- | --- | --- |
| `ch5_position` | | CH5 position from the last S.BUS frame: 0 off, 1 link, 2 RC balance ([protocol](ros2-uart-protocol.md#command)). |
| `rc_valid` | | The RC frame is fresh and the receiver reports no failsafe. |
| `link_balance` | | The link holds balance on (position 1 only). |
| `cmd_speed_mps`, `cmd_turn_rad_s` | m/s, rad/s | The stored `Command`. |
| `cmd_age_ms` | ms | Age of the stored `Command`; `4294967295` before the first one. |
| `balance_state` | | `balanceOutput.state`: 0 Disabled, 1 Arming, 2 Run, 3 Commissioning, 4 Fault. |
| `balance_fault` | | `balanceOutput.fault`: 0 None, 1 StaleImu, 2 StaleWheels, 3 StaleBus, 4 InvalidInput, 5 BadDt, 6 Fall, 7 Overspeed, 8 InvalidEnvelope, 9 InvalidConfig, 10 AuthorityExhausted, 11 TiltSaturated. |
| `overspeed_tripped` | | The wheel overspeed guard is latched. |
| `motor_test_active` | | A motor test is running. |
| `link_tx_drop_uart`, `link_tx_drop_usb` | | Frames dropped because a TX queue was full, per port, since boot. |
| `link_rx_bad_uart`, `link_rx_bad_usb` | | Chunks dropped for a bad COBS structure or CRC, per port, since boot. |
| `link_req_dropped` | | Requests dropped before the control loop, because the request queue was full, since boot. |
| `link_undecodable` | | Requests the endpoint could not decode, since boot. |
| `uptime_s` | s | Seconds since boot. |
| `heap_free_b`, `heap_min_free_b` | B | Free heap now, and the lowest it has been. |
| `stack_free_loop_b` | B | Smallest free stack of the loop task. |
| `stack_free_link_b` | B | Smallest free stack of the link reader and writer tasks. |

The health values are refreshed once a second. The link counters count from boot.

The enum values follow the order of `BalanceController.h`. The PC side names them in [navbot_link.health](../../../src/navbot_link/health.py) (`BALANCE_STATES`, `FAULT_REASONS`).

## Limitations

- Calibration stalls the loop and all streams for at least 3 s.
- Rows are not evenly spaced, and a signal that changes every 20 ms aliases against a sampling period that is not a multiple of it. Use `t_us`.
- USB carries less than a third of UART1: about 10 entries at the full tick rate.
- ROM boot output and panic dumps are not framed.
