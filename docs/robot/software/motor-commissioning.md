# Motor commissioning: bounded current and voltage identification

This page describes the Package 1 diagnostic commands for the two wheel motors, the manual phase-resistance method, the `KV_rating` fit, and the commissioning record. It is the operating procedure for [Package 1 of the robust control plan](../plans/robust-control-system.md#package-1-measurement-support-controller-unchanged). Software support is implemented, and the commissioning record below holds the measurements from the unloaded runs of 2026-10-22 (robot supported, wheels clear, legs hanging free).

## Preconditions

Run these steps only on a supported robot. Support the body so that a fall cannot drive the wheels. For any run with a wheel on the ground, the wheels must be clear of the surface and the robot must be secured.

- Balance is off: CH5 at position 0. The firmware refuses `T` and `M` requests while balance is enabled, and it latches a fault if balance is enabled during a request.
- Calibration is inactive (`E` not running).
- Both wheels are below the overspeed limit (60 rad/s shaft velocity, wheel-clear runs; see the caps table). A request is refused otherwise.
- The robot is connected over the USB serial console at 576000 baud. Use [trace capture](trace-capture.md) for logs.

## Commands

The commands are sent on the serial console while the trace is running (`K58`) or in Idle. The grammar is exact: single spaces, uppercase letters, and no extra text. The capture script's `--interactive` mode accepts the same grammar.

| Command | Meaning | Unit |
| --- | --- | --- |
| `T <left_A> <right_A> <lease_ms>` | Per-wheel current target for one bounded test | A, motor-local sign |
| `T STOP` | End the current test cleanly; no fault is latched | — |
| `T RESET` | Clear a latched fault; outputs stay off | — |
| `M <motor_1\|2> <voltage_V> <lease_ms>` | Voltage target on the selected wheel only; the other wheel's driver is disabled | V, q-axis, motor-local sign |
| `M STOP` | End the current identification clean; no fault is latched | — |
| `M RESET` | Clear a latched fault; outputs stay off | — |

`motor_1` is the left wheel and `motor_2` is the right wheel (`motor1` and `motor2` in the trace). The targets reach each motor unchanged, so the sign is the motor-local SimpleFOC direction for each wheel. It is not forward-positive; the boundary sign is applied in Package 2 by `toMotorTargets()` and `fromMotorVelocities()` in [WheelBoundary.cpp](../../../src/ES-02/OllieFOCdrive/WheelBoundary.cpp). Compare left and right results only after checking this sign for each wheel.

### Compile-time caps

These caps are in `MotorDiagnostic.h` and mirrored in `scripts/capture_balance_trace.py`. Increasing any cap requires a reviewed recompile.

| Quantity | Cap |
| --- | --- |
| Current per wheel, `T` | ±0.35 A (2.0 V at 5.75 Ω) |
| Voltage `q`, `M` | ±2.0 V |
| Lease | integer 1 to 5000 ms |
| Shaft velocity overspeed | 60 rad/s per wheel, latched after 50 ms continuously over the limit. Wheel-clear runs only. |

The default 60 rad/s overspeed cap, 2.0 V voltage cap, and 5000 ms lease cap apply only to runs with the wheels clear of the surface. On 2026-10-22 the operator lowered the voltage cap back to 2.0 V and the overspeed cap back to 60 rad/s; the lease cap (5000 ms) and the current cap (now 0.35 A, see below) changed. The ceiling sweep to 6 V ran earlier the same day with a temporary recompile at 6.0 V and 120 rad/s (firmware commit `54c04ca`). Repeating that sweep needs the same reviewed recompile, and the default caps must be restored afterwards. The earlier history: the 3 rad/s cap ended every identification step, because the wheels break away near 0.3 to 0.4 V and pass 3 rad/s before any speed plateau exists. The 15 rad/s cap then let the steps run their full lease, but speed still did not settle. See [the first Package 1 hardware run](../../../lab/robot/2026-10-22-package1-motor-diagnostic-first-run.md). These caps are diagnostic bounds, not commissioned limits.

The 0.35 A current cap equals the 2.0 V voltage cap divided by the commissioned 5.75 Ω per phase, so a `T` command at the cap needs about the same voltage as an `M` command at its cap. It is a diagnostic bound, not a measured limit. The operator raised it from 0.02 A on 2026-10-22: the old value was sized for the 22 Ω estimate, and at 5.75 Ω it commanded only about 0.115 V and did not move the wheels (`T 0.02 0.02` gave q = 0.104 V, no motion).

A `T` request sets both wheel currents. A 0 A target is not an off state: in estimated-current mode the wheel still follows the back-EMF voltage. A `M` request disables the other wheel's driver.

### Lease, faults, and reset

Each request runs until its lease expires, `STOP` is received, or a fault occurs. Faults include lease expiry, invalid or overspeed velocity, balance enabled, and calibration active. Every fault disables both drivers, zeroes both targets, and stays latched.

Overspeed is debounced by time. A wheel counts as over the limit when its shaft velocity magnitude is above 60 rad/s, on either wheel. The fault latches once that condition has lasted 50 ms, measured with the millisecond clock. A sample at or below the limit clears the timer, so a spike shorter than 50 ms is ignored. The −40 and −65 rad/s single-row spikes from the first hardware run were seen one trace row (about 17 ms) apart. Under the 60 rad/s cap the −65 rad/s spike is over the limit and the −40 rad/s spike is not. Their true duration is not measured. A non-finite velocity still latches `InvalidVelocity` at once. The timer restarts on each new request and clears on `STOP`, `RESET`, and any latched fault.

- A lease that runs out **latches** a `LeaseExpired` fault. Send `T RESET` or `M RESET` before the next request. `STOP` ends a request without a fault.
- `RESET` clears a latched fault only when balance is off, calibration is inactive, and both wheels are within the overspeed limit. A reset never enables output; a new request is needed.

## Serial-console workflow

`K58` starts the DRIVE trace; `K0` stops it. While a trace runs, the firmware logger suppresses message text. The structured DRIVE rows are then the authoritative record. Result text (`MOTOR_TEST`, `MOTOR_ID_RESULT`) appears only after `K0`.

```text
K58                        DRIVE rows start (balance off, CH5 = 0)
T 0.01 0 500               left +0.01 A, right 0 A, 500 ms lease
...                        rows show motor1_current_sp and motor1_shaft_velocity
T RESET                    lease expired and latched; clear it, outputs stay off
K0                         trace stops; MOTOR_TEST result text is printed
```

For a voltage identification of the right wheel:

```text
K58
M 2 1.0 5000               right wheel only, 1.0 V q-axis, 5000 ms lease
...
M RESET
K0                         MOTOR_ID_RESULT prints requested_q, actual_q, shaft_velocity
```

## Logged capture workflow

Use the capture script so that the log keeps the DRIVE rows and every sent command (`# CMD ...`):

```bash
python3 scripts/capture_balance_trace.py --trace-mode 58 --interactive --seconds 40 --output motor-id-left-01.log
```

Type the bounded commands from the table above when prompted. Anything else is ignored with a message. After the capture ends, the script sends `K0` and keeps the replies for two seconds, so the result text is written to the log. Use a new output filename for each run.

Summarize and check the log with:

```bash
python3 scripts/summarize_balance_trace.py motor-id-left-01.log
python3 scripts/analyze_drive_trace.py motor-id-left-01.log --format json
```

The DRIVE columns for this work are listed in [trace capture](trace-capture.md#capture-a-balance-shutdown). `motor1_current_sp`, `motor1_voltage_q`, and `motor1_shaft_velocity` are the per-wheel values to compare with the commanded values.

## Phase resistance by meter

The firmware does not measure resistance. Measure it by hand with power off:

1. Disconnect the wheel motor's phase wires from the driver, or otherwise isolate them, and keep the robot unpowered.
2. Measure each phase pair (A–B, B–C, C–A) with a four-wire meter if one is available, at room temperature.
3. Record the three values and their mean as `R_pp`.
4. For a star (wye) winding, `R_phase = R_pp / 2`. This is the per-phase, line-to-neutral value. For a delta winding, `R_phase = 1.5 × R_pp`.
5. Record the winding connection, the meter, and which value you put in `phase_resistance`. Verify the winding connection before choosing the formula.

### Published reference values (not measured on this robot)

The BOM lists the wheel motors as `2208-90kv` (12N14P gimbal motor, 7 pole pairs). No manufacturer datasheet was found. Community sources give line-to-line resistance (between two phase leads) for this motor class as 12.5 Ω to 16.4 Ω; the [SimpleFOC motor table](https://docs.simplefoc.com/bldc_motors) lists 13 to 16 Ω, and one [forum measurement](https://community.simplefoc.com/t/the-2208-90kv-bldc-has-a-phase-resistance-of-16-4/814) reports 16.4 Ω against an advertised 13.3 Ω. For a star winding the per-phase value is half of the line-to-line value, about 6 to 8 Ω. The former firmware constant `kWheelPhaseResistanceOhm` was 22 Ω, which is above the published line-to-line range; its origin is not recorded. Package 2 removed it. The nominal rating is 90 rpm/V. Use these numbers only to judge whether a measurement is plausible. They do not replace the manual measurement below, because listings for this price class are not reliable.

## KV fit

Use voltage identification with wheels clear. Repeat the steps for each wheel at least three voltages, for example 0.5, 1.0, 1.5, and 2.0 V. Keep each lease within the 5000 ms cap. Voltages above about 4.8 V are outside the linear range of the modulator at the 8.4 V supply setting; see [Usable voltage ceiling](#usable-voltage-ceiling).

1. Command one voltage with `M <n> <V> <lease_ms>`, long enough for the shaft velocity to settle to a plateau.
2. Take the median shaft velocity over the plateau, and the commanded `q` voltage. The plateau must be steady: no drift and no rising speed.
3. Convert the speed to rpm: `rpm = ω / RPM_TO_RADS`, where `RPM_TO_RADS = 2π/60`.
4. Fit `rpm` against the voltage across the measured points. The back-EMF convention used for the fit is that from the plan, `V_bemf = ω / (KV · √3 · RPM_TO_RADS)`. So the slope of `rpm` against voltage is `KV · √3`, and `KV_rating = slope / √3`. Verify this convention against the vendored SimpleFOC source before using the value.
5. Use only points where the speed is steady. At these small voltages the resistive term `I · R` is small, but report the residual; do not extrapolate beyond the measured points.

### Usable voltage ceiling

Increase the voltage in steps while speed still rises. The usable ceiling is the highest voltage before the speed stops rising. It is a candidate for `voltage_limit` in Package 2. It must stay within the 2.0 V cap (`kMaximumVoltageV`) until a reviewed recompile changes the cap.

How the request is limited, traced through the sketch and the vendored SimpleFOC 2.4.0 source:

- `M` sets the motor target unchanged. In torque mode the target is copied to `current_sp` without a clamp.
- Voltage torque mode clamps `voltage.q` to `motor.voltage_limit`, which the diagnostic sets to `kMaximumVoltageV`. This is the cap clamp. Any request up to 2.0 V passes it (up to 6.0 V in the temporary sweep build).
- The SpaceVectorPWM modulator does not clamp `voltage.q`. It centers the phases on `driver.voltage_limit / 2` and removes the zero-sequence offset. The phase voltages then have to fit between 0 and `voltage_power_supply`. A pure sine above `supply / √3` cannot fit, so the duty cycles saturate at 0 or 1 without a flag.
- At the 8.4 V setting, the largest `voltage.q` the modulator reproduces is about 8.4 / √3 ≈ 4.85 V. A 6.0 V request (possible only in the temporary sweep build) is therefore clipped to a distorted waveform, not applied as 6.0 V. The duty cycle is computed for 8.4 V whatever the real bus voltage is, so a lower real supply (2S, 7.4 V nominal) scales the applied voltage by about 0.88. The applied amplitude at saturation is then about 4.3 V.
- Speed above the saturation point can still rise, so a plateau search can run past the linear range without a visible limit. Measured speed above about 4.8 V is not a clean KV point.

## Commissioning record

Only the two wheel `R_pp` values are measured; every other value is pending or derived. Record each value with its date, firmware commit, and operator, and keep the source log for each one.

| Item | Unit | Status |
| --- | --- | --- |
| Payload and leg-pose description | text | Robot supported on the main body, wheels clear, legs hanging free at servo position 0, no load on the legs (operator, 2026-10-22). No payload other than the installed hardware was recorded. |
| Firmware commit used for the run | hash | `c309861` (flashed from the clean tree; capture `lab/robot/captures/2026-10-22-package1-t7-voltage-sweep-2v.log`) |
| Left wheel `R_pp`; `R_phase` | Ω | `R_pp` 11.5 Ω MEASURED 2026-10-22 by the operator (phase pair, meter and winding connection not recorded). `R_phase` 5.75 Ω if star (`R_pp / 2`): derived, winding unverified. |
| Right wheel `R_pp`; `R_phase` | Ω | `R_pp` 11.5 Ω MEASURED 2026-10-22 by the operator (phase pair, meter and winding connection not recorded). `R_phase` 5.75 Ω if star (`R_pp / 2`): derived, winding unverified. |
| Left wheel `KV_rating` and fit residual | rpm/V | 79.5 rpm/V from a linear fit over 0.5, 1.0, 1.5, 2.0 V (plateau medians 8.7, 18.2, 25.3, 30.4 rad/s), max residual 1.2 rad/s. The speed curve flattens with voltage, so the fit depends on the range; the 0.5 to 1.0 V pair alone gives about 105 rpm/V. Nominal rating 90 rpm/V. 2026-10-22, operator-run. |
| Right wheel `KV_rating` and fit residual | rpm/V | 76.7 rpm/V, same method (plateau medians 8.7, 18.5, 25.0, 29.7 rad/s), max residual 1.5 rad/s; 0.5 to 1.0 V pair alone about 108 rpm/V. |
| Left wheel usable `q` ceiling | V | Speed stops rising near 4 to 5 V. Plateau speeds 39.5 (3 V), 44.3 (4 V), 47.1 (5 V), 47.3 rad/s (6 V); the gain from 5 to 6 V is under 1 %. Candidate `voltage_limit` for Package 2: about 4.2 V (bus 7.2 to 7.4 V divided by √3, the modulation limit); it depends on the battery voltage. 2026-10-22, `captures/2026-10-22-package1-t9-voltage-sweep-6v.log`. |
| Right wheel usable `q` ceiling | V | Same behavior: 37.5 (3 V), 41.4 (4 V), 44.1 (5 V), 44.7 rad/s (6 V); gain from 5 to 6 V about 1.4 %. Candidate about 4.2 V, battery dependent. |
| Motor sign convention, each wheel, verified against wheel motion | text | Positive q gave positive `shaft_velocity` on both wheels (motor-local SimpleFOC sign). The `T` current test agrees on each wheel independently (`T 0.02 0.02` gives +7.1/+7.3 rad/s, `T -0.02 -0.02` gives -7.4/-7.4, `T 0.02 -0.02` gives +7.0/-7.5; `lab/robot/captures/2026-10-22-package1-t8-current-test-signs.log`). Physical rolling direction, observed by the operator on 2026-10-22 with `M 1 0.5 3000` and `M 2 0.5 3000` (robot supported, wheels clear): positive q makes both wheels roll backward in the robot frame. The boundary sign for Package 2 is therefore negative for both wheels: forward = negative motor-local target on each wheel. Both wheels share the same sign. |
| IMU sample rate and maximum interval from a supported capture | Hz; µs | Wheels-clear diagnostic run, no balance load: maximum IMU interval 7460 µs; the derived rate (424 Hz) and the long-gap counter (24577) disagree with the nominal 1 kHz and are not yet explained. |
| Control execution time and maximum loop interval from a supported capture | µs | Peak control execution 690 µs; peak `loop()` entry interval 7342 µs; DRIVE rows about 62 per second (median 16 ms), `logger_dropped` 0, `uart_write_failures` 0 over 3512 rows. Taken without balance running. |
| Leg pose used (fixed pose check) | text | Free-hanging legs at servo position 0, unloaded. This is not a loaded fixed-pose check; that check stays PENDING for Package 2 stage 3. |

The measured 11.5 Ω phase-to-phase value lies just below the published 12.5 to 16.4 Ω range. The former 22 Ω firmware estimate was about twice the measured value and about four times the star per-phase value.

Package 2 decisions, made on 2026-10-22 and implemented in firmware. Only the two `R_pp` readings are measured; the other values below are decisions based on them:

- Phase resistance: `phase_resistance` = 5.75 Ω on both wheels, assuming a star winding (`R_pp / 2`). SimpleFOC uses this as the per-phase, line-to-neutral value in `voltage.q = I · R + back-EMF`. The winding connection is NOT verified.
- KV: `KV_rating` 79.5 rpm/V (left) and 76.7 rpm/V (right), the fit values above, in SimpleFOC's convention. The fit is range dependent.
- Voltage limit: `voltage_limit` 4.2 V on both wheels, the usable ceiling above (battery dependent).
- Current limit: `current_limit` = 4.2 V / 5.75 Ω ≈ 0.73 A on both wheels. This is the stall current at the voltage ceiling, derived from the values above, not separately measured.
- Boundary sign: −1 for both wheels (`kLeftMotorSign`, `kRightMotorSign` in [WheelBoundary.h](../../../src/ES-02/OllieFOCdrive/WheelBoundary.h)). Positive motor q rolls both wheels backward, so forward is a negative motor-local target. Motor signs appear only in `toMotorTargets()` and `fromMotorVelocities()`. Wheel speeds in telemetry are forward-positive and are computed from `motor.shaftAngle()`, which applies each motor's `sensor_direction`. Motor 2's sensor is CW (motor 1 is CCW), so its raw sensor angle moves opposite to `shaft_velocity`. Computing speed from raw sensor angles gave the right wheel the wrong forward sign; the legacy negation on sensor 2 had been compensating for this. See the [lab note of 2026-10-22](../../../lab/robot/2026-10-22-package1-motor-diagnostic-first-run.md).
- Overspeed fault: 30 rad/s since 2026-10-24 (60 rad/s before; the diagnostic cap above stays 60, so the guard trips first), latched after 50 ms over the limit, in every state with the drivers enabled. Implemented in [OverspeedGuard.cpp](../../../src/ES-02/OllieFOCdrive/OverspeedGuard.cpp), it trips on either wheel and disables both drivers. `T RESET` or `M RESET` re-arms it only when both wheels are below the limit.

Verification, wheels clear, on 2026-10-22: a temporary build with the guard limit at 20 rad/s (not the final 60 rad/s build) was used. `M 1 1.5 3000` reached 28.6 rad/s, logged `WHEEL_OVERSPEED`, and disabled both drivers; a following `M 2 1.0` was refused until `M RESET`. `M 2 0.8 2000` gave 15.0 rad/s in torque mode with the commissioned settings. `T 0.02 0.02` gave q = 0.104 V with no wheel motion (see the caps table).

Current-command sign check, wheels clear, on 2026-10-22 with the final 60 rad/s build and the current cap raised to 0.35 A by the operator. `T 0.35 0.35 1500` saturated q at the 2.0 V voltage limit; the estimated-current mode adds back-EMF to I·R, so the voltage limit, not the current, bounds the output. `shaft_velocity` was +33 to +36 rad/s on both wheels, and the forward-positive columns (`motor1_velocity_f`, `motor2_velocity_f`) read -34 and -37 rad/s. Both wheels therefore roll backward for positive motor-local current, matching the operator's earlier `M` observation. `T -0.35 -0.35` gave `shaft_velocity` -37 and -35 rad/s and forward-positive +37.8 and +36.2 rad/s: both wheels roll forward. The earlier build with `T 0.35 -0.35` gave opposite shaft directions, as expected. The overspeed guard at 60 rad/s did not trip; the peak was about 38 rad/s. The first Package 2 build had the right-wheel sign error described under Boundary sign; the fixed `shaftAngle()` build is the one tested here.

Not yet checked: loaded or standing behavior; overspeed at the real 60 rad/s limit (never reached; tested only at 20 rad/s in a temporary build).

Until these rows are filled from supported runs, the firmware caps stay in force and no loaded control is commissioned.
