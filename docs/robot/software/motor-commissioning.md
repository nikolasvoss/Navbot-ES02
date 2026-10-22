# Motor commissioning: bounded current and voltage identification

This page describes the Package 1 diagnostic commands for the two wheel motors, the manual phase-resistance method, the `KV_rating` fit, and the commissioning record. It is the operating procedure for [Package 1 of the robust control plan](../plans/robust-control-system.md#package-1-measurement-support-controller-unchanged). Software support is implemented; physical acceptance is pending. The only measured values recorded here are the two wheel phase-to-phase resistances; no `KV_rating` or voltage ceiling is recorded yet.

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

`motor_1` is the left wheel and `motor_2` is the right wheel (`motor1` and `motor2` in the trace). The targets reach each motor unchanged, so the sign is the motor-local SimpleFOC direction for each wheel. It is not forward-positive; the boundary sign arrives in Package 2. Compare left and right results only after checking this sign for each wheel.

### Compile-time caps

These caps are in `MotorDiagnostic.h` and mirrored in `scripts/capture_balance_trace.py`. Increasing any cap requires a reviewed recompile.

| Quantity | Cap |
| --- | --- |
| Current per wheel, `T` | ±0.02 A |
| Voltage `q`, `M` | ±2.0 V |
| Lease | integer 1 to 5000 ms |
| Shaft velocity overspeed | 60 rad/s per wheel, latched after 50 ms continuously over the limit. Wheel-clear runs only. |

The 60 rad/s overspeed cap, the 2.0 V voltage cap, and the 5000 ms lease cap apply only to runs with the wheels clear of the surface. On 2026-10-22 the operator confirmed the wheels are clear and approved raising them from 15 rad/s, 0.5 V, and 2000 ms. The earlier 3 rad/s cap ended every identification step: the wheels break away near 0.3 to 0.4 V and pass 3 rad/s before any speed plateau exists. At 15 rad/s the steps ran their full lease, but speed still did not settle; see [the first Package 1 hardware run](../../../lab/robot/2026-10-22-package1-motor-diagnostic-first-run.md). These caps are diagnostic bounds, not commissioned limits.

The 0.02 A current cap is an estimate-based bound, not a measured limit, and it is not derived from the voltage cap. It rests on the 22 ohm `kWheelPhaseResistanceOhm` estimate, which is not measured. At 0.02 A that estimate gives about 0.44 V, below the 2.0 V voltage cap. The value is unchanged.

A `T` request sets both wheel currents. A 0 A target is not an off state: in estimated-current mode the wheel still follows the back-EMF voltage. A `M` request disables the other wheel's driver.

### Lease, faults, and reset

Each request runs until its lease expires, `STOP` is received, or a fault occurs. Faults include lease expiry, invalid or overspeed velocity, balance enabled, and calibration active. Every fault disables both drivers, zeroes both targets, and stays latched.

Overspeed is debounced by time. A wheel counts as over the limit when its shaft velocity magnitude is above 60 rad/s, on either wheel. The fault latches once that condition has lasted 50 ms, measured with the millisecond clock. A sample at or below the limit clears the timer, so a spike shorter than 50 ms is ignored. The −40 and −65 rad/s single-row spikes from the first hardware run were seen one trace row (about 17 ms) apart; their true duration is not measured yet, so whether this rule rejects them on the robot is open. The −65 rad/s spike is above the 60 rad/s cap, so only the 50 ms debounce decides whether it trips. A non-finite velocity still latches `InvalidVelocity` at once. The timer restarts on each new request and clears on `STOP`, `RESET`, and any latched fault.

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

The BOM lists the wheel motors as `2208-90kv` (12N14P gimbal motor, 7 pole pairs). No manufacturer datasheet was found. Community sources give line-to-line resistance (between two phase leads) for this motor class as 12.5 Ω to 16.4 Ω; the [SimpleFOC motor table](https://docs.simplefoc.com/bldc_motors) lists 13 to 16 Ω, and one [forum measurement](https://community.simplefoc.com/t/the-2208-90kv-bldc-has-a-phase-resistance-of-16-4/814) reports 16.4 Ω against an advertised 13.3 Ω. For a star winding the per-phase value is half of the line-to-line value, about 6 to 8 Ω. The firmware constant `kWheelPhaseResistanceOhm` is 22 Ω, which is above the published line-to-line range; its origin is not recorded. The nominal rating is 90 rpm/V. Use these numbers only to judge whether a measurement is plausible. They do not replace the manual measurement below, because listings for this price class are not reliable.

## KV fit

Use voltage identification with wheels clear. Repeat the steps for each wheel at least three voltages, for example 0.5, 1.0, 1.5, and 2.0 V. Keep each lease within the 5000 ms cap.

1. Command one voltage with `M <n> <V> <lease_ms>`, long enough for the shaft velocity to settle to a plateau.
2. Take the median shaft velocity over the plateau, and the commanded `q` voltage. The plateau must be steady: no drift and no rising speed.
3. Convert the speed to rpm: `rpm = ω / RPM_TO_RADS`, where `RPM_TO_RADS = 2π/60`.
4. Fit `rpm` against the voltage across the measured points. The back-EMF convention used for the fit is that from the plan, `V_bemf = ω / (KV · √3 · RPM_TO_RADS)`. So the slope of `rpm` against voltage is `KV · √3`, and `KV_rating = slope / √3`. Verify this convention against the vendored SimpleFOC source before using the value.
5. Use only points where the speed is steady. At these small voltages the resistive term `I · R` is small, but report the residual; do not extrapolate beyond the measured points.

### Usable voltage ceiling

Increase the voltage in steps while speed still rises. The usable ceiling is the highest voltage before the speed stops rising. It is a candidate for `voltage_limit` in Package 2. It must stay within the 2.0 V cap (`kMaximumVoltageV`) until a reviewed recompile changes the cap.

## Commissioning record

Only the two wheel `R_pp` values are measured; every other value is pending or derived. Record each value with its date, firmware commit, and operator, and keep the source log for each one.

| Item | Unit | Status |
| --- | --- | --- |
| Payload and leg-pose description | text | PENDING |
| Firmware commit used for the run | hash | `c309861` (flashed from the clean tree; capture `lab/robot/captures/2026-10-22-package1-t7-voltage-sweep-2v.log`) |
| Left wheel `R_pp`; `R_phase` | Ω | `R_pp` 11.5 Ω MEASURED 2026-10-22 by the operator (phase pair, meter and winding connection not recorded). `R_phase` 5.75 Ω if star (`R_pp / 2`): derived, winding unverified. |
| Right wheel `R_pp`; `R_phase` | Ω | `R_pp` 11.5 Ω MEASURED 2026-10-22 by the operator (phase pair, meter and winding connection not recorded). `R_phase` 5.75 Ω if star (`R_pp / 2`): derived, winding unverified. |
| Left wheel `KV_rating` and fit residual | rpm/V | 79.5 rpm/V from a linear fit over 0.5, 1.0, 1.5, 2.0 V (plateau medians 8.7, 18.2, 25.3, 30.4 rad/s), max residual 1.2 rad/s. The speed curve flattens with voltage, so the fit depends on the range; the 0.5 to 1.0 V pair alone gives about 105 rpm/V. Nominal rating 90 rpm/V. 2026-10-22, operator-run. |
| Right wheel `KV_rating` and fit residual | rpm/V | 76.7 rpm/V, same method (plateau medians 8.7, 18.5, 25.0, 29.7 rad/s), max residual 1.5 rad/s; 0.5 to 1.0 V pair alone about 108 rpm/V. |
| Left wheel usable `q` ceiling | V | At least 2.0 V (the cap): speed still rose from 25.3 to 30.4 rad/s between 1.5 and 2.0 V with no saturation. Not found. |
| Right wheel usable `q` ceiling | V | At least 2.0 V (the cap); 25.0 to 29.7 rad/s between 1.5 and 2.0 V. Not found. |
| Motor sign convention, each wheel, verified against wheel motion | text | Positive q gave positive `shaft_velocity` on both wheels (motor-local SimpleFOC sign). The physical rolling direction was not observed. PENDING for the boundary sign. |
| IMU sample rate and maximum interval from a supported capture | Hz; µs | Wheels-clear diagnostic run, no balance load: maximum IMU interval 7460 µs; the derived rate (424 Hz) and the long-gap counter (24577) disagree with the nominal 1 kHz and are not yet explained. |
| Control execution time and maximum loop interval from a supported capture | µs | Peak control execution 690 µs; peak `loop()` entry interval 7342 µs; DRIVE rows about 62 per second (median 16 ms), `logger_dropped` 0, `uart_write_failures` 0 over 3512 rows. Taken without balance running. |
| Leg pose used (fixed pose check) | text | PENDING |

The measured 11.5 Ω phase-to-phase value lies just below the published 12.5 to 16.4 Ω range. The firmware constant `kWheelPhaseResistanceOhm` (22 Ω) is about twice the measured value and about four times the star per-phase value. The constant is unchanged, so `T` current targets still scale with 22 Ω and the 0.02 A cap was derived from it. Package 2 decides which value the firmware uses and which convention SimpleFOC `phase_resistance` expects.

Until these rows are filled from supported runs, the firmware caps stay in force and no loaded control is commissioned.
