# Plan: host simulation of the balancing robot

Status: **proposal, being implemented.** Branch `feat/simulation` in the main checkout, based on `feat/link-firmware` at `9c3c553`. When the plan is implemented, task T7 moves its content into `docs/robot/software/simulation.md` and deletes this file.

Goal: a program on the PC runs the firmware's own balance path against a model of the robot. A change to the controller, its gains or the sketch glue is checked in closed loop before it reaches the robot: sign errors, unstable gains, wrong state changes, faults that do not trip. The checks are host tests.

## Decisions (user, 2026-10-31)

| # | Topic | Decision |
| --- | --- | --- |
| 1 | Physics | Own planar model in C++, no physics engine |
| 2 | Location | `src/sim/` as a new target |
| 3 | Sketch glue | The balance path moves out of `OllieFOCdrive.ino` into `ControlTick.cpp`, which also builds on the host; behavior unchanged |
| 4 | Physical parameters | Measured later by the user; placeholders until then (see [Open](#open)) |
| 5 | Implementation | Parallel Claude Haiku agents at medium effort, plain `general-purpose` agents, no pstack |
| 6 | Agent workflow branch | Not a dependency. `feat/agent-workflow` is being implemented at the same time; it is merged with this branch later and its conflicts are resolved then |

## Why not an existing simulator

| Option | Why not now |
| --- | --- |
| MCU emulators (Wokwi, esp32sim-mcp) | They emulate the chip, not the robot's dynamics |
| MuJoCo | Contact, slopes and 3D, but it needs a model file, a binding to the C++ controller and more parameters to fit. The legs hold one fixed pose, so the plant is planar |
| Gazebo with ros2_control | A ROS-level simulator for SLAM and Nav2. Not installed on the desktop and far more than firmware tests need |

## What runs

Firmware code, compiled unchanged: `BalanceController`, `ControlTick`, `RobotConfig`, `WheelBoundary`, `OverspeedGuard`, `rc_mode.h`, `LegPoseRamp.h`, `imu.cpp`, `MahonyFilter.cpp`, `filter.cpp`, `Logging.cpp`, and for the live link `link_endpoint.cpp`, `link_subscriptions.cpp`, `link_tx.cpp`, `link_frame.cpp`, `link_log.cpp`, nanopb and `signals.def.h`.

The simulation replaces the hardware below them: SimpleFOC (`SimMotor`, `focStep`), the ICM-42688 driver (`initICM42688`, `readIMUDataIfReady`), the AS5600 encoders, the battery ADC, the S.BUS receiver, `micros()`, and the UART (a TCP socket).

Two modes of one binary, `navbot_sim`:

- **Batch**: a scenario file in, a CSV out, faster than real time, with a fixed seed. The tests use this mode.
- **Live**: real time, the link served on `tcp:127.0.0.1:7651` (the multiplexer uses 7650). `navlink`, `run_drive_trial.py` and the ROS bridge connect unchanged.

## Shared names

Every task uses exactly these names. They are committed before the tasks start; a task that needs a change to them stops and reports it.

| File | Content |
| --- | --- |
| `src/ES-02/OllieFOCdrive/RobotConfig.h`, `RobotConfig.cpp` | Motor constants, leg constants, `kRcFrameTimeoutMs`, `kControlTimestepMinSec`, `defaultBalanceConfig()`. Done (moved from the sketch, firmware compiles) |
| `src/ES-02/OllieFOCdrive/ControlTick.h` | `ControlTick::State` and the functions of the balance path. Implemented by T1 |
| `src/sim/sim.h` | Every simulation type and function, and the sketch globals the simulation defines. Implemented by T2 to T6 |
| `src/sim/Arduino.h` | `micros()`, `millis()`, `delay()` for the shared firmware code. Defined in `hal.cpp` (T3) |

Build flags for every host build: `c++ -std=c++17 -Wall -Wextra -Werror -I src/sim -I src/ES-02/OllieFOCdrive`; nanopb with `cc -std=c99` as in `tests/link/run_host_test.sh`. `src/sim` comes first, so `#include <Arduino.h>` finds the shim.

## Model

### Plant (`plant.cpp`)

Coordinates: axle position `x`, body pitch `θ` (forward lean positive), yaw `ψ` (left positive). Wheels roll without slip. With `mb` body mass, `mw` and `Iw` mass and spin inertia of one wheel, `h` centre-of-mass height above the axle, `Jb` body pitch inertia about its centre of mass, `Jz` body yaw inertia, `r` wheel radius, `d` track width, `b` bearing friction, `τL`, `τR` the forward motor torques on the wheels:

```text
wheel speeds    φL' = (v - ψ' d/2) / r        φR' = (v + ψ' d/2) / r
net torques     nL = τL - b (φL' - θ')         nR = τR - b (φR' - θ')
M  = mb + 2 mw + 2 Iw / r²
[ M            mb h cosθ   ] [ x'' ]   [ (nL + nR) / r + mb h sinθ θ'² ]
[ mb h cosθ    Jb + mb h²  ] [ θ'' ] = [ mb g h sinθ - (nL + nR)        ]
Jψ = Jz + 2 mw (d/2)² + 2 Iw (d/2)² / r²
ψ'' = (d / (2 r)) (nR - nL) / Jψ
```

This is the coupled torque model of the [robust control plan](robust-control-system.md#coupled-torque-model) without the small-angle approximation. Integrate with RK4. Store `accelMps2` and `pitchAccelRadPerSec2` of the step. When `|θ|` reaches `groundContactPitchRad`, set `fallen`, clamp `θ`, and zero every rate; a fallen plant no longer moves. While `lifted`, `x`, `θ`, `ψ` and their rates stay 0 and each wheel turns on its own: `Iw φ'' = τ - b φ'`.

`applyPush(P)`: solve the 2×2 matrix above for `[Δv, Δθ'] = A⁻¹ [P, P h cosθ]`, add both, and update the wheel speeds.

### Motor (`motor.cpp`)

SimpleFOC 2.4.0 in torque mode with `estimated_current` (`FOCMotor.cpp`, `move()` and `loopFOC()`), per call with `dt`:

```text
shaft_angle    = sensorAngleRad
shaft_velocity = LPF(Δangle / dt, Tf = kWheelVelocityFilterTfSec)     (0 on the first call)
current_sp     = clamp(target, ±current_limit)
currentQ       = LPF(current_sp, Tf = 1 / (2π · 300))                  (SimpleFOC: DEF_CURR_FILTER_Tf on ESP32)
voltage_bemf   = shaft_velocity / (KV_rating · √3 · 0.10471975512)
voltage.q      = clamp(currentQ · phase_resistance + voltage_bemf, ±voltage_limit)
```

`LPF(x)`: `alpha = Tf / (Tf + dt)`, `y = alpha·y + (1 - alpha)·x`; if `dt > 0.3 s`, `y = x`. While disabled, `focStep` updates only `shaft_angle` and `shaft_velocity`. `disable()` sets `enabled = false` and zeroes `target`, `current_sp`, `currentQ`, `voltage.q` and `voltage.d`, as the sketch's `setWheelEnabled(false)` with `zeroMotorOutput()`.

The real motor (`motorOutput`): `Ke = 1 / (kvRpmPerV · √3 · 0.10471975512)`, `i = (Vq_applied - Ke·w) / R`, torque `= torquePerKe · Ke · i`. A disabled motor carries no current. `w` is the motor-local rotor speed relative to the stator: `motorSign · (φ' - θ')`. Forward torque on the wheel is `motorSign · torque`. `motorSign` is `WheelBoundary::kLeftMotorSign` or `kRightMotorSign`, negated when `flipMotorSign`.

### Sensors (`sensors.cpp`)

IMU axes, fixed to the body: x right, y forward, z up. These reproduce the signs measured on 2026-10-24 (forward lean lowers roll and gives gyro x below 0; a left turn raises gyro z). With `hI` the IMU height and `a = accelMps2`:

```text
ax = a + hI (cosθ θ'' - sinθ θ'²)          (IMU point, world forward)
az = hI (-sinθ θ'' - cosθ θ'²)             (IMU point, world up)
specific force in g:  fy = (ax cosθ - (az + g) sinθ) / g
                      fz = (ax sinθ + (az + g) cosθ) / g
                      fx = -(v ψ') / g
gyro in rad/s:        gx = -θ'   (+θ' with flipGyroX)    gy = -ψ' sinθ    gz = ψ' cosθ
```

Add bias and Gaussian noise, convert to counts with `imu.cpp`'s scales (8 g and 2000 deg/s over 32768), round and saturate to int16. Temperature 0.

Encoder: `round(angle / q) · q` with `q = 2π / encoderCountsPerTurn`. Battery ADC: `round(busV / board::kBatteryVoltsPerAdcCount + noise)`, clamped to 0..4095.

### Battery (`robot.cpp`)

`busV = openCircuitV - internalResistanceOhm · I`, `I = otherLoadA + max(0, Σ 1.5 · Vq_applied · i) / busV`, using the currents of the previous physics step.

### One pass (`Robot::pass`)

1. `t` = the pass start time; set `hal().nowUs = t`; update `loopEntryIntervalUs`.
2. `focStep` on both motors with the encoder reading of `motorSign · (φ - θ)` and `dt` since the previous pass.
3. `imu::service()`. If `rc.framesOn` and `t >= nextRcFrameUs`: `ControlTick::applyRcFrame(control, rc.channels, t)` and advance `nextRcFrameUs` by `framePeriodUs`.
4. `controlTimestepSec = (t - previousControlTimeUs) / 1e6`. If at least `kControlTimestepMinSec`, the control tick in the sketch's order: `controlStartUs = t`; `linkPassStart`; `readBattery`; `updateWheels` with `motor1.shaftAngle()` and `motor2.shaftAngle()`, then the filtered velocities (`LPF`, Tf 0.01 s, into `m1FilteredVelocityRadPerSec` and `m2FilteredVelocityRadPerSec`); `serviceOverspeed` (tripped: `disable()` both); `runBalance` with `rc.signalOk` and `motorTestOwnsOutputs = false`; apply the `WheelCommand` (`Drive`: `enable()` both, then the targets; `Disable`: `disable()` both; `Keep`: nothing); `stepLegs`; `controlExecutionUs = 0`; `linkPassEnd`; `previousControlTimeUs = t`.
5. Physics from `t` to `t + passUs ± jitter` in steps of `physicsStepUs`: battery, motor currents and torques, `stepPlant`, and at each multiple of `imuPeriodUs` a new `imuSample` into `hal()` with `imuReady = true` (a newer sample overwrites an unread one).

### Scenario file

One step per line, `#` starts a comment, blank lines are ignored. Times in seconds, non-decreasing.

| Line | Effect |
| --- | --- |
| `<t> rc <channel 1-18> <raw>` | Sets one S.BUS channel |
| `<t> rc_frames on\|off` | Starts or stops the receiver frames |
| `<t> rc_failsafe on\|off` | The receiver reports failsafe (`signalOk = false`) or not |
| `<t> push <N s>` | `Robot::push` |
| `<t> lift on\|off` | `Robot::setLifted` |
| `<t> battery <V>` | Sets `battery.openCircuitV` |
| `<t> end` | Last line; the run stops at `t` |

### Command line and CSV

```text
navbot_sim --scenario FILE [--out FILE.csv] [--every N] [--set NAME=VALUE ...]
navbot_sim --live --listen HOST:PORT [--scenario FILE] [--set NAME=VALUE ...]
```

Exit codes: 0 done, 2 usage or scenario error (message on stderr). `--out` defaults to stdout; `--every N` writes every N-th control tick (default 1). Log lines go to stderr as `<t_us> <TAG> <text>`.

CSV header, one row per written control tick:

```text
t_us,x_m,v_mps,pitch_rad,pitch_rate_rad_s,yaw_rad,yaw_rate_rad_s,left_wheel_rad_s,right_wheel_rad_s,bus_v,left_current_a,right_current_a,tilt_meas_rad,balance_state,balance_fault,motor1_target_a,motor2_target_a,motors_enabled,overspeed_tripped,lifted,fallen
```

`tilt_meas_rad` is `control.balanceOutput`'s input tilt: `-imu::latest().correctedRollDegrees` in radians. States and faults are the numbers of `BalanceController.h`; booleans are 0 or 1.

## Rules for every agent

1. Work in `/home/niko/Dokumente/Bastelei/roboter/Navbot-ES02` on branch `feat/simulation`. Do not switch branches, commit, push, flash, open a serial port, or `ssh`. Other agents work in the same checkout at the same time: change only the files your task lists. If you need another file, stop and report it. The lead commits each task.
2. Before you start, read `AGENTS.md`, this file up to this section, your task, and the headers of [Shared names](#shared-names). Do not change those headers; report a needed change.
3. No legacy: delete what your task replaces. No compatibility shims, no fallbacks.
4. Match the surrounding code: the sketch's C++ style (`.clang-format` in the sketch folder), host tests as one `run_host_test.sh` per `tests/<area>/` folder that compiles with the flags of [Shared names](#shared-names) and runs a test program built on `assert` with literal expected values, as in `tests/balance_controller/`. Python tests with pytest and literal expected values.
5. Tests use no network except `127.0.0.1`, no serial port and no robot.
6. Run your task's tests and `python3 tests/layout/test_repo_layout.py`, and report the exact pass and fail counts.
7. Change an existing file only with Edit, never by rewriting it whole. No `git checkout`, `git stash`, `git reset`, or formatters over whole files.
8. Report in at most 300 words: files changed, test output, open questions.

## Schedule

| Wave | Tasks, in parallel | Starts when |
| --- | --- | --- |
| 0 | Shared names, this plan (lead) | done |
| 1 | T1 firmware glue, T2 plant and motor, T3 sensors and clock | wave 0 committed |
| 2 | T4 robot and batch runner | T1, T2, T3 committed |
| 3 | T5 live link, T6 scenarios | T4 committed |
| 4 | T7 docs (lead) | T5, T6 committed |

The lead runs each task's tests and the layout test, reviews the diff against this plan, and commits it as `<area>: <what>`. After T1 the lead also builds the firmware with `python3 scripts/build_firmware.py` in a login shell.

## T1: firmware glue (`ControlTick.cpp`)

Files: `src/ES-02/OllieFOCdrive/ControlTick.cpp` (new), `src/ES-02/OllieFOCdrive/OllieFOCdrive.ino`, `src/ES-02/OllieFOCdrive/OllieFOCdrive.h`, `src/ES-02/OllieFOCdrive/signals.def.h`, `tests/link/test_signals_def.cpp`, `tests/link/run_host_test.sh`, `tests/control_tick/run_host_test.sh` (new), `tests/control_tick/test_control_tick.cpp` (new), `tests/control_tick/support/` (new, only if a stub header is needed).

- Implement every function of `ControlTick.h` by moving the sketch's code, unchanged in behavior: `applyRcChannels` → `applyRcFrame`; `rcFrameFresh`; `readBatteryVoltage` without `analogRead`; `updateWheelVelocities` without the two `LowPassFilter` calls; `serviceWheelOverspeed` without the motor writes (the sketch writes them when it returns true); `resetWheelOverspeed`; `balanceRequested`; `linkCommandAgeMs`; `handleLinkCommand` and `setLinkBalance` without the `SENSOR_DIAGNOSTIC_MODE` check; `balanceMeasurements`, `balanceRequest`, `logBalanceChange`, the balance lines of `runControlTick` and the decision of `applyBalanceOutput` → `runBalance`; the leg lines of `runControlTick` → `stepLegs`. `rcStickFraction`, `updateLinkBalance`, the `kRc*` constants, `SBUS_CHANNEL_MIN`/`MAX` and `kTiltSign`, `kTiltRateSign`, `kYawRateSign` move into `ControlTick.cpp` as file-local names.
- The sketch keeps one global `ControlTick::State control;` and deletes every global that moved. It keeps the hardware: `readSbus` calls `applyRcFrame`; `analogRead` then `readBattery`; the velocity filters fill `m1FilteredVelocityRadPerSec` and `m2FilteredVelocityRadPerSec` after `updateWheels`; `applyBalanceOutput` becomes the switch over `WheelAction`; the action and Command handlers keep the `SENSOR_DIAGNOSTIC_MODE` check and call `ControlTick`. `rcSourceValid` uses `ControlTick::rcFrameFresh`. The diagnostic loop sets `control.rcValid`. Delete `robotTumble` and the two `ROBOT_*TUMBLING` defines (assigned, never read).
- `signals.def.h`: change only the expressions of the moved state to `control.<field>` and `ControlTick::linkCommandAgeMs(control, micros())`. Names, units, order and comments stay.
- `tests/link/test_signals_def.cpp`: replace the stand-ins of the moved state with `ControlTick::State control;` and a `micros()` stand-in; link the sources it needs in `tests/link/run_host_test.sh`. The printed table must not change.
- `tests/control_tick/`: tests through the public functions with literal values: CH5 positions from raw 191, 592, 1401, 1402; stick fractions at 192, 992, 1792; a stale frame (100 ms + 1 µs) gives `rcValid` 0 and zero demand; link position: Command accepted only in position 1 after the position was seen, refused texts word for word, expired after `RcMode::kLinkCommandTimeoutUs`; `setLinkBalance` refusals; overspeed trip after 50 ms above 30 rad/s gives `Disable` and enable off until CH5 goes 0 and back; `runBalance` with a motor test gives `Keep`; measurement signs (`correctedRollDegrees` −10 gives tilt +0.1745 rad, gyro x −1 gives tilt rate +1, gyro z +1 gives yaw rate +1); `stepLegs` from 10° reaches 20° after 0.34 s of 1 ms steps and caps a 1 s step at 0.6°.

Tests: `bash tests/control_tick/run_host_test.sh`, `bash tests/link/run_host_test.sh`, `bash tests/balance_controller/run_host_test.sh`.

## T2: plant and motor

Files: `src/sim/plant.cpp`, `src/sim/motor.cpp`, `tests/sim_model/run_host_test.sh`, `tests/sim_model/test_sim_model.cpp`.

Implement the plant and motor functions of `sim.h` exactly as in [Plant](#plant-plantcpp) and [Motor](#motor-motorcpp). Tests with default `PlantParams`:

- Linearization: from `θ = 0.001` at rest without torque, one 1 µs step gives `θ''` within 1 % of `A·θ` with `A = mb g h / D`, `D = Jb + mb h² - (mb h)² / M`; a torque sum `u` at `θ = 0` gives `θ'' = -B u` with `B = (1 + mb h / (r M)) / D`, within 1 %.
- Energy: no torque, no friction, from `θ = 0.3`, `θ' = 0` for 0.2 s of 50 µs steps: kinetic plus potential energy changes by less than 0.1 %.
- Fall: from `θ = 0.1` without torque it reaches `fallen` within 1 s and stays there.
- Yaw: equal and opposite torques ±0.01 N m give `ψ'' = (d / (2 r)) · 0.02 / Jψ` and `x'' = 0`.
- Lifted: torque 0.001 N m on the left wheel gives `φL'' = 0.001 / Iw`; the body stays at 0.
- Push: `applyPush(0.01)` at rest changes `v` and `θ'` to the solution of the 2×2 system.
- Motor: `configureSimMotor` with `kLeftWheelMotor` gives `KV_rating` 143.1, `current_limit` 0.7304; `focStep` at rest with target 0.5 A converges to `voltage.q = 2.875` V; target 2 A is clipped to `voltage.q = 4.2`; a disabled motor keeps `voltage.q = 0`; `appliedVoltageQ` at `voltage.q` 4.2 and bus 8.4 gives 4.2, at bus 6.0 gives 3.0, and a `voltage.q` of 6.0 at bus 8.4 clamps to 4.850; `motorOutput` at 0 V and 100 rad/s with KV 79.5 gives a current of `-100 · Ke / 5.75`.

## T3: sensors and clock

Files: `src/sim/sensors.cpp`, `src/sim/hal.cpp`, `tests/sim_sensors/run_host_test.sh`, `tests/sim_sensors/test_sim_sensors.cpp`.

Implement the sensor functions and `hal()` of `sim.h` as in [Sensors](#sensors-sensorscpp); `hal.cpp` also defines `micros()`, `millis()`, `delay()` (advances `hal().nowUs`), `initICM42688()` and `readIMUDataIfReady()`. Tests, with noise set to 0:

- Upright at rest: accel counts (0, 0, 4096), gyro (0, 0, 0).
- Forward lean 0.1 rad at rest, `imuHeightM` 0: accel y `round(-sin(0.1) · 4096)`, z `round(cos(0.1) · 4096)`.
- Pitch rate +1 rad/s: gyro x `round(-1 · 180/π · 16.384)`; with `flipGyroX` the opposite sign. Yaw rate +1 upright: gyro z `round(180/π · 16.384)`.
- Sign chain through the real `imu.cpp` (link `imu.cpp`, `filter.cpp`, `MahonyFilter.cpp` and a stand-in `Logging::message`): `imu::begin`, then 20000 samples 1 ms apart of a forward lean of 0.1 rad, each through `hal()` and `imu::service()`: `correctedRollDegrees` within 0.5° of −5.73, as on the robot.
- Encoder: 1.0 rad with 4096 counts gives `round(1.0 / q) · q`. Battery: 7.77 V gives 813 counts, 100 V gives 4095.
- `readIMUDataIfReady` returns the sample once after `imuReady`, then false.

## T4: robot and batch runner

Files: `src/sim/robot.cpp`, `src/sim/scenario.cpp`, `src/sim/sim_log.cpp`, `src/sim/main.cpp`, `src/sim/build.sh`, `src/sim/link_server.cpp` (a stub: `runLive` prints `live mode not built` to stderr and returns 2; T5 replaces it), `tests/simulation/conftest.py`, `tests/simulation/test_runner.py`.

- `robot.cpp`: the globals at the end of `sim.h`, `defaultRc()`, and `Robot` as in [One pass](#one-pass-robotpass) and [Battery](#battery-robotcpp). `boot()` sets `balanceSettings = defaultBalanceConfig()`, designs the battery filter with `biquadFilterInitLPF(&control.voltageFilterLpf, 50, 1000)`, sets `control.legPoseDegrees = kLegSagDegrees`, calls `imu::begin(imu::Settings{}, {})`, configures both motors with `configureSimMotor` and disables them.
- `sim_log.cpp`: `LinkPublishLog`, `logLines()`, `logHook`.
- `scenario.cpp`: `parseScenario`, `applyStep`, `setParam` with every field of `Params` and its member structs.
- `main.cpp`: the command line, the CSV and the exit codes of [Command line and CSV](#command-line-and-csv). Batch: boot, then passes until the `end` time, applying each step when the clock reaches its time.
- `build.sh OUT_DIR`: compiles nanopb, the firmware sources of [What runs](#what-runs) and every `src/sim/*.cpp` with the flags of [Shared names](#shared-names) and `-O2`, into `OUT_DIR/navbot_sim`. Prints nothing on success.
- `tests/simulation/conftest.py`: a session fixture `sim_binary` that runs `build.sh` into a temporary folder; a helper `run_sim(binary, scenario_text, *sets) -> (rows, stderr)` with `rows` a list of dicts of floats.
- `test_runner.py`: the CSV header word for word; an empty scenario (`0 end`) exits 0; a bad line exits 2 with `line 1:` on stderr; an unknown `--set` exits 2; 1 s with CH5 off gives `balance_state` 0 in every row and `motors_enabled` 0; two runs with the same seed give identical CSV.

## T5: live link

Files: `src/sim/link_server.cpp` (replaces the stub), `src/sim/sim_signals.cpp`, `tests/simulation/test_live.py`.

- `sim_signals.cpp`: expands `signals.def.h` as the sketch does (`#define SIGNAL_DEF_FILE "signals.def.h"`, `#include "signal_table_expand.h"`) after defining what its expressions name that `sim.h` does not: `systemHealth` (uptime from the clock, other fields 0), `motorDiagnosticState` (`MotorDiagnostic::State{}`), `LinkCountersNow()` (zeros), `BalanceGroup`, `ImuGroup`, `ServoTrimGroup`, `LogGroup` as in the sketch, and the action handlers: `balance` calls `ControlTick::setLinkBalance(control, args[0] >= 0.5f, false, false)`; `motor_current`, `motor_voltage`, `motor_stop`, `motor_reset`, `calibrate` and `reboot` refuse with `not simulated`. The build ID is `sim`.
- `link_server.cpp`: `runLive` listens on `HOST:PORT`, accepts one client at a time, decodes frames with `LinkRxDecoder`, answers with one `LinkEndpoint` over the table (`SetCommandHandler` → `ControlTick::acceptLinkCommand`), sends rows and log lines (`logHook`) as frames, and `WheelOdometry` and `Imu` every 20 ms as `LinkPublishOdometry` in `link_uart.cpp` does. Sets `robot.linkPassStart`/`linkPassEnd`. Runs passes while the simulation clock is behind the wall clock, then sleeps 1 ms. A closed client ends its subscriptions; the next client is accepted.
- `test_live.py`: start `navbot_sim --live --listen 127.0.0.1:<free port>` with a scenario that sets CH5 to position 1 (`rc 5 1000`); with `navbot_link.link.Link("tcp:127.0.0.1:<port>", 0)`: the catalog names equal `scripts/signals_catalog.py`'s parse of `signals.def.h`; `get balance_state` is 0; `call balance on=1` is accepted and `balance_state` becomes 2 within 5 s; while `Link.command(0.1, 0.0)` is sent every 100 ms for 2 s, `get cmd_speed_mps` reads 0.1.

## T6: scenarios

Files: `tests/simulation/test_scenarios.py`.

Each test writes its scenario text, runs it with `run_sim`, and asserts on the rows. Arming needs CH5 off then on (`rc 5 192` at 0, `rc 5 1792` at 0.5).

| Test | Scenario | Pass when |
| --- | --- | --- |
| `test_stand` | arm, end at 10.5 | `balance_state` 2 from 1.0 s on; `|pitch_rad| < 0.035` and `|x_m| < 0.1` from 2 s on; `fallen` 0 |
| `test_drive_step` | arm, `rc 3 1792` at 2, `rc 3 992` at 5, end 9 | `v_mps >= 0.27` before 4.5 s; `|v_mps| < 0.05` after 8 s; travel after 5 s below 1 m; state 2 throughout |
| `test_turn` | arm, `rc 4 192` at 2, `rc 4 992` at 5, end 6 | mean `yaw_rate_rad_s` from 3.5 to 5 s within 20 % of +1.0; `|pitch_rad| < 0.09` |
| `test_push_recovers` | arm, `push 0.05` at 3, end 6 | state 2 throughout; `|pitch_rad| < 0.035` from 5 s on |
| `test_fall_faults` | arm, `push 1.0` at 3, end 4 | `balance_fault` 6 and `motors_enabled` 0 within 0.1 s after `|pitch_rad|` first exceeds 0.611 |
| `test_rc_loss_keeps_balancing` | arm, `rc_frames off` at 3, end 8 | state 2 throughout; `|pitch_rad| < 0.035` from 4 s on |
| `test_ch5_off_disables` | arm, `rc 5 192` at 3, end 3.5 | state 0 and `motors_enabled` 0 within 20 ms after 3 s |
| `test_overspeed_trips` | `lift on` at 0, arm, `rc 3 1792` at 2, end 6 | `overspeed_tripped` 1 and `motors_enabled` 0 by the end |
| `test_low_battery` | arm, `battery 5.5` at 3, end 4 | state 4 with fault 8 by the end |
| `test_flipped_motor_sign_falls` | `--set flipMotorSign=1`, arm, end 4 | `fallen` 1 or state 4 by the end |
| `test_flipped_gyro_sign_falls` | `--set sensors.flipGyroX=1`, arm, end 4 | `fallen` 1 or state 4 by the end |

If a test fails with the placeholder parameters, report the measured values; do not change firmware or placeholders to make it pass. The lead decides.

## T7: docs (lead)

`docs/robot/software/simulation.md` (new): how to build and run, the scenario format, the CSV, the model and its parameters, the test list. Index it in `docs/robot/README.md`, point the roadmap entry to it, delete this plan and its index line.

## Validation

The model is trusted only after it matches the robot. Not a task for the implementing agents.

1. Motor model: replay the voltage steps and sweeps of 2026-10-22 (`lab/robot/captures/2026-10-22-package1-t5…t9`, wheels free) and compare wheel speed. With the fitted motor parameters, `kWheelKvSafetyFactor` 1.0 must run away and 1.8 must slow down, as in the [bench test](../../../lab/robot/2026-10-24-balance-controller-first-bench-test.md).
2. Closed loop: new `navlink capture --preset drive` runs with the current controller: 30 s standing, a forward step, a stop, a turn. Compare the standstill oscillation frequency and amplitude, the step rise time and overshoot, and the stop distance; proposed tolerance 30 %.
3. Record the result in a lab note; fit the placeholders there; then set the T6 limits from the robot.

## Merge with the agent workflow branch

Done later, by hand. Expected conflicts: `AGENTS.md` table and `tests/layout/test_repo_layout.py` (both add entries), `OllieFOCdrive.ino`, `signals.def.h` and `tests/link/test_signals_def.cpp` (its `reset_reason` signal and diagnostic-build guard), and its T14, which changes `balanceRequested()`, now in `ControlTick.cpp`. Its `verify.py` runs `tests/*/run_host_test.sh` and `pytest tests`, so the simulation tests then run for every task.

## Open

- [ ] Measure total mass, wheel mass, centre-of-mass height and swing period (user, later).
