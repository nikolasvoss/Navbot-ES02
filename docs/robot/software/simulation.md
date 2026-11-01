# Host simulation of the balancing robot

As of 2026-10-31 · code in [`src/sim/`](../../../src/sim/), tests in `tests/simulation/`, `tests/sim_model/`, `tests/sim_sensors/` and `tests/control_tick/`.

`navbot_sim` runs the firmware's own balance path on the PC against a model of the robot. It catches sign errors, unstable gains, wrong state changes and faults that do not trip before a robot test. The parameters are measured or fitted against the robot ([validation](../../../lab/robot/2026-11-01-simulation-validation.md)); see [Limits](#limits) for what the model does not match.

## What runs

Firmware code, compiled unchanged: `ControlTick` (the balance path of the control tick: RC and link requests, measurements, the controller, the overspeed guard, the battery filter, the leg pose), `BalanceController`, `RobotConfig` (motor constants and the boot balance settings), `WheelBoundary`, `OverspeedGuard`, `rc_mode.h`, `imu.cpp` with `MahonyFilter` and the biquad filters, `Logging`, and for the live link the link endpoint, nanopb and `signals.def.h`.

The simulation replaces the hardware below them:

| Firmware sees | Simulation (`src/sim/`) |
| --- | --- |
| SimpleFOC `BLDCMotor` | `SimMotor` and `focStep()` in `motor.cpp`: SimpleFOC 2.4.0 torque mode with `estimated_current` |
| ICM-42688 driver | `initICM42688()` and `readIMUDataIfReady()` in `hal.cpp`, samples from `sensors.cpp` at 1 kHz |
| AS5600 encoders, battery ADC | `encoderAngle()` (12 bit), `batteryAdc()` in `sensors.cpp` |
| S.BUS receiver | `RcInput`: raw channels, a frame every 9 ms, failsafe flag |
| `micros()`, `millis()`, `delay()` | the simulation clock in `hal.cpp` |
| UART1 | a TCP socket (`link_server.cpp`) |

`robot.cpp` is the sketch's `setup()` and `loop()` for the balance path: one pass every 1.77 ms (±0.2 ms), a control tick once 1 ms has passed, and the physics in 50 µs steps between passes.

## Build and run

```bash
bash src/sim/build.sh build/sim
```

```bash
build/sim/navbot_sim --scenario stand.txt --out stand.csv
```

```bash
build/sim/navbot_sim --live --listen 127.0.0.1:7651 --scenario link.txt
```

```text
navbot_sim --scenario FILE [--out FILE.csv] [--every N] [--set NAME=VALUE ...]
navbot_sim --live --listen HOST:PORT [--scenario FILE] [--set NAME=VALUE ...]
```

Exit codes: 0 done, 2 usage or scenario error. Log lines go to stderr as `<t_us> <TAG> <text>`. `--set` takes the dotted field names of `sim::Params` in [`sim.h`](../../../src/sim/sim.h), for example `plant.comHeightM=0.07`, `battery.openCircuitV=7.2`, `timing.passUs=1000`, `seed=3`, and the test switches `flipMotorSign=1` and `sensors.flipGyroX=1`.

**Batch** runs faster than real time with a fixed seed: the same scenario and seed give the same CSV. **Live** runs in real time and serves the NavBot link to one TCP client at a time. `navlink`, `run_drive_trial.py` and the ROS bridge connect with the port `tcp:127.0.0.1:7651` (the multiplexer on the ros2_pc uses 7650):

```bash
python3 scripts/navlink.py info --port tcp:127.0.0.1:7651 --json
```

The catalog is the firmware's own, with build ID `sim`. `motor_current`, `motor_voltage`, `motor_stop`, `motor_reset`, `calibrate` and `reboot` are refused with `not simulated`; `balance` and `Command` work as on the robot. In live mode the `end` line is ignored; stop the process.

## Scenario file

One step per line; `#` starts a comment; times in seconds, non-decreasing; the last line is `end`.

| Line | Effect |
| --- | --- |
| `<t> rc <channel 1-18> <raw>` | Sets one S.BUS channel. At start every channel is 992, CH5, CH6 and CH7 are 192 (off) |
| `<t> rc_frames on\|off` | Starts or stops the receiver frames |
| `<t> rc_failsafe on\|off` | The receiver reports failsafe |
| `<t> push <N s>` | A horizontal impulse at the body centre of mass, forward positive |
| `<t> lift on\|off` | Holds the body in the air; the wheels spin free |
| `<t> battery <V>` | Open-circuit battery voltage |
| `<t> end` | The run stops |

Arming needs CH5 off, then on, as on the robot:

```text
0 rc 5 192
0.5 rc 5 1792
2 rc 3 1792    # drive forward at the speed cap
5 rc 3 992
9 end
```

## CSV

One row per control tick (`--every N`: every N-th):

```text
t_us,x_m,v_mps,pitch_rad,pitch_rate_rad_s,yaw_rad,yaw_rate_rad_s,left_wheel_rad_s,right_wheel_rad_s,bus_v,left_current_a,right_current_a,tilt_meas_rad,balance_state,balance_fault,motor1_target_a,motor2_target_a,motors_enabled,overspeed_tripped,lifted,fallen
```

Plant values are the model's truth; `tilt_meas_rad` is the tilt the controller measured; currents and targets are motor-local (SimpleFOC sign). States and faults are the numbers of [`BalanceController.h`](../../../src/ES-02/OllieFOCdrive/BalanceController.h).

## Model

### Plant

Axle position `x`, body pitch `θ` (forward lean positive), yaw `ψ` (left positive); the wheels roll without slip. `mb` body mass, `mw` and `Iw` mass and spin inertia of one wheel, `h` centre-of-mass height above the axle, `Jb` body pitch inertia about its centre of mass, `Jz` body yaw inertia, `r` wheel radius, `d` track width, `b` bearing friction, `τL`, `τR` forward motor torques on the wheels:

```text
wheel speeds    φL' = (v - ψ' d/2) / r        φR' = (v + ψ' d/2) / r
net torques     nL = τL - b (φL' - θ')         nR = τR - b (φR' - θ')
M  = mb + 2 mw + 2 Iw / r²
[ M            mb h cosθ   ] [ x'' ]   [ (nL + nR) / r + mb h sinθ θ'² ]
[ mb h cosθ    Jb + mb h²  ] [ θ'' ] = [ mb g h sinθ - (nL + nR)        ]
Jψ = Jz + 2 mw (d/2)² + 2 Iw (d/2)² / r²
ψ'' = ((d / (2 r)) (nR - nL) - cψ ψ') / Jψ          (cψ: floor friction when turning)
```

This is the coupled torque model of the [robust control plan](../plans/robust-control-system.md#coupled-torque-model) without the small-angle approximation, integrated with RK4. At `|θ|` = 1.2 rad the body touches the ground and the plant stops. Lifted, the body stays upright and each wheel turns on its own.

### Motor

The firmware side mirrors SimpleFOC 2.4.0 `estimated_current`: `current_sp = clamp(target, ±current_limit)`, a current low-pass with `Tf = 1/(2π·300)`, `voltage.q = clamp(current.q·R + shaft_velocity/(KV_rating·√3·0.10472), ±voltage_limit)` with `KV_rating` = measured KV × `kWheelKvSafetyFactor`. The bridge applies `voltage.q · Vbus / 8.4`, limited to `Vbus/√3`. The real motor uses the measured KV: `Ke = 1/(KV·√3·0.10472)`, `i = (Vq - Ke·w)/R`, torque `torquePerKe·Ke·i` (fitted 1.0), with `w` the rotor speed relative to the body. A disabled motor carries no current. The battery sags by its internal resistance times the supply current.

### Sensors

IMU axes fixed to the body: x right, y forward, z up. The IMU reports the specific force at its mounting point and the body rates (gyro x = −θ', gyro z = ψ' cosθ), with noise, quantized to the firmware's ±8 g and ±2000 deg/s. The accelerometer is rotated by the IMU tilt offset, so the robot balances at the measured 1.1° forward lean. These axes reproduce the signs measured on 2026-10-24: a forward lean lowers roll and gives gyro x below 0, and a left turn raises gyro z.

## Parameters

The defaults in [`sim.h`](../../../src/sim/sim.h), from the [validation of 2026-11-01](../../../lab/robot/2026-11-01-simulation-validation.md):

| Parameter | Value | Source |
| --- | --- | --- |
| Wheel radius | 0.01915 m | measured 2026-10-24 |
| Track width | 0.113 m | measured 2026-10-26 |
| Body mass | 0.594 kg | measured: 680 g total minus both wheels |
| Wheel mass, with motor | 0.043 kg | motor listings (39 g) plus 10 % |
| Wheel spin inertia | 4.1e-6 kg m² | estimate from rotor and tyre geometry |
| Body centre of mass above the axle | 0.0572 m | measured: 5 cm for the whole robot |
| Body pitch inertia | 3.39e-4 kg m² | measured: swing period 0.52 s |
| Body yaw inertia | 1.5e-3 kg m² | placeholder |
| Phase resistance | 5.75 Ω | [motor commissioning](motor-commissioning.md) |
| Real KV | 105 rpm/V | fitted; the low-speed KV of the 2026-10-22 sweeps |
| Torque | 1.0 × Ke × current | fitted |
| Floor friction when turning | 0.04 N m s on the yaw rate | fitted to the turn rate |
| IMU tilt offset | 0.0192 rad (1.1°) | fitted to the standing lean |
| IMU height above the axle | 0.03 m | placeholder (no measurable effect) |
| Gyro and accelerometer noise | 0.008 rad/s, 0.008 g | fitted |
| Battery | 7.8 V open circuit, 0.25 Ω, 0.3 A other load | placeholder |
| Loop pass | 1.77 ms ± 0.2 ms | measured 2026-11-01: median control tick 1.84 ms |

The model matches the robot's cruise speed, turn rate and standing lean within 8 %. Its transients differ by 30 to 45 %: the real robot accelerates faster, wobbles more, and rolls back faster after a stop. Use it to catch sign errors, wrong state changes and gross gain errors, not to tune gains to a few percent.

## Tests

```bash
python3 -m pytest -q tests/simulation
```

`verify.py` and `deploy.py` run these tests before a deploy when the simulator's inputs changed; see the [simulation gate](../../integration/software/agent-workflow.md#simulation-gate).

`tests/simulation/test_scenarios.py` runs these closed-loop scenarios with the firmware's boot settings:

| Test | Checks |
| --- | --- |
| `test_stand` | arms and stands 20 s: tilt within 2°; holds its place within 5 cm once the speed integral has learned the standing lean (it drifts back about 0.17 m before) |
| `test_drive_step_rolls_back_like_the_robot` | the robot's drive run: 0.2 m/s cruise within 10 %; stop overshoot, roll-back and backward speed within a factor of two of the robot's |
| `test_turn` | 1 rad/s commanded gives 79 % (the robot's ratio under floor friction) within 15 % |
| `test_push_recovers` | a 0.05 N s push: back within 2° |
| `test_fall_faults` | a fall: drivers off within 0.1 s after 35° |
| `test_rc_loss_keeps_balancing` | frames stop in Run: zero demand, still balancing |
| `test_ch5_off_disables` | Disabled and drivers off within 20 ms |
| `test_overspeed_trips` | lifted wheels at speed trip the guard |
| `test_low_battery` | a sagging battery latches `InvalidEnvelope` |
| `test_flipped_motor_sign_falls`, `test_flipped_gyro_sign_falls` | a sign error makes the robot fall or fault |

`test_live.py` checks the link: the catalog matches `signals.def.h`, `balance on=1` in CH5 position 1 starts balance, and `Command` reaches the controller. `test_runner.py` checks the command line, the CSV and repeatability. `tests/sim_model/` checks the plant against its linearization and energy, and the motor against SimpleFOC; `tests/sim_sensors/` checks the IMU signs through the real `imu.cpp`.

Findings:

- In a fall the spinning wheels trip the overspeed guard 11 ms after the body passes 35°, before the Fall fault latches, so the controller ends in Disabled, not in Fault 6. The drivers are off either way.
- After a stop the robot rolls back, on the robot and in the simulation (see the [validation](../../../lab/robot/2026-11-01-simulation-validation.md)).

## Limits

The transients are 30 to 45 % off the robot (see [Parameters](#parameters)). The body yaw inertia, the IMU height and the battery model are not measured. The backward drift after arming that the model predicts is not checked on the robot.
