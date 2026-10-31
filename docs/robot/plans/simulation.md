# Plan: host simulation of the balancing robot

Status: **proposal, not implemented.** Branch `feat/simulation`, based on `feat/link-firmware` at `9c3c553`. When the plan is implemented, its content moves into `docs/robot/software/simulation.md` and this file is deleted.

Goal: a program on the PC runs the firmware's own control code against a model of the robot. A change to the controller, its gains or the sketch glue is checked in closed loop before it reaches the robot: sign errors, unstable gains, wrong state changes, faults that do not trip. The checks are host tests, so they run in every `pytest tests` run.

## Decisions (user, 2026-10-31)

| # | Topic | Decision |
| --- | --- | --- |
| 1 | Physics | Own planar model in C++, no physics engine |
| 2 | Location | `src/sim/` as a new target |
| 3 | Sketch glue | Move the control tick glue out of `OllieFOCdrive.ino` into a file that also builds on the host; behavior unchanged |
| 4 | Physical parameters | Measured later by the user; placeholders until then (see [Open](#open)) |

## Why not an existing simulator

| Option | Why not now |
| --- | --- |
| MCU emulators (Wokwi, esp32sim-mcp) | They emulate the chip, not the robot's dynamics |
| MuJoCo | Contact, slopes and 3D, but it needs a model file, a binding to the C++ controller and more parameters to fit. The legs hold one fixed pose, so the plant is planar |
| Gazebo with ros2_control | A ROS-level simulator for SLAM and Nav2. Not installed on the desktop and far more than firmware tests need. Revisit when ROS commands drive the robot |

The planar model is the coupled torque model of the [robust control plan](robust-control-system.md#coupled-torque-model), without the small-angle approximation so that falls are real.

## What runs

Firmware code, compiled unchanged:

- `BalanceController`, `WheelBoundary` (including `WheelSpeedEstimator`), `OverspeedGuard`, `rc_mode.h`, `LegPoseRamp.h`, `settings_groups.h`.
- `ICM42688.cpp`, `imu.cpp`, `MahonyFilter.cpp`, `filter.cpp`: the simulated IMU sits behind the SPI mock (as in `tests/imu/`), so the real driver, biquads and Mahony filter run.
- The link: `link_endpoint.cpp`, `link_subscriptions.cpp`, `link_tx.cpp`, `link_frame.cpp`, `link_log.cpp` and the signal table from `signals.def.h` (as in `tests/navbot_link/endpoint_server.cpp`).
- The control tick glue (decision 3): building `Measurements` and `Request` from the raw inputs, the CH5 and link-command handling, the overspeed guard, applying `Output` through `WheelBoundary`, the leg pose ramp, and the tick order of `runControlTick()`. The sketch keeps only the hardware adapters: SimpleFOC, S.BUS, ADC, servos, FreeRTOS.

Simulation code in `src/sim/`:

| Part | Content |
| --- | --- |
| Plant | Axle position, body pitch and rate, both wheel angles and rates, heading. No wheel slip. The motor stator sits on the body, so motor speed and encoder angle are relative to the body. The run ends when the body touches the ground |
| Motor | SimpleFOC estimated-current mode per wheel: target clamped to `current_limit`, `voltage.q = I·R + estimated back-EMF` from the configured KV (measured KV times `kWheelKvSafetyFactor`), clamped to `voltage_limit` and to bus/√3. The real current comes from the real back-EMF; torque from the real KV. This reproduces the runaway at factor 1.0 from the [bench test](../../../lab/robot/2026-10-24-balance-controller-first-bench-test.md) |
| Battery | Open-circuit voltage minus internal resistance times the motor supply current, read through the GPIO17 divider model |
| Sensors | ICM-42688 register file at 1 kHz ODR with noise and bias; AS5600 12-bit angle; ADC counts |
| Timing | The loop of `loop()`: a configurable pass time (default 1.4 ms with jitter, [measured](../../../lab/integration/2026-10-25-odometry-signs-and-rate.md)), a control tick once 1 ms has passed, an IMU read on each pass that accepts only a new sample |
| Inputs | Scripted RC channels (raw S.BUS values, failsafe, frame loss), link `Command`, pushes on the body, parameter changes, sign flips for tests |
| Outputs | CSV in the `navlink capture` format, so `analyze_drive_trace.py` reads it; the state and fault changes as log lines |

Two modes:

- **Batch**: a scenario in, CSV out, faster than real time, with a fixed noise seed. The tests use this mode.
- **Live**: real time, the link served on `tcp:127.0.0.1:7651` (the multiplexer uses 7650). `navlink`, `run_drive_trial.py` and the ROS bridge connect unchanged; `Link` already accepts `tcp:` ports.

## Physical parameters

| Parameter | Value | Source |
| --- | --- | --- |
| Wheel radius | 0.019 m (exact 0.01915 m) | [System overview](../hardware/system-overview.md#key-findings-and-consequences) |
| Track width | 0.113 m; yaw from the wheels reads as 0.128 m | [Turn test](../../../lab/integration/2026-10-26-odometry-turn-test.md) |
| Phase resistance | 5.75 Ω (star winding assumed) | [Motor commissioning](../software/motor-commissioning.md) |
| KV | 79.5 rpm/V left, 76.7 rpm/V right | [Motor commissioning](../software/motor-commissioning.md) |
| Voltage limit | 4.2 V | `kWheelVoltageLimitV` |
| Battery | 2S 18650 | [System overview](../hardware/system-overview.md) |
| Total mass | missing | weigh |
| Wheel mass and inertia, rotor included | missing | weigh one wheel; inertia from its geometry |
| Body centre-of-mass height above the axle | missing | balance the body on an edge |
| Body pitch inertia | missing | time 10 swings with the robot hanging from its axle |
| Yaw inertia | missing | from the geometry |
| Battery internal resistance | missing | voltage dips in existing captures |
| Friction | missing | coast-down of a lifted wheel |

## Scenarios (first version)

Each scenario is a pytest test with literal limits. Proposed limits; they become final after [validation](#validation).

| Scenario | Pass when |
| --- | --- |
| Stand | CH5 off to on: Arming, then Run; 10 s with tilt within 2° and axle drift below 0.1 m |
| Drive step | 0.3 m/s for 3 s, then 0: 0.27 m/s reached within 2 s; stops within 1 m; no fault |
| Turn | 1 rad/s for 3 s: yaw rate within 20 %; tilt within 5° |
| Push | A body impulse the robot can take: back to stand within 2 s |
| Fall | A large push: `Fall` fault, drivers off within 20 ms after the tilt passes 35° plus the debounce |
| RC loss | Frames stop in Run: zero demand, still Run, still balancing |
| CH5 off | Disabled, drivers off |
| Overspeed | Wheels off the ground at speed: guard trips, Disabled |
| Low battery | Falling bus voltage: the configured fault (`AuthorityExhausted` or `InvalidEnvelope`) |
| Sign errors | A flipped wheel sign or gyro sign in a test build: Stand must fail. Proves that the simulation catches sign errors |
| KV factor | Wheels lifted, `motor_current 0 0`: factor 1.0 runs away, 1.8 slows down |
| Link command | CH5 position 1, `Command` 0.1 m/s: drives; commands stop: zero demand within the command timeout |

The whole suite stays below 30 s, so it can run on every task.

Later, only when needed: fault injection (stale IMU, encoder dropouts, dt gaps, voltage dips); odometry against the true pose through the bridge's `/odom`; gain sweeps; plots of a run; the simulation as the `--robot` of the device tests (see below).

Not planned: leg and servo dynamics (the pose is fixed), PWM-level FOC, terrain, LiDAR.

## Validation

The model is trusted only after it matches the robot:

1. Motor model: replay the voltage steps and sweeps of 2026-10-22 (`lab/robot/captures/2026-10-22-package1-t5…t9`, wheels free) and compare wheel speed.
2. Closed loop: new `navlink capture --preset drive` runs with the current controller: 30 s standing, a forward step, a stop, a turn. The existing balance and drive-stop captures come from the old control system and do not count. Compare the standstill oscillation frequency and amplitude, the step rise time and overshoot, and the stop distance; proposed tolerance 30 %.
3. Record the result in a lab note; fit the missing parameters there.

## Relation to the agent workflow plan

The agent workflow plan (`docs/integration/plans/agent-workflow.md` on `feat/agent-workflow`) is compatible with this plan:

- Its task T2 adds a robot roadmap section for this simulation unless this branch already did. This branch adds the section, so T2 drops it (its step O1).
- Its `tests/conftest.py` collects every `tests/*/run_host_test.sh`, and `verify.py` runs `pytest tests` as the `host` stage. The tests in `tests/simulation/` therefore run for every task without a robot. The harness needs only `pytest` and `protobuf` from `tests/requirements.txt`, no NumPy.
- `verify.py` gives `src/sim/**` the `host` stage only. The glue move changes `OllieFOCdrive.ino`, which requires a firmware build, a deploy and tier A and B on the robot.
- Its device tests take `--robot <port>`, and a `tcp:` port works. Later, the live simulation can be that robot: tier B steps that wait for a CH5 flip could run unattended with scripted RC. That needs simulated stand-ins for the health signals (`heap_min_free_b`, `stack_free_*`, `reset_reason`).
- Shared files: its T1 edits the `AGENTS.md` table and `tests/layout/test_repo_layout.py` (S2 adds `sim/` to both); its T6 and T14 edit `OllieFOCdrive.ino`, T14 also `balanceRequested()` and `rc_mode.h` (S1 moves that glue). Merge the agent workflow branch first, or rebase S1 onto it.
- It puts task branches in `.claude/worktrees/<slug>/`. This branch lives in the main checkout until `task.py` exists.

## Tasks

| Task | Files | Starts when |
| --- | --- | --- |
| S1 control tick module | `src/ES-02/OllieFOCdrive/` (new module, `OllieFOCdrive.ino`), `tests/control_tick/` | agent workflow T6 and T14 merged, or rebase (see above) |
| S2 plant and motor | `src/sim/`, `tests/simulation/`, `AGENTS.md` table, layout test | now; unit tests check the linearization against the A and B of the robust control plan |
| S3 sensors and timing | `src/sim/`, `tests/simulation/` | S2 |
| S4 batch runner and scenarios | `src/sim/`, `tests/simulation/` | S1, S3 |
| S5 live mode with the link over TCP | `src/sim/`, `tests/simulation/` | S4 |
| S6 validation | lab note, parameters | S4, measurements done |
| S7 docs | `docs/robot/software/simulation.md`, index, this file deleted | S5 |

S1 changes firmware: build and run it on the robot once (stand, drive, CH5 off) before S4 relies on it.

## Open

- [ ] Measure total mass, wheel mass, centre-of-mass height and swing period (user, later).
