# Balance controller: states, switches, and safety rules

As of 2026-10-24 · firmware commit `776684b` · code in [BalanceController.h](../../../src/ES-02/OllieFOCdrive/BalanceController.h) and [BalanceController.cpp](../../../src/ES-02/OllieFOCdrive/BalanceController.cpp), the balance path of the control tick in [ControlTick.cpp](../../../src/ES-02/OllieFOCdrive/ControlTick.cpp), boot settings in [RobotConfig.cpp](../../../src/ES-02/OllieFOCdrive/RobotConfig.cpp), hardware in [OllieFOCdrive.ino](../../../src/ES-02/OllieFOCdrive/OllieFOCdrive.ino). The same balance path runs in the [host simulation](simulation.md).

## What it does today

The controller is a state machine plus the control math. The sketch gives it measurements and the remote request on every control tick, and applies its output. Since 2026-10-24 the robot stands on its own on the ground. Gains in `balanceSettings` in `OllieFOCdrive.ino` (all uncommissioned starting values found by hand): tilt `tiltP` 8 A/rad, `gyroDamping` 0.4 A/(rad/s), yaw hold `yawP` 0.6 A/(rad/s), speed loop `speedKp` 0.25 rad per m/s and `speedKi` 0.08 rad per m, integral on. The leg servos are fixed at 20 degrees. After boot the commanded angle ramps from the assumed sag angle (10 degrees) to 20 degrees at 30 degrees per second (`stepLegPoseToward` in `LegPoseRamp.h`) after the motor calibration; Arming is not gated on the ramp. The wheel drivers are on in Run and Commissioning. The estimated back-EMF handed to SimpleFOC is the measured KV times `kWheelKvSafetyFactor` (1.8, a compile-time constant); without that margin an enabled wheel at 0 A runs away, see the [bench test note](../../../lab/robot/2026-10-24-balance-controller-first-bench-test.md). Disabled, Arming and Fault keep the drivers off and the wheels free.

What works and is tested on the robot: arming, run, fall detection, fault latching, remote loss, and the commissioning state with the trim knob and the save request.

## Signal flow

The sketch calls the controller once per control tick. It passes the measurements and the remote request, and it gets back two wheel currents, a drive enable and the state. The tick runs when at least 1 ms has passed since the previous tick (`kControlTimestepMinSec`). That is a minimum, not a fixed rate. The measured loop rate is in the roadmap item [Speed up the main loop pass](../plans/roadmap.md#speed-up-the-main-loop-pass).

```mermaid
flowchart LR
    RC[SBUS remote control<br/>CH3 speed · CH4 turn<br/>CH5 enable] --> REQ[Request: speed, yaw rate, switches]
    IMU[ICM-42688P<br/>acceleration + angular rate] --> MAH[Filters + Mahony<br/>corrected roll, gyro rates]
    ENC[2 × AS5600<br/>wheel angle] --> VEL[Wheel speed estimate]
    BAT[Battery ADC<br/>GPIO17] --> BC[BalanceController]
    REQ --> BC
    MAH --> BC
    VEL --> BC
    BC -->|2 current targets, A| FOC[2 × SimpleFOC<br/>torque mode, estimated current]
    ENC --> FOC
    FOC --> DRV[2 × MP6536<br/>wheel motors]
    SERVO[4 × servo PWM<br/>fixed pose]
    DRV --> ROBOT[Mechanics and motion]
    SERVO --> ROBOT
    ROBOT -. measurement .-> IMU
    ROBOT -. measurement .-> ENC
```

- **Attitude.** The IMU is read on every loop pass over SPI. Acceleration and gyro pass a low-pass filter, and Mahony fuses the filtered values with the measured sample interval. The controller uses the corrected roll angle. It uses the unfiltered gyro rates: x for the tilt rate and z for the yaw rate. A forward lean is positive. The IMU design rate is 1000 Hz (`imu::kSampleRateHz`), and the filter cutoffs start at 25 Hz.
- **Wheels.** Each wheel angle comes from its AS5600. The speed estimate divides the angle change by the measured time between two reads. The controller uses this value without smoothing. The sketch also computes a low-pass copy, but nothing reads it.
- **Battery.** GPIO17 is read on every control tick, and a 50 Hz low-pass filter designed for 1000 Hz smooths it. Below 7.4 V (`kLowBatteryVolts`) the status LED blinks faster, and a warning is logged at most once per second. Nothing shuts down.
- **Remote control.** Every loop pass reads the newest S.BUS frame. CH1, CH2, CH8 and CH10 are not read. A frame older than 100 ms, or a receiver failsafe, makes the request invalid. The request then counts as zero speed and zero yaw, and the demand ramps down at the brake cap.
- **Demand.** CH3 sets the speed request, up to 0.3 m/s. CH4 sets the yaw-rate request, up to 1.0 rad/s, left positive. Acceleration and braking are limited. The speed loop turns the speed error into a tilt reference, limited to plus and minus 5 degrees and slewed at 0.5 rad/s.
- **Current.** The common current is the tilt gain times the tilt error, plus the damping gain times the tilt rate. The yaw difference is the yaw gain times the yaw-rate error. The left wheel gets the common current minus the difference, and the right wheel gets the common current plus it. Each wheel is clipped to a current envelope that follows its speed and the bus voltage. Balance has priority. When the envelope leaves no room, the yaw difference is reduced.
- **Motor output.** Both wheels run SimpleFOC in `torque` mode with `estimated_current`. The target current becomes a voltage, `voltage.q = I × R + back-EMF`, limited to 4.2 V (`kWheelVoltageLimitV`). The current is an estimate, not a sensed value. The `current_limit` value, about 0.73 A, is the stall current at the voltage ceiling. It is not a measured limit. The motor sign is applied in `WheelBoundary::toMotorTargets()`.
- **Drive off.** When the controller does not drive (CH5 off, Arming, Disabled or Fault), both targets are zero and both drivers are disabled. The loop keeps running.
- **Legs.** The four servos get 50 Hz PWM and move to one fixed angle, 20 degrees. After boot the commanded angle ramps from 10 degrees to 20 degrees at 30 degrees per second. The servo trims do not apply to this pose, and no inverse kinematics run.
- **Gyro calibration.** The `calibrate` action with step 1 measures the gyro bias. It is refused while balance is requested. Once started, it blocks the control loop for several seconds.

Where to look in the code:

| Question | Location |
| --- | --- |
| Gains and limits | `RobotConfig.cpp`: `defaultBalanceConfig()`; the running copy is `balanceSettings` in `OllieFOCdrive.ino` |
| Loop order and control tick | `OllieFOCdrive.ino`: `loop()`, `runControlTick()` |
| Remote frame and request | `OllieFOCdrive.ino`: `readSbus()`; `ControlTick.cpp`: `applyRcFrame()`, `buildRequest()` |
| Measurements for the controller | `ControlTick.cpp`: `buildMeasurements()` |
| State machine and current allocation | `BalanceController.cpp`: `update()`, `checkRunning()`, `computeEffort()` |
| Wheel outputs | `ControlTick.cpp`: `runBalance()` decides; `OllieFOCdrive.ino`: `applyBalanceOutput()` applies; `WheelBoundary.cpp`: `toMotorTargets()` |
| IMU fusion | `imu.cpp`: `processAccepted()`; `MahonyFilter.cpp` |
| Leg servos | `ControlTick.cpp`: `stepLegs()`; `OllieFOCdrive.ino`: `writeLegServos()`; `ServoControl.cpp` |

## The five states

The sketch logs one `BALANCE state=<n> fault=<n> commissioningRefused=<0|1>` line whenever `state` or `fault` changes.

| `state` | Name | Meaning |
| --- | --- | --- |
| 0 | Disabled | Off. Drivers off, wheels free. Waits for CH5 to go from off to on. |
| 1 | Arming | CH5 went on. The controller checks the start conditions below. When they hold it goes to Run. After 3 s without success it returns to Disabled. |
| 2 | Run | Normal operation. The controller watches for faults and the wheel drivers are on at 0 A (a slight brake). |
| 3 | Commissioning | Standstill setup (CH7 on): trim adjustment and save request. Drive and turn requests are ignored. |
| 4 | Fault | A fault is latched. Drivers off. Only CH5 off leaves this state; the robot never restarts by itself. |

## State diagram

The start arrow marks the initial state, Disabled. The diagram follows `BalanceController::update()` in the code. CH5 off takes every state to Disabled and clears the fault. The request is live when the remote signal is fresh.

```mermaid
stateDiagram-v2
    direction LR
    [*] --> Disabled
    Disabled --> Enabled: CH5 off to on
    Enabled --> Disabled: CH5 off, clears fault
    Arming --> Disabled: timeout 3 s

    state Enabled {
        [*] --> Arming
        Arming --> Run: start conditions met
        Arming --> Fault: invalid envelope or config
        Run --> Commissioning: CH7 on, transfer ok
        Commissioning --> Run: CH7 off, trim matches
        Run --> Fault: any fault
        Commissioning --> Fault: any fault
    }
```

Disabled, Arming and Fault keep the drivers off. Run and Commissioning run the controller with the wheel drivers on; the output is 0 A while all gains are 0. `Enabled` groups the four states that need CH5 on.

- **Disabled to Arming** needs the CH5 edge with a live request, `trimValidated` and `fixedPoseReachable`, and no fault.
- **Arming to Run** needs every start condition in the [start conditions](#start-conditions-arming-to-run) section and an envelope that contains 0 A.
- **Arming to Disabled** after the 3 s timeout, if the start conditions never hold together (drawn as "timeout 3 s").
- **Arming to Fault** on an invalid envelope or config, checked only while the start conditions hold.
- **Run to Commissioning** needs CH7 on and a live request; if the transfer check fails, the state stays in Run with `commissioningRefused=1`.
- **Commissioning to Run** needs CH7 off, a live request, `trimValidated` and a working trim equal to the stored trim.
- **Any fault** from Run or Commissioning goes to Fault: stale or invalid input, bad dt, fall, overspeed, invalid envelope or config, authority exhausted, tilt saturated (see the [faults](#faults-state-4) table).
- **CH5 off** returns every state to Disabled and clears the fault.
- A lost remote leaves Run and Commissioning unchanged; the request counts as zero. It holds Arming, and the timeout still applies.

## Remote switches

| Channel | Function | Notes |
| --- | --- | --- |
| CH3 | Drive request | Forward stick = forward, up to the speed cap. Raw 992 is centre. |
| CH4 | Turn request | Left stick = left turn (positive yaw rate). |
| CH5 | Enable | Three positions. 0 (raw below 592) = off. 1 (592 to 1401) = link mode: balance follows the link's `balance` action (signal `link_balance`), and the link drives with `Command`; see [Command](../../integration/software/ros2-uart-protocol.md#command). 2 (1402 and above) = RC balance on. Only the change from off to on in position 2 starts Arming. Position 2 that is already on at boot, or when the remote reconnects, is ignored: flip CH5 off and on again. |
| CH6 | Save trim | Raw 992 or above = on. Only the change from off to on counts, and only in Commissioning. |
| CH7 | Commissioning | Raw 992 or above = on. Switch it on in Run to enter Commissioning, off to leave. |
| CH9 | Trim knob | Works only in Commissioning, with pickup (below). Raw 992 = trim 0, the end stops are plus and minus 3 degrees. |

## Start conditions (Arming to Run)

All of these must hold at the same time, checked on every tick during the 3 s window:

- IMU, wheel speeds, battery voltage, and the remote signal are fresh.
- Corrected roll is within 5 degrees of the stored trim (stored trim is 0 today).
- Tilt rate is at most 0.1 rad/s.
- Drive and turn sticks are within about 17 percent of centre (0.05 m/s and 0.05 rad/s).
- Both wheels turn at most 2 rad/s.
- The motor model is valid: valid battery range and a current envelope that contains 0 A.

Hold the robot upright and still before switching CH5 on. A lost remote signal blocks Arming too.

## Faults (state 4)

`fault` says why. All are checked every tick in Run and Commissioning; faults 8 and 9 can also latch in Arming once the start conditions hold.

| `fault` | Reason | Trigger |
| --- | --- | --- |
| 1, 2, 3 | Stale IMU, wheels, or battery | Timestamp older than 20 ms (battery 100 ms) |
| 4 | Invalid input | IMU or wheels flagged invalid, or a non-finite value |
| 5 | Time gap | Time between IMU samples is 0 or above 20 ms |
| 6 | **Fall** | Corrected roll beyond 35 degrees for 20 ms, same as the upstream project |
| 7 | Overspeed | A wheel above 30 rad/s for 50 ms |
| 8 | Invalid envelope | Battery out of range, non-finite value, or an inverted current range |
| 9 | Invalid config | A configuration value is unusable |
| 10 | Authority exhausted | The balance effort is limited for more than 2 s |
| 11 | Tilt saturated | The tilt reference is limited for more than 0.5 s |

CH5 off always returns to Disabled and clears the fault. In the sketch, a tripped wheel overspeed guard (30 rad/s for 50 ms) is read as CH5 off, so a runaway usually ends in Disabled instead of Fault 7; flip CH5 off and on to restart.

## Losing the remote

In Run and Commissioning the controller stays in its state. The request counts as zero, the trim is held, and balance would continue. In Disabled and Arming a lost remote blocks starting. When the remote returns, CH5 and CH6 are compared with their last live position: a switch that moved while the signal was lost does not start the robot and does not save.

## Commissioning and the trim

The trim is the small forward or backward lean that balances an uneven payload. It is set at standstill and then saved.

1. **Enter.** In Run, with sticks centred and the robot still, switch CH7 on. The controller refuses with `commissioningRefused=1` if a wheel moves, the tilt is off, the sticks are deflected, or any limit is active. Refusal changes nothing; the controller stays in Run.
2. **Adjust.** Turn CH9. The knob takes over only after its position passes the current trim (pickup), so nothing jumps. After that the trim follows the knob at up to 0.05 rad/s, limited to plus and minus 3 degrees. The trim is not logged and has no visible effect while the wheels are off.
3. **Save request.** Switch CH6 from off to on while in Commissioning. If the robot is still and the conditions hold, the log shows `save requested trim=<deg> deg; storage not wired`. Otherwise it shows `save refused; no pulse`. Nothing is stored yet: the trim storage is [package 6](../plans/robust-control-system.md#package-6-trim-storage). Switching CH7 off and on again does **not** request a save; only the CH6 edge does.
4. **Leave.** Switch CH7 off. The controller returns to Run only when the working trim equals the stored trim. After a changed, unsaved trim it stays in Commissioning until CH5 goes off.

## Measured signs

Checked on the robot on 2026-10-24:

| Movement | Reading | In the controller |
| --- | --- | --- |
| Lean forward | Roll falls, gyro x below 0 | Both are negated, so a forward lean is positive |
| Turn left | Gyro z rises | Left turn is a positive yaw rate |
| Turn a wheel forward | Wheel speed positive | Forward is positive |
| Forward drive current | Negative motor-local sign | Applied by `WheelBoundary::toMotorTargets()` |

## Not done yet

- **Tuning.** The gains above are a hand-tuned set. Driving with the sticks works (turn rate cap 1.0 rad/s). Not tested: long runs and low battery.
- **Gains, trim storage, scheduling** on the IMU rate, and **KV and resistance per wheel**.
- **The remaining starting values.** All `balanceConfig()` values marked `uncommissioned` in the sketch are proposals, not measured.
