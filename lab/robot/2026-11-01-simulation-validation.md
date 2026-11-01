# Host simulation against the robot

Status: concluded

Firmware `125d9e41` (branch `feat/simulation`, the balance path in `ControlTick.cpp`), boot gains, robot on the floor, battery about 7.8 V. Simulation: [host simulation](../../docs/robot/software/simulation.md).

## Observation

**Measurements (user).** Total mass 680 g; centre of mass of the whole robot 5 cm above the axle; 10 full swings, hanging from the axle, in about 5.2 s. The wheel motor is a 2208 gimbal motor; listings give 39 to 42 g, no 90KV listing was found, so 39 g plus 10 % for the tyre, 43 g per wheel, is used. Derived: body 0.594 kg, body centre of mass 0.0572 m above the axle, swing period 0.52 s, body pitch inertia about its centre of mass `m g h T² / (4π²) - m h²` = 3.39e-4 kg m².

**Captures** (`navlink capture`, 200 Hz, CH5 position 1, link Commands from a script):

- [Standing, 30 s](captures/2026-11-01-sim-validation-stand.csv).
- [Drive profile](captures/2026-11-01-sim-validation-drive.csv): 0.2 m/s for 3 s, stop, 5 s, turn left 0.8 rad/s for 3 s. Speeds are the wheel encoder speeds, which include the body wobble, so both sides are compared after a 0.1 s moving average.
- [First attempt](captures/2026-11-01-sim-validation-drive-hit-wall.csv): the robot hit a wall 0.19 m ahead, 1.5 s into the step. It then pressed against the wall at about 4.8° lean for 14 s, still in Run; no fault tripped.

**Fit.** The simulation replays the profile with equivalent sticks after 30 s of standing (the robot had been balancing for minutes). Swept: real KV, torque per back-EMF constant, bearing friction, IMU height, body inertia, centre-of-mass height, loop pass time, sensor noise, and two terms added for this comparison: an IMU tilt offset and a floor friction on the yaw rate. Result with the measured masses and the fitted values (KV 105 rpm/V, torque 1.0 × Ke × i, IMU offset 0.0192 rad, yaw friction 0.04 N m s, pass 1.77 ms):

| Metric | Robot | Simulation | Difference |
| --- | --- | --- | --- |
| Step: time to 0.18 m/s | 0.70 s | 0.92 s | +31 % |
| Step: cruise speed | 0.239 m/s | 0.242 m/s | +2 % |
| Step: peak speed | 0.296 m/s | 0.251 m/s | −15 % |
| Stop: travel after the stop | 0.104 m | 0.138 m | +33 % |
| Stop: roll-back | 0.075 m | 0.107 m | +43 % |
| Stop: peak backward speed | 0.067 m/s | 0.042 m/s | −38 % |
| Turn: yaw rate at 0.8 rad/s commanded | 0.634 rad/s | 0.640 rad/s | +1 % |
| Standing: mean measured lean | 1.01° | 1.09° | +8 % |
| Standing: tilt standard deviation | 0.095° | 0.020° | −79 % |

The motor fit agrees with the 2026-10-22 sweeps: below about 16 rad/s (the balance range) the plateaus give about 105 rpm/V, and with that value a `kWheelKvSafetyFactor` of 1.0 makes a free wheel at 0 A speed up and 1.8 slow it down, as in the [bench test](2026-10-24-balance-controller-first-bench-test.md). None of the swept values closes the transient differences; they trade against each other.

## Evidence

The captures above; the comparison ran `navbot_sim` on the profile and compared the smoothed speed, the integrated travel, the gyro z rate and the measured tilt in the same time windows.

## Practical consequence

- The simulation matches the steady behavior: cruise speed, turn rate under floor friction, and the standing lean. It is about 30 to 45 % off in the transients: the real robot accelerates faster, wobbles about five times more, and stops with a shorter overshoot but a faster roll-back. The model is good enough to catch sign errors, wrong state changes and gross gain errors, not to tune gains to a few percent.
- The roll-back after a stop is real: 0.075 m at up to 0.067 m/s, settled after about 3 s. The scenario test now checks the simulation against these values.
- With the measured IMU offset the simulation predicts a backward drift of about 0.17 m in the first 10 s after arming, while the speed integral learns the lean. Not yet checked on the robot.
- Pushing against an obstacle does not fault: the robot stayed in Run at about 5° lean for 14 s with the wheels stalled.
