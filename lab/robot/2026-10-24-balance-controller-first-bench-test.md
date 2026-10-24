# Balance controller first bench test

Status: open

Date: 2026-10-24

## Observation

Firmware `d2b7ee3`, then fixes up to `776684b`. The robot was supported with the wheels free, all controller gains 0.

1. **Runaway in Run.** With CH5 on (state 2) the wheels ran at full speed after being touched and did not stop until CH5 went off. A tilt did not stop them either.
2. **Same effect in the diagnostic.** `T 0.05 0.05 1500` (0.05 A on both wheels) with `K48` and the battery at about 8 V showed `voltage.q` rising to the 2.0 V limit and the shaft speed reaching 37.6 and 37.9 rad/s within the 1.5 s lease.
3. **Signs.** Leaning forward lowered corrected roll and drove gyro x below 0. Turning left raised gyro z. Turning a wheel forward gave positive wheel speed. CH5 reads 192, 992, and 1792 for positions 0, 1, and 2.
4. **State machine.** All of these matched the expected states: Arming time-out when tilted 30 degrees, Run when upright and still, Fault 6 (fall) when tilted beyond 35 degrees, fault stays after righting the robot, CH5 off then on rearms, remote off keeps Run, the remote switched on with CH5 already on does not arm, a reset with CH5 on does not arm, sticks off centre or a turning wheel keep Arming from reaching Run, and CH7 on with a turning wheel logs `commissioning transfer refused`.
5. **Save request.** CH6 on with CH7 off gives no message. CH6 on in Commissioning logs `save requested trim=…; storage not wired`. Switching CH7 off and on again without CH6 gives no save message.
6. **Wheel diameter** measured as 38.3 mm; 38 mm is used.

## Evidence

User-reported log lines and observations from the bench session in the task chat, plus a capture by the assistant of the `K48` and `T` test (876 rows, battery 7.9 to 8.1 V).

## Practical consequence

- **A wheel at 0 A is not off.** In estimated-current mode SimpleFOC adds the estimated back-EMF as voltage. That is positive feedback, so any error in `KV_rating` accelerates the wheel to the voltage ceiling. At 6.7 V the plateau of about 55 rad/s lies below the 60 rad/s overspeed limit, so the guard never trips. The upstream project used the velocity mode, where a speed target of 0 holds the wheels.
- **Run keeps the wheels off** until the drive is commissioned (`kBalanceDrivesWheels` is `false`). The plan's coast decision is unchanged.
- **Before any gain is set:** choose a back-EMF setting that cannot accelerate the wheel (KV set high enough that the estimate is below the real back-EMF, or no back-EMF term), and lower the overspeed limit below the plateau. Neither is decided.
- The measured signs are now in the sketch: a forward lean is positive, so positive gains drive the wheels forward.

## Update 2026-10-24: KV safety factor, left wheel

The estimated back-EMF handed to SimpleFOC uses the measured KV times a factor (Commander `J`, default now 1.8). The left wheel was pushed by hand with `T 0 0 5000` active and `K7` running, battery about 8 V:

| Factor | Observation |
| --- | --- |
| 1.0 | `T 0.05 0.05`: the wheel ran up to 37 rad/s within the lease |
| 1.4 | After one push the wheel slowed from 45 to 24 rad/s, then sped up again to 30, and later from 17 to 27 rad/s, so the drive kept the wheel turning |
| 1.8 | Two pushes (peaks 38.7 and 45.9 rad/s): the speed fell continuously to 0 both times, with no renewed speed-up at higher speed. After stopping, the wheel turned back to about -3.9 rad/s and died away, probably the motor's cogging torque (not checked) |

Practical consequence: 1.8 is the working value for the left wheel at 0 A. The right wheel has a lower KV (76.7 against 79.5 rpm/V) and takes the same factor on the operator's decision, without a separate test. The wheels stay off in Run (`kBalanceDrivesWheels` is `false`) until the drive is enabled again on purpose.

## Update 2026-10-24: drive enabled again in Run, both wheels

With `kBalanceDrivesWheels` set to `true` and the KV safety factor at 1.8 for both wheels (gains 0), the operator ran the Run tests on the supported robot: wheels turn by hand and brake slightly, a pushed wheel slows down on both sides without a renewed speed-up, no wheel starts from rest, a tilt past 35 degrees gives Fault 6 and the wheels go off, and CH5 off gives Disabled with free wheels. The operator reported that both wheels felt alike. All of these passed.

Practical consequence: the robot now sits in a safe passive state in Run, and the next step is the first non-zero gain with the measured signs. The overspeed limit of 60 rad/s should still be lowered before driving. Status stays open for that reason.

## Update 2026-10-24: first ground test, hand-tuned gains

Wheel overspeed limit lowered to 30 rad/s. With the robot held and then released on the ground, the wheels turned the right way for tilt (hand test with free wheels, 80 rad/s peak tripped the guard). Gains were raised in steps; each row is what the operator saw.

| Firmware | Gains | Observation |
| --- | --- | --- |
| `efa3f8d` | tiltP 4, gyro 0.2, others 0 | Stands briefly, then drifts and falls; turns while driving |
| `6aa5378` | + yawP 0.3, speedKp 0.1 | Turning almost gone, drift much better, still tips slowly in one direction (centre of mass) |
| `1fbf573` | + speedKi 0.05, integral on | Without a cable it ran away twice and tripped the guard; CH5 off/on did not restart (only serial `T RESET` cleared the guard) |
| `b97996d` | speedKp 0.25, speedKi 0.15, CH5 off rearms the guard | Swings 10 to 15 cm, absorbs side pushes |
| `6f5d611` | speedKi 0.08, yawP 0.6 | Same swing, no more turning, slightly more jitter |
| `587ff73` | tiltP 6, gyro 0.3, legs 20 degrees | Swing 2 to 3 cm, less jitter, side pushes still absorbed, no shutdown |
| `b0574c4` | tiltP 8, gyro 0.4 | Still better, adopted as the working set |

Practical consequence: the inner tilt loop was too soft; stiffening it removed most of the swing. Legs at 20 degrees hold the body higher than at 0. Status stays open: driving with the sticks, long runs and the battery effect are untested.
