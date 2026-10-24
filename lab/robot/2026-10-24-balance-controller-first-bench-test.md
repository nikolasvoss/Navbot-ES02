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
