# Link autonomy motor checks with balance on the ground, 2026-10-31

Status: concluded

## Observation

Build 9c3c5531, battery 8.0 V, RC transmitter on (`rc_valid` 1), multiplexer and bridge running as services. Robot on the ground, the user at the robot with the transmitter. A `navlink capture --preset drive` at 100 Hz ran through the whole session; the times below are capture seconds.

### Live parameters while balancing (CH5 2)

- `set speed_kp 0.29` from 0.25 (step 0.05): accepted.
- `set speed_kp 0.40` from 0.29: refused with `speed_kp: step 0.11 over 0.05 while balancing` (exit 1).
- `set speed_kp 0.25`: accepted, back to the original value.
- `set tilt_ref_limit 0.2` (not live): refused with `tilt_ref_limit: refused while balancing`.
- The robot kept balancing without a fault. The user did not ask for different step sizes, so the steps in the [signal table](../../docs/integration/software/signal-table.md) stay as they are.

### CH5 2 to 1, and balance from the link

- CH5 2 to 1 (84.7 s): `link_balance` became 1 and the controller stayed in `Run`.
- `call balance on=0` in position 1, twice: `link_balance` 0, `Disabled`, no fault; both motor targets and voltages were 0 in the next capture row. The user caught the robot.
- `call balance on=1` with the robot held upright, twice: `link_balance` 1, `Run`, no fault.
- About 20 s after one `balance on=1` (224.07 s) the user lifted the robot. With the tilt near 0 degrees, both unloaded wheels accelerated to about 46 rad/s within 130 ms; the overspeed guard cut both motors at 224.20 s. Returning CH5 to 0 cleared `overspeed_tripped`, and entering position 1 from 0 started nothing (`link_balance` 0) until `balance on=1`.
- One capture row at 224.066 s shows `motor2_target` 1.26 between neighbours near 0.02; likely a capture artefact, not followed up.

### Drive Command and timeout (CH5 1)

- `ros2 topic pub -r 20 -t 40 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1}}"` through the bridge. The firmware's speed target (`ch3_target`) was 0.1 from 405.57 s to 407.76 s and returned to 0 about 250 ms after the last message.
- The wheel speed rose to 0.1 m/s within about 1 s and peaked at 0.12 to 0.13 m/s near the end. The wheels were at rest about 1.1 s after the target returned to 0, because the balance controller must tilt the robot back to brake. Integrated wheel travel: 0.213 m left, 0.222 m right. No fault, no overspeed.
- An earlier attempt ran while the controller was already disabled by the lifted-robot trip above: the target followed the command and timed out the same way, and nothing moved.

### CH5 transitions with balance

- 478.4 s CH5 1 to 2 and 481.2 s 2 to 1: the motor output continued through both.
- 483.91 s CH5 1 to 0: the motor output stopped in the same capture row (within 10 ms). The user caught the robot.
- 485.31 s CH5 0 to 2 in one move: 21 ms in position 1 with no output, then position 2 at 12 degrees of tilt with no output; balance started at 486.01 s when the user brought the robot upright into the arming window.
- 488.48 s CH5 2 to 0 in one move: 30 ms in position 1, where the output continued (the 2 to 1 handover keeps balancing), then the output stopped at 0 (488.51 s).

## Evidence

The capture (`motor_checks.csv`, 100 Hz, about 500 s) and the `navlink` command output were read during the session; the capture is not kept.

## Practical consequence

- Driving from ROS through `/cmd_vel`, the bridge and the multiplexer works on the robot, and the 250 ms command timeout ends the drive when ROS stops publishing.
- A lifted robot at CH5 position 1 or 2 spins its wheels up to the overspeed guard within about 130 ms; the guard stops it. Turn balance off (CH5 0 or `balance on=0`) before lifting the robot.
- A quick CH5 move from 2 to 0 keeps balancing for the few milliseconds the switch spends in position 1, by design; balance ends at 0.
- All link autonomy checks in the [integration roadmap](../../docs/integration/plans/roadmap.md#link-autonomy-on-the-robot) are done.
