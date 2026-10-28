# Logging, telemetry and control over the NavBot link: remaining work

Status: **proposal, partly implemented.** Migration steps 1 to 5 are implemented and host-tested; the checks on the robot for steps 2 to 5 are open. The implemented design is in the [signal table, parameters, actions and link logs](../software/signal-table.md), the [protocol](../software/ros2-uart-protocol.md) and [navlink](../software/navlink.md). What is left is below.

| Step | Status |
| --- | --- |
| 1. Delete the dead yaw path (`Y`, `yawPid`, `yawPidSettings`, `K9`, `K33`, CTRL yaw columns) | Done |
| 2. Firmware request queue, TX queue and writer per port, per-port `seq` | Done; checked on the robot |
| 3. Entry table, catalog, subscriptions on both ports, `navbot_link` library, `navlink`, presets | Done; host-tested; the robot check is open |
| 4. Settings groups and actions replace the letter commands | Done; host-tested; the robot check is open |
| 5. Delete `K`, the profiles, the CSV logger and `Commander`; the analysis scripts read navlink | Done; host-tested; the robot check is open |
| 6. Sensor diagnostic mode | Open |
| 7. ROS bridge: `/rosout`, `/navbot/signals`, parameters, then the dashboard | Open |

## Checks on the robot

Each step ends in a state that a robot run can show. Run these before relying on the step:

- Step 2: 50 Hz odometry is unchanged, and the loop-interval maximum (`loop_interval_max_us`) shows no stalls.
- Step 3: `navlink capture --preset drive` runs on the robot over `/dev/ttyAMA3`, and `navlink shell` lists the catalog and reads the parameters.
- Step 4: `navlink shell` sets a parameter and calls `motor_stop` and `motor_reset`, with the robot supported and balance off.
- Step 5: a `Log` event from a `set` made on one port appears in a capture on the other port, and the USB port shows the same `Log` frames as UART1.

## Step 6: sensor diagnostic mode

Run the link endpoint and the sampler from `diagnosticLoop()`, or delete the mode. Today that build returns from `setup()` before `LinkUartInit()` and never runs the control tick. Step 5 removed the mode's CSV sample, so the mode reports only its boot line, its setup messages and its IMU timing as `Log` frames ([sensor diagnostic guide](../../robot/software/sensor-diagnostic-mode.md)).

## Step 7: ROS bridge

`/rosout` gets the `Log` frames, `/navbot/signals` gets the `SignalBatch` rows, the parameters become ROS parameters, and a dashboard follows. The `Log` path and the signal topic already exist in the [ros2_pc bridge](../software/navbot-link-bridge.md) for the current `Log` and `SignalBatch` frames; the parameters and the dashboard do not. The dashboard tool is an open decision on the [roadmap](roadmap.md#decide-the-dashboard-for-logs-and-debug-signals).

## Open limitation

The firmware does not act on ROS `Command` yet. RC/ROS arbitration and a command timeout are not part of this plan; they belong with the ROS control work.
