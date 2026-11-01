# Plan: fully autonomous driving

Status: **proposal, not implemented.** Branch `feat/autonomous-driving`, created on 2026-10-31 from `feat/simulation` at `c07ead0`. It moves onto `main` once `feat/agent-workflow` and `feat/simulation` are merged there ([P0](#put-the-pieces-on-one-branch-p0)). When a phase is done, its result moves into the `hardware/` and `software/` pages and its section here is deleted. The file goes with the last phase.

The goal is that you pick a goal pose on the saved map in the web hub and the robot drives there by itself. It avoids what the LiDAR sees, and when something it can detect goes wrong, it stops in place and keeps balancing. The transmitter stays on as the emergency stop ([D1](#decisions-user-2026-11-01)).

This page lists the required and the useful features, then orders the required ones into phases P0 to P7. Each phase ends in a check on the host, on the Nav2 test bench, or on the robot. Parallel Haiku agents write the code, as described in [Implement the code with parallel agents](#implement-the-code-with-parallel-agents). The speed limits come from D3. The other start values are derived, not measured, until P2 replaces them.

## What exists today

| Piece | State | Evidence |
| --- | --- | --- |
| Link drive path | CH5 position 1 (link mode), the `balance` action, and `Command` with a 250 ms timeout are in the code on this branch and host-tested. Tier B of the agent workflow tests them on the robot, but it has not run yet. | `rc_mode.h`, `ControlTick.cpp:94-104`, `tests/rc_mode/`, `tests/control_tick/` |
| ROS 2 bridge | Subscribes to `/cmd_vel`. Publishes `/odom`, the TF `odom` to `base_link`, `/imu/data`, `/diagnostics` and `/battery_state`. Runs as a service on the ros2_pc behind the link multiplexer. The boot service takes the heading from the wheels. The odometry covariances are zero. | [bridge page](../software/navbot-link-bridge.md), `systemctl` on the ros2_pc, 2026-10-31 |
| Odometry | Forward sign and left/right mapping are checked. The wheel yaw reads 13.5 % high. The gyro read 1080.4° for three turns, but the bridge's gyro heading has run only on simulated samples. Wheel radius and twist signs are not checked. | [turn test](../../../lab/integration/2026-10-26-odometry-turn-test.md), [integration roadmap](roadmap.md) |
| SLAM | `slam_toolbox` maps from `/scan_slam`, which drops tilted scans and floor hits. One map is saved. Mapping with the robot's wheel odometry is not verified. | [LiDAR software](../../ros2_pc/software/lds08rr-lidar.md), [ros2_pc roadmap](../../ros2_pc/plans/roadmap.md) |
| Localization | AMCL on the saved map, a global search (**Locate**), and a switch between mapping and localization in the web hub. `/scan_slam` also drops scans above 0.8 rad/s or 0.3 m/s. This is on branch `feat/localization`, not merged here, and not verified on the robot. | `feat/localization:docs/ros2_pc/plans/localization.md`, `feat/localization:src/ros2_pc/ros2/src/lds08rr_lidar/lds08rr_lidar/motion_gate.py` |
| Nav2 | Navigation2 1.3.13 for Jazzy is installed on the ros2_pc. That includes the regulated pure pursuit, MPPI and graceful controllers, the velocity smoother, the collision monitor, the waypoint follower, docking and the simple commander. Nothing configures or starts it. `ros-jazzy-nav2-loopback-sim` is in apt but not installed. | `dpkg -l` and `apt-cache policy` on the ros2_pc, 2026-10-31 |
| ros2_pc load | CM5 with 4 cores and 1.9 GiB RAM. With the LiDAR service, SLAM, the bridge and the multiplexer running, 1.29 GiB was available. The bridge used 28 % of one core, and a Claude Code process used 318 MB. | `free -m` and `ps` on the ros2_pc, 2026-10-31 |
| LiDAR | 5 Hz, 1° bins, range capped at 4 m in SLAM and AMCL because a mirror made a phantom room. The scan plane is 0.122 m above the floor. | [LiDAR observations](../../../lab/ros2_pc/2026-10-13-lds08rr-observations.md), `mount.py` |
| Robot limits | Speed cap 0.3 m/s, acceleration 0.5 m/s², braking 1.0 m/s², yaw rate 1.0 rad/s, all uncommissioned. The yaw demand is clamped but not ramped. The wheels are 38.3 mm in diameter. The robot has no bumper, cliff or distance sensor. Footprint, height and mass are not measured. | `RobotConfig.cpp:49-54`, `BalanceController.cpp:180-186`, [robot overview](../../robot/hardware/system-overview.md) |
| Host simulation | Models pitch, axle position and yaw with the firmware's balance path. The live mode serves the link over TCP to one client and accepts `balance` and `Command`. It has no LiDAR, no walls, and placeholder masses. The same branch moved the firmware's balance path into `ControlTick.cpp`, which has not run on the robot yet. On `feat/simulation`, which goes into `main` soon. | [simulation](../../robot/software/simulation.md), [robot roadmap](../../robot/plans/roadmap.md#validate-the-host-simulation) |
| Agent workflow | `scripts/task.py` gives each task a worktree and a `state.md`. `scripts/deploy.py` flashes behind a safety gate and runs tier A. `scripts/verify.py` runs the stages that the changed paths need, including `colcon build` and `colcon test` on the ros2_pc. Hooks block direct uploads and `git worktree add`, and create `agent-` worktrees for subagents. It does not deploy ros2_pc services. On `feat/agent-workflow`, which goes into `main` soon. | `feat/agent-workflow:docs/integration/software/agent-workflow.md` |

## How the robot stops today

A balancing robot has no safe "off" while it stands. With the drivers off, the wheels coast and the robot falls over. The safe stop is zero demand, so the controller keeps balancing in place. Every stop in this plan sends zero velocity or cancels the goal. None of them sends `balance on=0`.

| Event | What the firmware does |
| --- | --- |
| Nav2, the bridge, the multiplexer or the link stops | After 250 ms without a `Command`, the demand is zero and the robot balances in place. |
| The transmitter goes off, or the receiver reports failsafe | The request counts as lost. The robot balances in place and ignores every link `Command` until the RC returns. |
| CH5 goes to position 0 | Balance switches off, the drivers switch off, and the robot falls. This is the emergency stop. |
| The robot falls (35° for 20 ms) | Fault. The drivers switch off and the wheels coast. CH5 to 0 clears the fault. |
| A wheel overspeeds (30 rad/s for 50 ms, 0.57 m/s at the rim) | The trip reads as balance off and the robot falls. CH5 to 0 rearms the guard. |

Sources: `ControlTick.cpp:83-107`, `BalanceController.cpp:441-447`, [balance controller](../../robot/software/balance-controller.md#losing-the-remote).

## Decisions (user, 2026-11-01)

| # | Question | Decision | Consequence for the plan |
| --- | --- | --- | --- |
| D1 | Must the transmitter be on while the robot drives by itself? | Yes, with CH5 in position 1. CH5 to 0 is the emergency stop. The user may change this later. | The firmware already ignores `Command` while the RC is lost, so this needs no firmware change. Driving with the transmitter off stays the later item L1. |
| D2 | Where may it drive? | On the floor of mapped rooms, never on a table. | Stairs and other drops become keepout zones, because the scan plane is 12 cm high and there is no cliff sensor. |
| D3 | How fast? | As fast as the remote: 0.3 m/s and 1.0 rad/s (`speedCapMps` and `yawRateCapRadPerSec` in `RobotConfig.cpp`). | The scan motion gate drops scans above 0.3 m/s and 0.8 rad/s. [P3](#make-odometry-and-localization-hold-while-driving-p3) retunes it at these speeds. |
| D4 | Docking and charging? | No, not now and not later. | No dock, no charging contacts, and no docking server in the launch. |

## Features

### Required

These make the goal work. The last column names the phase that builds each one.

| # | Feature | Why | Phase |
| --- | --- | --- | --- |
| R1 | One branch with the link firmware, the simulation, the localization and the agent workflow | AMCL lives on `feat/localization` and the deploy tools on `feat/agent-workflow`. | P0 |
| R2 | The link drive path proven on the robot, including its failure cases | Every later phase sends `/cmd_vel` through it, and it has never run on the robot. | P1 |
| R3 | Measured drive behaviour: speed tracking, stop distance, rollback, lean while speeding up and braking, and the smallest speed that moves the robot | Nav2's velocity and acceleration limits come from these numbers. | P2 |
| R4 | Measured footprint, height and mass | The costmaps and the collision monitor need the outline. | P2 |
| R5 | Odometry that Nav2 can use: gyro heading on the robot, the wheel radius checked against a tape, and the `/odom` twist signs | The wheel yaw reads 13.5 % high, and AMCL and the controller both build on `odom`. | P3 |
| R6 | AMCL keeps the pose while the robot drives | Nav2 plans in `map`. A pose jump steers the robot at a wall. | P3 |
| R7 | A Nav2 configuration for this robot: planner, controller, velocity smoother, costmaps from `/scan_slam`, recoveries, and composition into one process | The nodes are stock. Only the parameters and the launch are ours. | P4 |
| R8 | A Nav2 test bench with no robot motion, built on `nav2_loopback_sim` and the saved map | Goals, recoveries, keepout zones and the hub can be tested without moving the robot. | P4 |
| R9 | A course script with a preflight check | It turns "works" into numbers (goals reached, final error, stops), and it refuses to start when a precondition fails. | P4, P7 |
| R10 | A CPU and RAM budget for the whole stack on the CM5 | 1.29 GiB is free today, and the Nav2 launch adds ten nodes. | P4, P7 |
| R11 | A safety chain: the collision monitor in front of the bridge, a keepout mask, and an autonomy supervisor that cancels the goal when the robot cannot follow it | The controller does not know about RC loss, balance faults or a lost pose. | P5 |
| R12 | A navigation mode in the web hub, supervised like SLAM: set a goal on the map, see the path, cancel, stop | The desktop has no ROS and no RViz, and the hub already shows the map. | P6 |

### Useful

These make the robot nicer to use or easier to debug. None of them blocks the goal. The list is in the suggested order after P7.

| # | Feature | Why | Builds on |
| --- | --- | --- | --- |
| U1 | A recording of every autonomous run: a rosbag2 of `/scan`, `/scan_slam`, `/tf`, `/odom`, `/cmd_vel`, `/diagnostics` and the Nav2 status, plus a `navlink capture` | A failed run can be replayed and compared with a good one. | P7 |
| U2 | Waypoints and named places, such as "go to the desk" or a patrol loop, with the stock waypoint follower | The hub sends a list instead of one goal. | P6 |
| U3 | Automatic relocalization. When AMCL reports `lost`, the supervisor cancels the goal, runs the existing global search, and offers to resume. | Today a lost pose needs a click on **Locate**. | P5 |
| U4 | Speed zones with the Nav2 speed filter | The robot slows down in narrow passages and at doors. | P5 |
| U5 | Several named maps, already on the [ros2_pc roadmap](../../ros2_pc/plans/roadmap.md) | One map per room or floor. | P6 |
| U6 | A battery rule that refuses goals below a voltage | The bridge publishes the voltage. The threshold needs a measured discharge curve under load. | P5 |
| U7 | The host simulation in the loop: the bridge and the Nav2 controller against `navbot_sim --live` | It tests the drive chain with the firmware's balance path before the robot does. Its numbers count only after the [simulation is validated](../../robot/plans/roadmap.md#validate-the-host-simulation). | P4 |
| U8 | Drawing keepout and speed zones in the hub | P5 uses an edited map image, which works but needs an image editor. | P6 |

### Later

These need new hardware or a new decision.

| # | Feature | What it needs |
| --- | --- | --- |
| L1 | Driving with the transmitter off (D1) | A firmware rule that makes the link the dead man in position 1, for example a heartbeat from the supervisor, and a safety review before any code. |
| L2 | Exploration, where the robot maps an unknown room by itself | Jazzy has no apt package for it (`apt-cache search` on the ros2_pc, 2026-10-31). It needs a source package or an own frontier node. |
| L3 | Down-facing distance sensors against drops, and a bumper | The LiDAR misses drops and everything below 12 cm, such as cables and door sills. |
| L4 | People detection with the HMMD radar | The radar publishes a raw range-Doppler map only, and its wiring is not verified. |

## Put the pieces on one branch (P0)

P0 starts after the user has merged `feat/agent-workflow` and `feat/simulation` into `main`. From then on, the agent workflow's rules in `AGENTS.md` apply to this work.

1. Rebase `feat/autonomous-driving` onto `main`. Its own commits change only this plan and the integration index.
2. Run `python3 scripts/task.py start feat/autonomous-driving` and work in the worktree it prints. Copy the goal, the decisions and the done-when lists of this plan into the Contract and Plan sections of `state.md`.
3. Merge `feat/localization`. Since `main` it changed only `lds08rr_lidar`, its tests, its docs and `AGENTS.md`. In `AGENTS.md`, keep the version from `main`, which already has the localization branch's two opening paragraphs.
4. Decide whether the mapping undo goes in. `feat/undo-backend` and `feat/undo-ui` build on `feat/localization` and are merged nowhere. They change `slam.py`, `web.py`, `slam.html` and `test_slam.py`, which T5 and T6 change too. Merge them here before wave 1, or leave them out.
5. Do task T8 of [link-autonomy.md](link-autonomy.md). Its status line says "not implemented", and [signal-table.md](../software/signal-table.md) and the [integration roadmap](roadmap.md) still say that the firmware ignores `Command`.
6. Flash the firmware of `main` with `python3 scripts/deploy.py`, with the transmitter off. The deploy also runs tier A.
7. On the ros2_pc, install `navbot-lidar.service` from this branch's worktree with `scripts/install_lidar_service.py`, so that the robot runs the localization code. Check for an unsaved map first ([rule 3](#rules-for-every-agent)).

Done when:

- `python3 scripts/verify.py` reports `done` for the merge. Its stages are `host` and `ros2_pc`, because only docs and `src/ros2_pc/` changed since `main`.
- The deploy passed the boot check and tier A.
- The hub on port 8081 shows the localization mode switch.

## Prove the link drive path on the robot (P1)

You are at the robot with the transmitter on. This is also the first robot run of the control tick that the simulation branch moved into `ControlTick.cpp`.

1. Run tier B with `python3 -m pytest tests/hil/test_tier_b.py --robot --guided`. It checks the CH5 positions, balance in position 1, `balance` off and on through the link, and the stop after the `Command` timeout.
2. Drive at 0.1 m/s with `teleop_twist_keyboard` in position 1, then stop the publisher. The robot stops and keeps balancing. This checks `/cmd_vel` through the bridge, which tier B does not use.
3. Start a capture over the USB port with `navlink capture --preset drive --port /dev/ttyUSB0`, so that it keeps running when a ros2_pc service stops.
4. Drive at 0.1 m/s in position 1 and switch the transmitter off. The robot stops and keeps balancing.
5. Drive again and stop `navbot-link-bridge.service`. The robot stops and keeps balancing. Start the service again.
6. Drive again and stop `navbot-link-mux.service`. The robot stops and keeps balancing. Start the service again.

Done when a lab note in `lab/integration/` records each check with its capture, and the robot stopped in place in every failure test.

## Measure how the robot drives (P2)

You are at the robot with the transmitter on and CH5 in position 1.

1. Use `scripts/drive_steps.py` (T1). It sends a fixed list of speed and yaw-rate steps with `Link.command` at 20 Hz while it captures, so every trial is repeatable, and writes the step report.
2. Run speed steps of 0.05, 0.1, 0.2 and 0.3 m/s for 3 s each, each followed by zero.
3. Run yaw-rate steps of 0.3, 0.6 and 1.0 rad/s the same way.
4. Read the report. For each step it gives the time to 90 % of the commanded value, the steady error, the stop distance, the rollback after the stop, the peak tilt, and the smallest speed that moves the robot.
5. Measure the outline seen from above with the wheels, the height, and the mass. Record them in the [robot hardware overview](../../robot/hardware/system-overview.md). The mass also replaces a placeholder in the host simulation.
6. Choose the Nav2 limits from the results. Keep the peak pitch below the 0.06 rad tilt gate while the robot speeds up and brakes.

Done when a lab note holds the step table and the chosen limits, and the dimensions are in the hardware overview.

## Make odometry and localization hold while driving (P3)

You are at the robot with the transmitter on.

1. Push the robot 2 m straight along a tape and compare `/odom`. Correct `wheel_radius_m` if they differ.
2. Let the robot balance for 60 s and read the gyro heading drift. If it is below 1° per minute, set `heading_source:=gyro` in the boot service. AMCL corrects slower drift.
3. Check the `/odom` twist signs while you drive forward, backward and turn left.
4. Run the open LiDAR yaw offset check in the [integration roadmap](roadmap.md) and the tilt gate check in the [ros2_pc roadmap](../../ros2_pc/plans/roadmap.md). Resolve the scan-plane height in the [LiDAR hardware page](../../ros2_pc/hardware/lds08rr-lidar.md), which says it is unknown, against the measured 0.122 m in `mount.py`.
5. Run WP5 of the localization plan.
6. Switch to localization and drive the robot with `teleop_twist_keyboard` through the room for 3 minutes, at the remote's 0.3 m/s and with turns at 1.0 rad/s (D3). End on a taped mark.
7. Retune the motion gate from this drive. `motion_gate.py` drops scans above 0.3 m/s and 0.8 rad/s, values it marks as provisional, so at the remote's speeds it drops the scans that AMCL and the costmaps need. Set the limits to the highest speed and yaw rate at which the localization stays `localized`.

Done when:

- The lab note shows the localization `localized` for the whole drive, no pose jump above 0.1 m between two updates, and a final pose within 0.1 m and 5° of the mark.
- The motion gate limits are set from the drive. If AMCL loses the pose at the remote's speeds, the lab note says at which speed or yaw rate, and Nav2 uses the highest values that passed. This is the only case in which Nav2 drives slower than D3.
- The WP5 boxes of the localization plan are checked.

## Configure Nav2 and test it without moving the robot (P4)

The navigation files go next to `localization.yaml` and `localization_launch.py` in `lds08rr_lidar`, because the LiDAR service already supervises the localization stack from there. T2 writes the configuration and the launches, and T7 writes the course script.

1. Install `ros-jazzy-nav2-loopback-sim` on the ros2_pc and add it to the [ros2_pc setup](../software/ros2-pc-setup.md).
2. Start the bench hub on the ros2_pc from the task worktree with `ROS_DOMAIN_ID=84` and `--bench --port 8082`, so that nothing reaches the live bridge. In place of AMCL's real scans and the robot, it runs `bench_launch.py`. That launch runs the localization stack on the saved map, the loopback simulator, which moves `base_link` from `/cmd_vel` and publishes `/scan_slam` from the map, and a fake robot that reports a healthy robot on `/diagnostics` and `/battery_state`.
3. Run a course of five goals on the bench with `scripts/nav_course.py --hub http://127.0.0.1:8082`. Measure CPU and RAM of the bench together with the live LiDAR service.
4. Check which clock stamps `/odom` and `/scan`. The bridge maps the robot's `t_us` onto the ros2_pc clock. Nav2 needs both topics on the same clock, within its `transform_tolerance`.

| Parameter | Start value | Reason |
| --- | --- | --- |
| Controller | Regulated pure pursuit with `use_rotate_to_heading` | Few parameters and little CPU. It turns on the spot before it drives, as a two-wheel robot can. |
| `controller_frequency` | 10 Hz | Above the 4 Hz that the 250 ms `Command` timeout needs, and half the CPU of the 20 Hz default. |
| Speed | 0.3 m/s desired and maximum | The remote's top speed (D3), equal to the firmware cap `speedCapMps`. |
| Yaw rate | 1.0 rad/s maximum, also for turning on the spot | The remote's top yaw rate (D3). A 5 Hz sweep then turns 11.5°. P3 checks that AMCL keeps the pose at this rate. |
| Acceleration and braking | 0.3 m/s² and 0.5 m/s² | A steady lean of atan(a/g) gives 0.031 rad and 0.051 rad, below the 0.06 rad tilt gate. |
| Local costmap | 3 m × 3 m rolling window, 0.05 m cells | The map has 0.025 m cells. Coarser cells mean a quarter of the cells to update. |
| Obstacle source | `/scan_slam` | Floor hits are already removed there. |
| Footprint | A circle with the radius from P2, 0.10 m until then | The outline is not measured. |
| Collision monitor | A stop polygon 0.20 m in front of the footprint, and a slowdown polygon to 0.45 m at 50 % speed | At 0.3 m/s, about 0.3 s from scan to wheels is 0.09 m, plus 0.09 m to brake at 0.5 m/s². The 0.3 s is a guess. |
| Goal tolerance | 0.10 m and 10° | The simulation rolls back after a stop. |
| Recoveries | The stock behavior tree: wait, spin and back up | The stock launch sends the recoveries' velocity through the velocity smoother and the collision monitor, like the controller's, so the same limits apply. |

Done when:

- The bench reaches 5 of 5 goals and never enters a keepout zone drawn on a copy of the map.
- The lab note records CPU and RAM. At least 300 MB stays available.

## Add the safety chain (P5)

The firmware is the fast stop. It already stops the motion at once on RC loss, CH5 changes and faults. The supervisor makes Nav2 give up the goal, so the robot does not drive on by itself when the condition clears.

1. Configure the collision monitor between the velocity smoother and the bridge, as the stock launch does, with the polygons from P4. Its source is `/scan_slam` with a `source_timeout` of 0.5 s, so two missing scans stop the robot.
2. Add the keepout filter. Its mask `keepout.yaml` is a copy of the exported map image with the keepout zones painted black, in the map folder `~/.local/share/navbot/slam/`. A second `map_server` and the `costmap_filter_info_server` serve it. Without the file, the filter has no mask and blocks nothing.
3. The supervisor (T3) is a ROS-free class next to `LocalizationTracker`, fed by the LiDAR service's ROS node (T4). It reads `balance_state`, `ch5_position` and `rc_valid` from `/diagnostics`, the localization state, the scan age, the voltage from `/battery_state`, and whether the Nav2 action server answers. Its states are `not_ready`, `ready`, `driving` and `stopped`, each with a reason.
4. While driving, the supervisor cancels the goal and enters `stopped` when one of the checks in [Supervisor](#supervisor-supervisorpy) fails.
5. In `stopped`, the supervisor refuses new goals until the reason has cleared and you press **Resume**.
6. Test the supervisor on the host with literal input sequences and literal expected states.

Done when:

- The host tests pass.
- On the bench, each stop reason, faked on `/diagnostics` or by stopping a process, cancels the goal within 1.5 s. `/diagnostics` arrives at 1 Hz.
- The bench course routes around the keepout zone.

## Add the navigation mode to the web hub (P6)

1. `SlamControl` gets a third mode `navigation` (T6). It can be entered only from localization while the localization is `localized`. It starts `navigation_launch.py` as a second supervised process next to the localization launch, so that entering navigation does not restart AMCL and lose the pose. Leaving the mode stops Nav2 first. The service never starts in navigation mode.
2. In navigation mode, you press **Set goal**, then press on the map at the goal position and drag to set the heading (T5). Without **Set goal**, a drag pans the map as before.
3. **Cancel goal** ends the goal. **Stop** ends it and puts the supervisor in `stopped`. **Resume** leaves `stopped`.
4. `/map.json` gains `control.nav` with the supervisor state, its checks, the goal, the planned path cut down to at most 200 points, and the outcome.
5. The goal action client lives in the ROS node, next to the existing service calls (T4).
6. With `--bench`, the hub runs without the LiDAR and with `bench_launch.py` in place of the localization launch (T6). It refuses mapping and every map change, so that a bench can never change the saved map.

Done when on the bench a goal set in the hub moves the simulated robot there and the hub shows its path. **Cancel goal** and **Stop** end the goal. A goal outside the map or inside a keepout zone ends as `failed` with Nav2's error text.

## Drive autonomously on the robot (P7)

You are at the robot with the transmitter on and CH5 in position 1.

1. Run `scripts/nav_course.py COURSE --preflight-only` on the ros2_pc. It passes only when the hub is in navigation mode, the supervisor is `ready` (every check of [Supervisor](#supervisor-supervisorpy) passes), and at least 300 MB of RAM is available.
2. Run these drives in order. Start each one only after the previous one passed.
   1. 1 m straight ahead in an open area.
   2. 2 m with a 90° turn.
   3. The course of five goals across the room, twice.
   4. A box in the path of a goal. The robot slows down and stops, then drives around the box.
   5. A person walks across the path.
   6. A goal behind a keepout zone. The path goes around the zone.
   7. Failure tests during a goal: switch the transmitter off, move CH5 to 2, stop the Nav2 container, and stop the LiDAR service.
3. Record every drive with the topics listed in U1 and a `navlink capture`.

Done when a lab note records:

- 10 of 10 course goals reached within 0.10 m and 10°.
- No contact with an obstacle.
- The robot balancing in place after every failure test.
- CPU and RAM during the course, with at least 300 MB available and no `bad_frames` or `lost_frames` in the bridge log.

After P7, the required features are done. The useful features follow in the order of their table. L1 starts only after you decide D1 again.

## Implement the code with parallel agents

Decisions (user, 2026-11-01):

| # | Topic | Decision |
| --- | --- | --- |
| I1 | Implementers | Claude Haiku subagents in parallel |
| I2 | Tooling | No PStack for the implementation. The agents are plain `general-purpose` agents. |
| I3 | Start | Only on the user's go. Nothing of this section runs before it. |

The lead is the main session. It does P0 and T0, starts the agents, reads each diff, merges it, runs the tests, and runs every robot phase with the user. The agents write code and tests only.

### How the agents work

The work runs under the agent workflow from P0. The lead works in the task worktree `.claude/worktrees/feat-autonomous-driving/` and keeps `work/feat-autonomous-driving/state.md` up to date.

- The lead starts each task with the Agent tool: `subagent_type: "general-purpose"`, `model: "haiku"`, `isolation: "worktree"`, `run_in_background: true`. The prompt is: "Read `docs/integration/plans/autonomous-driving.md`, sections Implement the code with parallel agents, How the agents work, Shared names and Rules for implementing agents, and task T<n>. Do only task T<n>."
- The `WorktreeCreate` hook gives each agent its own worktree `agent-<name>` on branch `worktree-<name>`, at the lead's HEAD. Parallel agents never share a checkout.
- The agent commits its task on that branch. The lead reads the diff, runs the task's tests, merges the branch into `feat/autonomous-driving`, and saves the agent's report as `work/feat-autonomous-driving/agents/T<n>.md`. The session-start sweep removes merged agent worktrees.
- A fix round is a fresh Haiku agent with the task, the lead's findings, and the branch to start from. The lead does not resume an old agent.
- ROS 2 is not installed on the desktop. Tests that need `rclpy` or `launch` skip there. The `ros2_pc` stage of `scripts/verify.py` runs them on the ros2_pc.

### Schedule

| Wave | Tasks, in parallel | Starts when |
| --- | --- | --- |
| 0 | P0, then T0 shared interfaces (lead) | the user's go, after the workflow and simulation merge |
| 1 | T1 drive steps, T2 Nav2 configuration and launches, T3 supervisor, T4 ROS node, T5 hub endpoints and page, T6 modes and service, T7 course script | T0 committed |
| 2 | Merge, `python3 scripts/verify.py`, fix rounds (lead, fresh Haiku agents) | every wave 1 task merged |
| 3 | Bench runs of P4, P5 and P6 on the ros2_pc, docs (lead) | wave 2 `done` |

The robot phases run beside the waves with the user. P1 and P3 can start after P0. P2 needs T1. P7 needs wave 3.

### Shared names

`PKG` is `src/ros2_pc/ros2/src/lds08rr_lidar/lds08rr_lidar/`, and `TEST` is `src/ros2_pc/ros2/src/lds08rr_lidar/test/`. T0 commits every name below as a stub with its signature and docstring. The tasks fill in the bodies. Only the lead changes a shared name.

#### Modes (`slam.py`)

- `NAVIGATION = "navigation"` and `MODES = (MAPPING, LOCALIZATION, NAVIGATION)`.
- `set_mode(NAVIGATION)` from `LOCALIZATION` starts the Nav2 process and keeps the localization process running. From `NAVIGATION`, `set_mode(LOCALIZATION)` cancels the goal and stops the Nav2 process only. `set_mode(MAPPING)` from `NAVIGATION` does that first, then switches as today.
- `initial_mode()` never returns `NAVIGATION`.
- `SlamControl.__init__` gains the keyword arguments `supervisor=None`, `nav=None` (the ROS node), `nav_process: SlamProcess | None = None`, `localization_command: Callable[[Path], list[str]] = localization_command`, and `map_changes: bool = True`.
- `MapStore.keepout_yaml() -> Path | None` returns `directory / "keepout.yaml"` when the file exists.

Refusals, exact texts:

| Call | Refused when | Text |
| --- | --- | --- |
| `set_mode(mode)` | unknown mode | `unknown mode {mode!r}; use mapping, localization or navigation` |
| `set_mode(NAVIGATION)` | no supervisor, node or Nav2 process | `Navigation is not available on this service` |
| `set_mode(NAVIGATION)` | mode is `MAPPING` | `Switch to localization first, then to navigation` |
| `set_mode(NAVIGATION)` | localization state is not `localized` | `Navigation needs a localized pose; wait or press Locate` |
| `set_mode(MAPPING)`, `save`, `delete`, `reset` | `map_changes` is false | `Map changes are not available on the bench` |
| `nav_goal` | mode is not `NAVIGATION` | `Goals need navigation mode` |
| `nav_goal` | x, y or yaw is not a finite number | `x, y and yaw must be finite numbers` |
| `nav_cancel` | no goal runs | `No goal is running` |
| `nav_resume` | mode is not `NAVIGATION` | `Resume needs navigation mode` |

The methods are `nav_goal(x: float, y: float, yaw: float) -> str | None`, `nav_cancel() -> str | None`, `nav_stop() -> None` and `nav_resume() -> str | None`. `nav_goal` passes on the refusal of `supervisor.request_goal()` and of `nav.send_nav_goal()` unchanged.

#### Navigation status in `/map.json`

`SlamControl.status()["nav"]` is `None` outside navigation mode. In navigation mode:

```json
{
	"supervisor": {"state": "driving", "reason": null, "checks": [{"name": "balance", "ok": true, "detail": "Run"}]},
	"goal_id": 3,
	"goal": {"x": 1.2, "y": -0.4, "yaw": 1.571},
	"distance_remaining_m": 0.8,
	"outcome": null,
	"error": "",
	"plan": [[0.1, 0.0], [0.4, -0.1]],
	"process": {"state": "running", "pid": 4242, "exit_code": null, "restarts": 0, "error": null}
}
```

- `goal_id` counts accepted goals from 1 and is 0 before the first.
- `outcome` belongs to `goal_id`. It is `null` while the goal runs, then `reached`, `failed` or `canceled`. `error` holds Nav2's error text for `failed` and is empty otherwise.
- `plan` has at most 200 points in the `map` frame.
- `process` is the Nav2 process's `SlamProcess.status()`.

#### Supervisor (`supervisor.py`)

```python
VOLTAGE_MIN_V = 6.8        # placeholder until U6 measures the discharge curve
HEALTH_MAX_AGE_SEC = 3.0
SCAN_MAX_AGE_SEC = 0.5
CHECKS = ("balance", "ch5", "remote", "health", "localization", "scan", "battery", "nav2")

class NavSupervisor:
	def __init__(self, clock: Callable[[], float] = time.monotonic) -> None: ...
	def set_health(self, values: Mapping[str, str]) -> None: ...   # KeyValues of the navbot_firmware status
	def set_voltage(self, volts: float) -> None: ...
	def set_localization(self, state: str) -> None: ...             # LocalizationTracker state
	def scan_seen(self) -> None: ...
	def set_nav_ready(self, ready: bool) -> None: ...
	def request_goal(self) -> str | None: ...
	def goal_ended(self) -> None: ...
	def stop(self) -> None: ...
	def resume(self) -> str | None: ...
	def check(self) -> bool: ...                                     # True: cancel the running goal now
	def snapshot(self) -> dict: ...
```

| Check | Passes when | `detail` and reason when it fails |
| --- | --- | --- |
| `balance` | `balance_state` is `Run` | `balance is <state>` |
| `ch5` | `ch5_position` is `1` | `CH5 is in position <n>, not 1` |
| `remote` | `rc_valid` is `1` or `True` | `no remote signal` |
| `health` | `set_health` came at most 3.0 s ago | `no robot health yet` or `no robot health for <s> s` (one decimal) |
| `localization` | the state is `localized` | `localization is <state>` |
| `scan` | `scan_seen` came at most 0.5 s ago | `no LiDAR scan for 0.5 s` |
| `battery` | the voltage is at least 6.8 V | `no battery voltage yet` or `battery at <v> V, below 6.8 V` (two decimals) |
| `nav2` | `set_nav_ready(True)` was the last call | `Nav2 is not ready` |

A passing check has the shown value as its `detail`, for example `Run`, `1`, `localized` or `7.62 V`. Before its first input, a check fails.

States and transitions:

| State | Meaning | Leaves by |
| --- | --- | --- |
| `not_ready` | No goal runs and a check fails. `reason` is the first failing check's detail. | All checks pass: `ready`. `stop()`: `stopped`. |
| `ready` | No goal runs and every check passes. | `request_goal()`: `driving`. A check fails: `not_ready`. `stop()`: `stopped`. |
| `driving` | A goal runs. | `goal_ended()`: `ready` or `not_ready`. `check()` finds a failing check: `stopped`, and it returns True once. `stop()`: `stopped`. |
| `stopped` | A goal was ended by a failing check or by `stop()`. `reason` is that check's detail or `stopped by the user`. | Only `resume()`: `ready` or `not_ready`. |

- `request_goal()` returns None in `ready` and in `driving`, where Nav2 replaces the running goal. In `not_ready` it returns the reason. In `stopped` it returns `stopped: <reason>; press Resume`.
- `resume()` in `stopped` returns `cannot resume: <first failing detail>` and stays while a check fails. In every other state it returns None and changes nothing.
- `goal_ended()` in `stopped` changes nothing.
- `snapshot()` returns `{"state": ..., "reason": ... or None, "checks": [...]}` with the checks in `CHECKS` order.

#### ROS node (`ros_node.py`)

- `LidarNode.__init__(self, config, localization, supervisor: NavSupervisor | None = None, publish_scans: bool = True)`. With `publish_scans` false, the node publishes neither `/scan` nor `/scan_slam`.
- With a supervisor, the node subscribes to `/diagnostics` (the status named `navbot_firmware`), `/battery_state`, `/scan_slam` (best effort) and `plan` (`nav_msgs/msg/Path`). It feeds the supervisor and keeps the newest plan.
- A 5 Hz timer calls `supervisor.set_localization(<tracker state>)` and `supervisor.set_nav_ready(self.nav_server_ready())`, then cancels the goal when `supervisor.check()` returns True.
- `nav_server_ready() -> bool`.
- `send_nav_goal(x, y, yaw, on_feedback: Callable[[float], None], on_done: Callable[[str, str], None]) -> str | None` sends `nav2_msgs/action/NavigateToPose` to `navigate_to_pose` in the `map` frame. It returns `Nav2 is not ready` when the server does not answer. `on_feedback` gets `distance_remaining`. `on_done` gets `("reached", "")`, `("canceled", "")`, `("failed", <error_msg>)`, or `("failed", "goal rejected by Nav2")`.
- `cancel_nav_goal() -> None` cancels the current goal, if there is one.
- `nav_plan() -> list[list[float]]` returns at most 200 points of the newest plan.

#### Hub endpoints (`web.py`)

| Route | Body | Calls | Answer |
| --- | --- | --- | --- |
| `POST /nav/goal` | `{"x": 1.2, "y": -0.4, "yaw": 1.571}` | `control.nav_goal(x, y, yaw)` | `{"ok": true}`, or `{"ok": false, "error": <text>}` with 409 |
| `POST /nav/cancel` | none | `control.nav_cancel()` | as above |
| `POST /nav/stop` | none | `control.nav_stop()` | `{"ok": true}` |
| `POST /nav/resume` | none | `control.nav_resume()` | as above |

Every route answers `{"ok": false, "error": "SLAM is not running on this service"}` with 409 when `control` is None, like the existing POST routes. A body that is not an object with numbers for `x`, `y` and `yaw` passes `nan` for the missing values, so `nav_goal` refuses it. The existing cross-site guard covers the new routes.

#### Launches and files

- `navigation.py` holds `NAVIGATION_LAUNCH`, `NAVIGATION_PARAMS` and `BENCH_LAUNCH` (paths next to the module), `navigation_command(keepout_yaml: Path | None) -> list[str]`, and `bench_command(map_yaml: Path) -> list[str]`.
- `navigation_command(k)` returns `["ros2", "launch", str(NAVIGATION_LAUNCH), f"params_file:={NAVIGATION_PARAMS}"]`, plus `f"keepout_mask:={k}"` when `k` is not None. `ros2 launch` rejects an empty `keepout_mask:=` (found on the ros2_pc, 2026-11-01).
- `bench_command(m)` returns `["ros2", "launch", str(BENCH_LAUNCH), f"map:={m}", f"params_file:={LOCALIZATION_PARAMS}"]`.
- `navigation_launch.py` takes the arguments `params_file`, `keepout_mask` (empty means no mask servers) and `use_sim_time` (default `false`).
- `bench_launch.py` takes `map`, `params_file`, `x`, `y` and `yaw` (the start pose, default 0).
- `bench_robot.py` is the fake robot of the bench. It is a ROS node run as `python3 -m lds08rr_lidar.bench_robot`. It publishes one `/initialpose` at the start pose once AMCL and the loopback simulator subscribe. Then it publishes, at 1 Hz until it is stopped, a `navbot_firmware` status with `balance_state` `Run`, `ch5_position` `1` and `rc_valid` `1`, and a `/battery_state` of 7.6 V.
- `package.xml` gains the exec dependencies `nav2_msgs`, `action_msgs`, `std_srvs`, `launch`, `launch_ros`, `rclcpp_components`, `nav2_bringup`, `nav2_map_server`, `nav2_loopback_sim`, `nav2_collision_monitor` and `nav2_velocity_smoother` (T0).

### Rules for implementing agents

1. You work in your own worktree, which the Agent tool created from the lead's HEAD. Commit your task there as one commit, `<area>: <what>`, when its tests pass. Do not switch branches, push, merge, rebase, flash, deploy, `ssh`, open a serial port, or start ROS nodes or services.
2. Before you start, read `AGENTS.md`, the sections named in your prompt, and the code your task changes. Change only your task's files. If you need another file or a change to a shared name, stop and report it.
3. No legacy and no compatibility code. Delete what your task replaces.
4. Match the surrounding code: the package's Python style, pytest tests with literal expected values, and fakes in the style of the existing tests (`FakeControl`, `FakeClock`, `FakeClient`). ROS imports stay in `ros_node.py`, `bench_robot.py` and the launch files. Logic stays ROS-free.
5. Tests use no network except `127.0.0.1`, no robot, no serial port and no ROS graph. A test that needs `rclpy` or `launch` starts with `pytest.importorskip`.
6. Run your task's tests and `python3 tests/layout/test_repo_layout.py`. Package tests run with `PYTHONPATH=src/ros2_pc/ros2/src/lds08rr_lidar python3 -m pytest -q <TEST file>`.
7. Your final message is the report, at most 300 words: the commit SHA, the files changed, the exact pass, fail and skip counts, and open questions.

### T0: shared interfaces (lead)

Files: `PKG/supervisor.py` (new, stubs), `PKG/navigation.py` (new, constants and stubs), `PKG/slam.py` (the constants, the new keyword arguments, and stubs of the `nav_*` methods and `keepout_yaml`), `PKG/ros_node.py` (stub signatures of the new methods), `src/ros2_pc/ros2/src/lds08rr_lidar/package.xml`.

Every stub raises `NotImplementedError`. Commit as `navigation: shared interfaces`, after the package tests still pass.

### T1: drive steps tool (P2)

Files: `scripts/drive_steps.py` (new), `scripts/navlink_presets.yaml`, `tests/scripts/test_drive_steps.py` (new), `tests/scripts/fixtures/drive-steps-*.csv` (new).

1. A new preset `steps` at 100 Hz with `ch5_position`, `balance_state`, `cmd_speed_mps`, `cmd_turn_rad_s`, `cmd_age_ms`, `motor1_velocity_f`, `motor2_velocity_f`, `roll_deg`, `gyro_z_rad_s` and `voltage_filtered_v`.
2. `drive_steps.py STEPS.yaml -o OUT.csv [--port tcp:127.0.0.1:7650] [--wheel-radius 0.01915]`. The steps file is `{"rate_hz": 20, "steps": [{"speed_mps": 0.1, "turn_rad_s": 0.0, "seconds": 3.0}]}` in YAML.
3. Before it sends anything, it reads `ch5_position` and `balance_state` through the link. Unless they are 1 and 2 (Run), it exits with 4 and `CH5 must be in position 1 with balance running`.
4. It captures the `steps` preset into `OUT.csv` the way `navlink capture` does (`src/navbot_link/capture.py`). Through a second `Link` on the same port, it sends `Link.command` at `rate_hz` for each step, and zero for 2 s at the end. The capture runs on until that is done.
5. A pure function `analyze_steps(rows, steps, wheel_radius_m) -> list[dict]` computes per step `t90_s`, `steady_error`, `peak_roll_deg`, and after a step back to zero `stop_distance_m` and `rollback_m`. It also returns the smallest step speed at which the wheels moved. Take the forward speed from the two wheel velocities with the signs that `scripts/analyze_drive_trace.py` uses (`direction()`).
6. It writes `OUT.md` with one table row per step. Exit codes are 0 done, 1 link error, 2 usage error, 3 no connection, and 4 refused.
7. Tests: `analyze_steps` against a synthetic CSV fixture with literal results, the refusal, and the command loop against `tests/navbot_link/fake_firmware.py` with `link_position` set.

Tests: `python3 -m pytest -q tests/scripts/test_drive_steps.py tests/scripts/test_navlink_presets.py`.

### T2: Nav2 configuration and launches (P4, P5)

Files: `PKG/navigation.py` (bodies), `PKG/navigation.yaml` (new), `PKG/navigation_launch.py` (new), `PKG/bench_launch.py` (new), `PKG/bench_robot.py` (new), `TEST/test_navigation.py` (new).

1. `navigation.yaml`: start from the Jazzy defaults in [nav2_params.yaml](https://raw.githubusercontent.com/ros-navigation/navigation2/jazzy/nav2_bringup/params/nav2_params.yaml). Keep only the nodes this launch starts. Set the values of the [P4 table](#configure-nav2-and-test-it-without-moving-the-robot-p4) and these:
   - Frames `map`, `odom` and `base_link`, and `robot_radius` 0.10 in both costmaps.
   - `controller_server`: `FollowPath` is `nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController` with `desired_linear_vel` 0.3, `lookahead_dist` 0.3 (0.2 to 0.6), `use_rotate_to_heading` true, `rotate_to_heading_angular_vel` 1.0, `max_angular_accel` 1.5 and `allow_reversing` false. The goal checker has `xy_goal_tolerance` 0.10 and `yaw_goal_tolerance` 0.17.
   - `planner_server`: `nav2_navfn_planner::NavfnPlanner` with `tolerance` 0.10 and `allow_unknown` false.
   - `velocity_smoother`: `OPEN_LOOP` at 20 Hz, `max_velocity` [0.3, 0.0, 1.0], `min_velocity` [-0.15, 0.0, -1.0], `max_accel` [0.3, 0.0, 1.5] and `max_decel` [-0.5, 0.0, -1.5].
   - `collision_monitor`: input `cmd_vel_smoothed`, output `cmd_vel`, and `source_timeout` 0.5. The source is the scan on `/scan_slam`. Polygon `stop_ahead` (stop) spans x 0.0 to 0.30 and y -0.12 to 0.12. Polygon `slow_ahead` (slowdown, ratio 0.5) spans x 0.0 to 0.55 and y -0.15 to 0.15.
   - The local costmap is a 3 m × 3 m rolling window with 0.05 m cells, with an obstacle layer on `/scan_slam` and an inflation layer with radius 0.25.
   - The global costmap has a static layer, an obstacle layer on `/scan_slam`, an inflation layer, and the filter `keepout_filter` (`nav2_costmap_2d::KeepoutFilter`, `filter_info_topic` `/costmap_filter_info`).
   - `behavior_server` has the plugins spin, backup and wait.
   - `bt_navigator` keeps its stock behavior tree.
2. `navigation_launch.py`: model it on the Jazzy [navigation_launch.py](https://raw.githubusercontent.com/ros-navigation/navigation2/jazzy/nav2_bringup/launch/navigation_launch.py), composed only. Start a `ComposableNodeContainer` `nav2_container` (`rclcpp_components`, `component_container_isolated`). Load `controller_server`, `smoother_server`, `planner_server`, `behavior_server`, `velocity_smoother`, `collision_monitor` and `bt_navigator` into it, with the stock plugin names and the stock `cmd_vel` to `cmd_vel_nav` remappings. Add `lifecycle_manager_navigation` with these node names. Leave out the route server, the waypoint follower and the docking server.
   - With a `keepout_mask`, also load `filter_mask_server` (`nav2_map_server::MapServer`, topic `/keepout_filter_mask`) and `costmap_filter_info_server` (type 0, mask topic `/keepout_filter_mask`), with their own `lifecycle_manager_costmap_filters`.
   - Like `localization_launch.py`, shut the whole launch down when any process exits.
3. `bench_launch.py` includes `localization_launch.py` with `map` and `params_file`. It also starts `nav2_loopback_sim`'s `loopback_simulator` with `base_frame_id` `base_link`, `scan_frame_id` `base_link`, `publish_map_odom_tf` false, `publish_clock` false, `scan_publish_dur` 0.2, `scan_range_max` 4.0, `scan_angle_increment` 0.017453 and the remapping `scan` to `/scan_slam`. Finally it starts `bench_robot.py` with the start pose.
4. `bench_robot.py` as in [Shared names](#launches-and-files).
5. Tests: the YAML values of step 1 as literals, which nodes the YAML holds (no `docking_server`, no `route_server`, no `waypoint_follower`), and `navigation_command` and `bench_command` as literal lists. With `importorskip("launch")`, also test that `generate_launch_description()` of both launches builds, and which node names it holds with and without a mask.

Tests: `PYTHONPATH=src/ros2_pc/ros2/src/lds08rr_lidar python3 -m pytest -q src/ros2_pc/ros2/src/lds08rr_lidar/test/test_navigation.py`.

### T3: supervisor (P5)

Files: `PKG/supervisor.py` (bodies), `TEST/test_supervisor.py` (new).

1. Implement [Supervisor](#supervisor-supervisorpy) exactly. It takes no lock: the callers hold the GIL and call it from the ROS thread and the HTTP thread, and each method changes its state in one step.
2. Tests with a `FakeClock` and literal sequences. Cover:
   - every check failing and passing, with its exact detail;
   - every transition of the table;
   - `check()` returning True exactly once per stop;
   - `request_goal` and `resume` refusals with their exact texts;
   - health and scan ageing out at 3.0 s and 0.5 s;
   - `rc_valid` as `1` and as `True`.

Tests: `PYTHONPATH=src/ros2_pc/ros2/src/lds08rr_lidar python3 -m pytest -q src/ros2_pc/ros2/src/lds08rr_lidar/test/test_supervisor.py`.

### T4: ROS node (P5, P6)

Files: `PKG/ros_node.py`, `TEST/test_ros_node.py`.

1. Implement [ROS node](#ros-node-ros_nodepy). Use `rclpy.action.ActionClient` from the HTTP thread the same way `call_service` uses its client, with async calls and callbacks. Never block the executor.
2. `send_nav_goal` builds the `PoseStamped` in `map` with the yaw as a quaternion about z, and maps the goal's result status to the outcomes of the shared names.
3. Keep the plan as every k-th pose, so that at most 200 points remain, including the last pose.
4. Tests in the existing `FakeNode` and `FakeClient` style, with `importorskip("rclpy")`. Cover the diagnostics filter on `navbot_firmware`, the plan reduction to 200 points with the last pose kept, the outcome mapping for each result status, the `Nav2 is not ready` refusal, and the timer cancelling once when `check()` returns True.

Tests: `PYTHONPATH=src/ros2_pc/ros2/src/lds08rr_lidar python3 -m pytest -q src/ros2_pc/ros2/src/lds08rr_lidar/test/test_ros_node.py` (skips on the desktop; the lead runs it on the ros2_pc).

### T5: hub endpoints and page (P6)

Files: `PKG/web.py`, `PKG/templates/slam.html`, `TEST/test_nav_web.py` (new).

1. The routes of [Hub endpoints](#hub-endpoints-webpy).
2. `slam.html`:
   - Replace the slide switch `#switchMode` with three buttons in `#modeButtons`: `#modeMapping`, `#modeLocalization` and `#modeNavigation`. The current mode gets `aria-pressed="true"`.
   - Review every `currentMode === 'localization'`. Navigation keeps **Save map**, **Reset map**, **Delete map** and **Locate again** disabled.
3. In navigation mode, show `#navPanel` with:
   - `#navState`: the state and the reason.
   - `#navChecks`: one row per check, with the labels Balance running, CH5 in link position, Remote signal, Robot health, Localized, LiDAR scans, Battery and Nav2 running, and with its detail.
   - The distance remaining, and the outcome with the error.
   - The buttons `#setGoal`, `#cancelGoal`, `#stopNav` (styled as danger) and `#resumeNav`.
4. **Set goal** arms the goal placement. While it is armed, the map does not pan. A pointer press sets the position and the drag sets the yaw, as `atan2` of the drag in map coordinates. The release posts `/nav/goal`. Escape or a second press on **Set goal** disarms it. Draw the goal arrow while dragging.
5. In `draw()`, draw `control.nav.plan` as a line and `control.nav.goal` as an arrow, with the existing `v.sx` and `v.sy`.
6. Tests with the Flask test client and a `FakeNavControl` in the new file:
   - every route's answers as literal JSON, and the 409 texts passed through;
   - a body without numbers, which reaches `nav_goal` as `nan`;
   - the cross-site refusal of `/nav/goal`;
   - the new element ids in `slam.html`.

Tests: `PYTHONPATH=src/ros2_pc/ros2/src/lds08rr_lidar python3 -m pytest -q src/ros2_pc/ros2/src/lds08rr_lidar/test/test_nav_web.py src/ros2_pc/ros2/src/lds08rr_lidar/test/test_slam.py`.

### T6: modes and service (P6)

Files: `PKG/slam.py`, `PKG/health.py`, `PKG/service.py`, `TEST/test_slam.py`, `TEST/test_health.py`.

1. Implement [Modes](#modes-slampy) and [Navigation status](#navigation-status-in-mapjson) in `SlamControl`.
   - Keep `command()` lock-free.
   - `nav_goal` stores the goal, increments `goal_id`, and passes callbacks to `nav.send_nav_goal`. The callbacks set `distance_remaining_m`, and on the end `outcome` and `error`, then call `supervisor.goal_ended()`.
   - `nav_stop` calls `supervisor.stop()` and `nav.cancel_nav_goal()`.
2. `health.py`: `_slam()` treats `navigation` like `localization`.
3. `service.py`:
   - With `--device`, build a `NavSupervisor` and pass it to `LidarNode`. Build the Nav2 process as `SlamProcess(lambda: navigation_command(store.keepout_yaml()), DEFAULT_SLAM_DIR / "nav.log")` and pass the supervisor, the node and the process to `SlamControl`.
   - Add `--bench`, exclusive with `--demo` and `--device`. It uses the demo source for the LiDAR page only, `LidarNode(..., publish_scans=False)`, `localization_command=bench_command`, and `map_changes=False`.
4. Tests with the existing fakes plus a fake supervisor and a fake node. Cover:
   - every refusal text of [Modes](#modes-slampy);
   - localization to navigation without a restart of the localization process;
   - navigation to localization stopping only the Nav2 process;
   - navigation to mapping stopping both;
   - `goal_id` and `outcome` across two goals;
   - the bench refusing map changes;
   - `initial_mode()` never returning navigation.

Tests: `PYTHONPATH=src/ros2_pc/ros2/src/lds08rr_lidar python3 -m pytest -q src/ros2_pc/ros2/src/lds08rr_lidar/test/test_slam.py src/ros2_pc/ros2/src/lds08rr_lidar/test/test_health.py`.

### T7: course script (P4, P7)

Files: `scripts/nav_course.py` (new), `tests/scripts/test_nav_course.py` (new).

1. `nav_course.py COURSE.yaml [--hub http://127.0.0.1:8081] [--preflight-only] [--min-free-mb 300] [--max-error-m 0.10] [--max-error-deg 10] [--timeout 120] [--json]`. It uses only the hub's HTTP API, with no ROS.
2. The course file is `{"repeat": 1, "goals": [{"name": "desk", "x": 1.2, "y": -0.4, "yaw_deg": 90}]}` in YAML. The script keeps courses outside the repository.
3. The preflight reads `/map.json`. It fails with `hub is in <mode> mode, not navigation` or `supervisor is <state>: <reason>`. When the hub is on `127.0.0.1` or `localhost`, it also reads `MemAvailable` from `/proc/meminfo` and fails below `--min-free-mb`. It prints each failing check.
4. For each goal, it posts `/nav/goal`, then polls `/map.json` every 0.5 s until `control.nav.goal_id` is the new one and `outcome` is set, the supervisor is `stopped`, or the timeout ends.
5. The final error is the distance from the top-level `pose` of `/map.json` to the goal, and the heading difference wrapped to ±180°. A goal passes when it is `reached` and within both limits.
6. It prints one line per goal and a summary line `passed <n>/<total>`. With `--json`, it prints one object per goal and one summary object.
7. Exit codes are 0 all passed, 1 a goal failed or the supervisor stopped, 2 usage or course file error, 3 hub unreachable, and 4 preflight failed.
8. Tests run a fake hub with `http.server` on `127.0.0.1` and scripted answers. Cover the literal output lines, every exit code, the heading wrap, and the timeout.

Tests: `python3 -m pytest -q tests/scripts/test_nav_course.py`.

### After the waves (lead)

1. Merge the wave 1 branches and run `python3 -m pytest tests -q`, the package tests, and `python3 tests/layout/test_repo_layout.py`.
2. Run `python3 scripts/verify.py` until it reports `done`. Send each failure, with its output, to a fresh Haiku fix agent.
3. Run the bench checks of P4, P5 and P6 on the ros2_pc, following [rules 2 and 3](#rules-for-every-agent).
4. Write the docs. Add a software page for the navigation mode and the bench to `docs/ros2_pc/software/`, update the [LiDAR software page](../../ros2_pc/software/lds08rr-lidar.md) and the [ros2_pc setup](../software/ros2-pc-setup.md), and add the pages to their indexes. Move what is done out of this plan.

## Rules for every agent

1. Start no motion on the robot unless the user is at the robot with the transmitter on. Unattended flashing and reboots are allowed.
2. On the ros2_pc, run every bench and every probe with `ROS_DOMAIN_ID=84`. Start it with `setsid`, record its process group, and stop only that group. Never use `pkill -f` with a ROS binary name, because the live service runs the same binaries.
3. Before you restart `navbot-lidar.service`, check `curl localhost:8081/map.json` for an unsaved map. If there is one, ask the user first.
4. Stop the robot with zero velocity or by cancelling the goal. Never send `balance on=0` to stop it.
5. Use the stock Nav2 nodes and plugins. Write new code only for the supervisor, the hub, the course script and the launches.

## Risks

- **Lean during a hard stop.** The firmware brakes at up to 1.0 m/s², a steady lean of about 0.10 rad. That is above the 0.06 rad tilt gate, so `/scan_slam` drops those scans and the collision monitor has no data for that time. The number is derived from atan(a/g). P2 measures the real lean.
- **Rollback at the goal.** The simulation rolls back at up to 0.076 m/s after a stop, and there is no robot drive trace with this controller yet. The controller may hunt around the goal. P2 measures the rollback, and the goal tolerance absorbs some of it.
- **Obstacles the LiDAR cannot see.** These are drops, everything below 12 cm, glass, mirrors and black surfaces. Keepout zones cover the known ones. L3 covers the rest.
- **RAM.** 1.29 GiB was free with no Nav2, and a Claude Code process alone took 318 MB. The navigation launch already leaves out the route server, the waypoint follower and the docking server. If P4 still misses the budget, stop agent sessions on the CM5 during drives.
- **CPU.** The bridge already takes 28 % of one core at 50 Hz. A late `/odom` shows up as TF extrapolation errors in Nav2. P4 and P7 watch for them.
- **Scan smear.** The LiDAR is not deskewed and one sweep takes 0.2 s. At the remote's 1.0 rad/s, a sweep turns 11.5°, which AMCL and the costmap see as blur. P3 measures whether AMCL keeps the pose at that rate.

## Alternatives not taken

- **Gazebo with a robot model.** It does not model the balancing, the CM5 is too small for it, and the desktop has no ROS. The loopback simulator tests Nav2, and the host simulation tests the balance path (U7).
- **A `robot_localization` filter for wheels and gyro.** The bridge already takes the heading from the gyro and the distance from the wheels. Nothing else needs fusing.
- **MPPI as the first controller.** It needs more CPU and more parameters. Regulated pure pursuit is enough at 0.3 m/s in rooms. Try MPPI only if P7 fails on path tracking, and measure its CPU first.
- **RViz on the desktop, or a new web framework.** The desktop has no ROS, and the hub already draws the map.
- **A new ROS package for navigation.** The localization configuration and launch already live in `lds08rr_lidar`, whose service supervises them. Renaming that package is a separate change.
