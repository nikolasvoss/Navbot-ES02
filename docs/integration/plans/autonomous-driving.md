# Plan: fully autonomous driving

Status: **proposal, not implemented.** Branch `feat/autonomous-driving`, created on 2026-10-31 from `feat/simulation` at `c07ead0`. When a phase is done, its result moves into the `hardware/` and `software/` pages and its section here is deleted. The file goes with the last phase.

The goal is that you pick a goal pose on the saved map in the web hub and the robot drives there by itself. It avoids what the LiDAR sees, and when something it can detect goes wrong, it stops in place and keeps balancing. The transmitter stays on as the emergency stop until you decide otherwise ([D1](#decide-these-first)).

This page lists the required and the useful features, then orders the required ones into phases P0 to P7. Each phase ends in a check on the host, on the Nav2 test bench, or on the robot. Before a phase starts, its task list (files, tests, shared names) is added here in the style of [link-autonomy.md](link-autonomy.md). The start values below are derived, not measured, until P2 replaces them.

## What exists today

| Piece | State | Evidence |
| --- | --- | --- |
| Link drive path | CH5 position 1 (link mode), the `balance` action, and `Command` with a 250 ms timeout are in the code on this branch and host-tested. They have never run on the robot. | `rc_mode.h`, `ControlTick.cpp:94-104`, `tests/rc_mode/`, `tests/control_tick/` |
| ROS 2 bridge | Subscribes to `/cmd_vel`. Publishes `/odom`, the TF `odom` to `base_link`, `/imu/data`, `/diagnostics` and `/battery_state`. Runs as a service on the ros2_pc behind the link multiplexer. The boot service takes the heading from the wheels. The odometry covariances are zero. | [bridge page](../software/navbot-link-bridge.md), `systemctl` on the ros2_pc, 2026-10-31 |
| Odometry | Forward sign and left/right mapping are checked. The wheel yaw reads 13.5 % high. The gyro read 1080.4° for three turns, but the bridge's gyro heading has run only on simulated samples. Wheel radius and twist signs are not checked. | [turn test](../../../lab/integration/2026-10-26-odometry-turn-test.md), [integration roadmap](roadmap.md) |
| SLAM | `slam_toolbox` maps from `/scan_slam`, which drops tilted scans and floor hits. One map is saved. Mapping with the robot's wheel odometry is not verified. | [LiDAR software](../../ros2_pc/software/lds08rr-lidar.md), [ros2_pc roadmap](../../ros2_pc/plans/roadmap.md) |
| Localization | AMCL on the saved map, a global search (**Locate**), and a switch between mapping and localization in the web hub. `/scan_slam` also drops scans above 0.8 rad/s or 0.3 m/s. This is on branch `feat/localization`, not merged here, and not verified on the robot. | `feat/localization:docs/ros2_pc/plans/localization.md`, `feat/localization:src/ros2_pc/ros2/src/lds08rr_lidar/lds08rr_lidar/motion_gate.py` |
| Nav2 | Navigation2 1.3.13 for Jazzy is installed on the ros2_pc. That includes the regulated pure pursuit, MPPI and graceful controllers, the velocity smoother, the collision monitor, the waypoint follower, docking and the simple commander. Nothing configures or starts it. `ros-jazzy-nav2-loopback-sim` is in apt but not installed. | `dpkg -l` and `apt-cache policy` on the ros2_pc, 2026-10-31 |
| ros2_pc load | CM5 with 4 cores and 1.9 GiB RAM. With the LiDAR service, SLAM, the bridge and the multiplexer running, 1.29 GiB was available. The bridge used 28 % of one core, and a Claude Code process used 318 MB. | `free -m` and `ps` on the ros2_pc, 2026-10-31 |
| LiDAR | 5 Hz, 1° bins, range capped at 4 m in SLAM and AMCL because a mirror made a phantom room. The scan plane is 0.122 m above the floor. | [LiDAR observations](../../../lab/ros2_pc/2026-10-13-lds08rr-observations.md), `mount.py` |
| Robot limits | Speed cap 0.3 m/s, acceleration 0.5 m/s², braking 1.0 m/s², yaw rate 1.0 rad/s, all uncommissioned. The yaw demand is clamped but not ramped. The wheels are 38.3 mm in diameter. The robot has no bumper, cliff or distance sensor. Footprint, height and mass are not measured. | `RobotConfig.cpp:49-54`, `BalanceController.cpp:180-186`, [robot overview](../../robot/hardware/system-overview.md) |
| Host simulation | Models pitch, axle position and yaw with the firmware's balance path. The live mode serves the link over TCP and accepts `balance` and `Command`. It has no LiDAR, no walls, and placeholder masses. | [simulation](../../robot/software/simulation.md) |
| Agent workflow | Builds, flashes and tests the firmware from the desktop. It does not deploy ros2_pc software. It is on `feat/agent-workflow`, not merged here. | `feat/agent-workflow:docs/integration/software/agent-workflow.md` |

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

## Decide these first

The plan uses the default in each row until you decide otherwise.

| # | Question | Default | Why it matters |
| --- | --- | --- | --- |
| D1 | Must the transmitter be on while the robot drives by itself? | Yes, with CH5 in position 1. CH5 to 0 is the emergency stop. Driving with the transmitter off is the later item L1. | The firmware ignores `Command` while the RC is lost. Nothing drives with the transmitter off until the firmware changes. |
| D2 | Where may it drive? | On the floor of mapped rooms, never on a table. You mark stairs and other drops as keepout zones. | The scan plane is 12 cm high and there is no cliff sensor, so the robot cannot see a drop. |
| D3 | How fast? | 0.2 m/s and 0.6 rad/s until P2 has measured the robot. | See the start values in [P4](#configure-nav2-and-test-it-without-moving-the-robot-p4). |
| D4 | Is charging in a dock part of the goal? | No, it is the later item L2. | It needs a dock and charging contacts on the robot. |

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
| U6 | A battery rule: refuse goals below a voltage, and later drive back to a home pose | The bridge publishes the voltage. The threshold needs a measured discharge curve under load. | P5 |
| U7 | The host simulation in the loop: the bridge and the Nav2 controller against `navbot_sim --live` | It tests the drive chain with the firmware's balance path before the robot does. Its numbers count only after the [simulation is validated](../../robot/plans/roadmap.md#validate-the-host-simulation). | P4 |
| U8 | Drawing keepout and speed zones in the hub | P5 uses an edited map image, which works but needs an image editor. | P6 |

### Later

These need new hardware or a new decision.

| # | Feature | What it needs |
| --- | --- | --- |
| L1 | Driving with the transmitter off (D1) | A firmware rule that makes the link the dead man in position 1, for example a heartbeat from the supervisor, and a safety review before any code. |
| L2 | Docking and charging with `opennav_docking` | A dock, charging contacts, and the charger's behaviour while the robot balances. |
| L3 | Exploration, where the robot maps an unknown room by itself | Jazzy has no apt package for it (`apt-cache search` on the ros2_pc, 2026-10-31). It needs a source package or an own frontier node. |
| L4 | Down-facing distance sensors against drops, and a bumper | The LiDAR misses drops and everything below 12 cm, such as cables and door sills. |
| L5 | People detection with the HMMD radar | The radar publishes a raw range-Doppler map only, and its wiring is not verified. |

## Put the pieces on one branch (P0)

1. Merge `feat/localization` into `feat/autonomous-driving`. Where both branches changed the bridge, the proto or the generated files, keep this branch's version, then rerun the localization tests.
2. Merge `feat/agent-workflow`. Expect conflicts in the `AGENTS.md` table, `tests/layout/test_repo_layout.py`, `OllieFOCdrive.ino`, `signals.def.h`, `tests/link/test_signals_def.cpp`, the deleted `OllieFOCdrive.h`, and `balanceRequested`, which moved into `ControlTick` on this branch.
3. Do task T8 of [link-autonomy.md](link-autonomy.md). Its status line says "not implemented", and [signal-table.md](../software/signal-table.md) and the [integration roadmap](roadmap.md) still say that the firmware ignores `Command`.
4. Build the firmware with `scripts/build_remote.sh`, flash it, then rebuild and restart the ros2_pc services.

Done when:

- Every host test and `python3 tests/layout/test_repo_layout.py` pass. The commands are in the [link-autonomy schedule](link-autonomy.md#schedule).
- The `lds08rr_lidar` and `navbot_link_bridge` tests pass on the ros2_pc.
- `navlink info` shows the build ID of the merged branch.

## Prove the link drive path on the robot (P1)

You are at the robot with the transmitter on.

1. Run robot checks 3, 4 and 7 of [link-autonomy.md](link-autonomy.md#robot-checks). They cover the CH5 positions, the `balance` action, `/cmd_vel` at 0.1 m/s from `teleop_twist_keyboard`, and the stop when the publisher stops.
2. Start a capture over the USB port with `navlink capture --preset drive --port /dev/ttyUSB0`, so that it keeps running when a ros2_pc service stops.
3. Drive at 0.1 m/s in position 1 and switch the transmitter off. The robot stops and keeps balancing.
4. Drive again and stop `navbot-link-bridge.service`. The robot stops and keeps balancing. Start the service again.
5. Drive again and stop `navbot-link-mux.service`. The robot stops and keeps balancing. Start the service again.

Done when a lab note in `lab/integration/` records each check with its capture, and the robot stopped in place in every failure test.

## Measure how the robot drives (P2)

You are at the robot with the transmitter on and CH5 in position 1.

1. Extend `scripts/run_drive_trial.py` with `--steps FILE`. It sends a fixed list of speed and yaw-rate steps with `Link.command` at 20 Hz while it captures, so every trial is repeatable.
2. Run speed steps of 0.05, 0.1 and 0.2 m/s for 3 s each, each followed by zero.
3. Run yaw-rate steps of 0.3, 0.6 and 1.0 rad/s the same way.
4. Extend `scripts/analyze_drive_trace.py` to report for each step the time to 90 % of the commanded value, the steady error, the stop distance, the rollback after the stop, the peak pitch, and the smallest speed that moves the robot.
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
6. Switch to localization and drive the robot with `teleop_twist_keyboard` at 0.2 m/s through the room for 3 minutes. End on a taped mark.

Done when:

- The lab note shows AMCL in `tracking` for the whole drive, no pose jump above 0.1 m between two updates, and a final pose within 0.1 m and 5° of the mark.
- The WP5 boxes of the localization plan are checked.

## Configure Nav2 and test it without moving the robot (P4)

The navigation files go next to `localization.yaml` and `localization_launch.py` in `lds08rr_lidar`, because the LiDAR service already supervises the localization stack from there.

1. Install `ros-jazzy-nav2-loopback-sim` on the ros2_pc and add it to the [ros2_pc setup](../software/ros2-pc-setup.md).
2. Write `navigation.yaml` and `navigation_launch.py`. The launch includes the stock `nav2_bringup` `navigation_launch.py` with `use_composition:=True` and our parameters. Use the start values below.
3. Write `bench_launch.py`. It starts the loopback simulator, a `map_server` with the saved map, and the navigation launch with the scan sources remapped to the simulator's `/scan`. Run it with `ROS_DOMAIN_ID=84`, so that nothing reaches the live bridge. The simulator publishes `/clock`, so the bench runs with `use_sim_time`.
4. Write the course script `navbot_course` as a console script of `lds08rr_lidar` with `nav2_simple_commander`. It reads a YAML list of goal poses, sends them one by one, and prints for each goal whether it was reached, the time, and the final position and heading error. With `--preflight`, it checks the conditions listed in [P7](#drive-autonomously-on-the-robot-p7) and exits with a non-zero code if one fails.
5. Run a course of five goals on the bench. Measure CPU and RAM of the bench together with the live LiDAR service.
6. Check which clock stamps `/odom` and `/scan`. The bridge maps the robot's `t_us` onto the ros2_pc clock. Nav2 needs both topics on the same clock, within its `transform_tolerance`.

| Parameter | Start value | Reason |
| --- | --- | --- |
| Controller | Regulated pure pursuit with `use_rotate_to_heading` | Few parameters and little CPU. It turns on the spot before it drives, as a two-wheel robot can. |
| `controller_frequency` | 10 Hz | Above the 4 Hz that the 250 ms `Command` timeout needs, and half the CPU of the 20 Hz default. |
| Speed | 0.15 m/s desired, 0.2 m/s maximum | Below the firmware cap of 0.3 m/s and the motion gate's 0.3 m/s. |
| Yaw rate | 0.6 rad/s maximum | Below the motion gate's 0.8 rad/s. A 5 Hz sweep then turns about 7°. |
| Acceleration and braking | 0.3 m/s² and 0.5 m/s² | A steady lean of atan(a/g) gives 0.031 rad and 0.051 rad, below the 0.06 rad tilt gate. |
| Local costmap | 3 m × 3 m rolling window, 0.05 m cells | The map has 0.025 m cells. Coarser cells mean a quarter of the cells to update. |
| Obstacle source | `/scan_slam` | Floor hits are already removed there. |
| Footprint | A circle with the radius from P2, 0.10 m until then | The outline is not measured. |
| Collision monitor | A stop polygon 0.10 m in front of the footprint, and a slowdown polygon to 0.30 m at 50 % speed | At 0.2 m/s, about 0.3 s from scan to wheels is 0.06 m, plus 0.04 m to brake at 0.5 m/s². The 0.3 s is a guess. |
| Goal tolerance | 0.10 m and 10° | The simulation rolls back after a stop. |
| Recoveries | Wait, spin, and back up by at most 0.10 m at 0.05 m/s | Slow and short recoveries keep the lean small. |

Done when:

- The bench reaches 5 of 5 goals and never enters a keepout zone drawn on a copy of the map.
- The lab note records CPU and RAM. At least 300 MB stays available.

## Add the safety chain (P5)

The firmware is the fast stop. It already stops the motion at once on RC loss, CH5 changes and faults. The supervisor makes Nav2 give up the goal, so the robot does not drive on by itself when the condition clears.

1. Configure the collision monitor between the velocity smoother and the bridge, as the stock launch does, with the polygons from P4. Its source is `/scan_slam` with a `source_timeout` of 0.5 s, so two missing scans stop the robot.
2. Add the keepout filter. Its mask is a copy of the exported map image with the keepout zones painted black, served by a second `map_server` and the `costmap_filter_info_server`.
3. Write the supervisor as a ROS-free class next to `LocalizationTracker`, fed by the LiDAR service's ROS node. It reads `balance_state`, `balance_fault`, `ch5_position` and `rc_valid` from `/diagnostics`, the localization state, the scan age, the voltage from `/battery_state`, and the Nav2 lifecycle state. Its states are `not_ready`, `ready`, `driving` and `stopped`, each with a reason.
4. While driving, the supervisor cancels the goal and enters `stopped` when one of these holds:
   - `balance_state` is not Run.
   - `ch5_position` is not 1, or `rc_valid` is false.
   - The localization state is not `tracking`.
   - The newest scan is older than 0.5 s.
   - The voltage is below the threshold, which is a placeholder until U6 measures it.
   - A Nav2 node is not active.
5. In `stopped`, the supervisor refuses new goals until the reason has cleared and you press **Resume**.
6. Test the supervisor on the host with literal input sequences and literal expected states.

Done when:

- The host tests pass.
- On the bench, each stop reason, faked on `/diagnostics` or by stopping a process, cancels the goal within 1.5 s. `/diagnostics` arrives at 1 Hz.
- The bench course routes around the keepout zone.

## Add the navigation mode to the web hub (P6)

1. Add a third mode `navigation` to `SlamControl`. It can be entered only from localization while the tracker is in `tracking`. It starts `navigation_launch.py` as a second supervised process next to the localization launch, so that entering navigation does not restart AMCL and lose the pose. Leaving the mode stops Nav2 first.
2. In navigation mode, a press on the map sets the goal position and a drag sets the heading. Add `POST /nav/goal` with `{"x": ..., "y": ..., "yaw": ...}`, `POST /nav/cancel`, and `POST /nav/resume`, behind the existing cross-site guard.
3. Add a **Stop** button that cancels the goal and puts the supervisor in `stopped`.
4. `/map.json` gains `nav` with the supervisor state and reason, the goal, and the planned path, cut down to at most 200 points.
5. Put the goal action client in the ROS node, next to the existing service calls.
6. Add a `--bench` option to the hub. With it, the hub runs on `ROS_DOMAIN_ID=84` on port 8082 without the LiDAR, so that the bench can be driven from the browser.
7. Test the endpoints with a fake action client and literal JSON, and check the page with the existing Playwright test.

Done when on the bench a goal set in the hub moves the simulated robot there and the hub shows its path. **Cancel** and **Stop** end the goal. A goal outside the map or inside a keepout zone is refused with a reason.

## Drive autonomously on the robot (P7)

You are at the robot with the transmitter on and CH5 in position 1.

1. Run `navbot_course --preflight`. It passes only when `ch5_position` is 1, `rc_valid` is true, `balance_state` is Run, the localization is `tracking`, scans arrive at more than 4 Hz, the voltage is above the threshold, and at least 300 MB of RAM is available.
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

## Rules for every agent

1. Start no motion on the robot unless the user is at the robot with the transmitter on. Unattended flashing and reboots are allowed.
2. On the ros2_pc, run every bench and every probe with `ROS_DOMAIN_ID=84`. Start it with `setsid`, record its process group, and stop only that group. Never use `pkill -f` with a ROS binary name, because the live service runs the same binaries.
3. Before you restart `navbot-lidar.service`, check `curl localhost:8081/map.json` for an unsaved map. If there is one, ask the user first.
4. Stop the robot with zero velocity or by cancelling the goal. Never send `balance on=0` to stop it.
5. Use the stock Nav2 nodes and plugins. Write new code only for the supervisor, the hub, the course script and the launches.

## Risks

- **Lean during a hard stop.** The firmware brakes at up to 1.0 m/s², a steady lean of about 0.10 rad. That is above the 0.06 rad tilt gate, so `/scan_slam` drops those scans and the collision monitor has no data for that time. The number is derived from atan(a/g). P2 measures the real lean.
- **Rollback at the goal.** The simulation rolls back at up to 0.076 m/s after a stop, and there is no robot drive trace with this controller yet. The controller may hunt around the goal. P2 measures the rollback, and the goal tolerance absorbs some of it.
- **Obstacles the LiDAR cannot see.** These are drops, everything below 12 cm, glass, mirrors and black surfaces. Keepout zones cover the known ones. L4 covers the rest.
- **RAM.** 1.29 GiB was free with no Nav2, and a Claude Code process alone took 318 MB. If P4 misses the budget, stop agent sessions on the CM5 during drives, or drop the waypoint follower, the route server and the docking server from the launch.
- **CPU.** The bridge already takes 28 % of one core at 50 Hz. A late `/odom` shows up as TF extrapolation errors in Nav2. P4 and P7 watch for them.
- **Scan smear.** The LiDAR is not deskewed and one sweep takes 0.2 s. At 0.6 rad/s, a sweep turns 7°, which AMCL and the costmap see as blur. The yaw-rate limit keeps it small.

## Alternatives not taken

- **Gazebo with a robot model.** It does not model the balancing, the CM5 is too small for it, and the desktop has no ROS. The loopback simulator tests Nav2, and the host simulation tests the balance path (U7).
- **A `robot_localization` filter for wheels and gyro.** The bridge already takes the heading from the gyro and the distance from the wheels. Nothing else needs fusing.
- **MPPI as the first controller.** It needs more CPU and more parameters. Regulated pure pursuit is enough at 0.2 m/s in rooms. Try MPPI only if P7 fails on path tracking, and measure its CPU first.
- **RViz on the desktop, or a new web framework.** The desktop has no ROS, and the hub already draws the map.
- **A new ROS package for navigation.** The localization configuration and launch already live in `lds08rr_lidar`, whose service supervises them. Renaming that package is a separate change.
