# Nav2 simulation bench attempt

Date: 2026-11-01. Host: the ros2_pc (CM5), next to the live `navbot-lidar`, bridge and multiplexer services. Branch `feat/autonomous-driving` up to `7c4f3d7`.

Status: concluded

The user set the bench aside after these runs; it is on the [integration roadmap](../../docs/integration/plans/roadmap.md#nav2-simulation-bench).

## Setup

- The hub ran with `--bench --port 8082` on `ROS_DOMAIN_ID=84`, started with `setsid` and stopped by its own process group. The live service was not touched.
- The live map folder had no saved map. The bench used `~/map-pgm-before-edit-20261028.pgm` (485 × 683 cells, 0.025 m assumed) with a hand-written `map.yaml`, in a separate `HOME`. The start pose (0, 0, 0) was the freest cell of that map.
- Courses of five goals were picked from the map by script and kept outside the repository.

## Observations

| Run | Result | Cause |
| --- | --- | --- |
| Bench start | The launch restarted in a loop | `--bench` was not forwarded by `scripts/start_lidar.py`; `bench_robot` could not import its package; its start pose defaults were integers. Fixed in `6d99775`, `95b07dc` and `1f799d8`. |
| Localization and navigation mode | `localized`, Nav2 started, all eight supervisor checks passed, preflight ok | |
| Course 1 | 1 of 5; goal 2 stopped with `localization is lost` | Goals in open areas. With the 4 m range cap, AMCL's y std grew to 0.34 m. |
| Course 2 | Goal 1 `reached` at 0.10 m, counted as a failure | Nav2's goal tolerance equalled the course limit. Tolerance set to 0.06 m and 0.12 rad in `c02b4fd`. |
| Course 3 | 2 of 5; goal 3 `failed` with an empty error | The goal lay behind a gap that the inflated costmap closes. The empty error text was fixed in `72875ef`. |
| Restart | Localization `lost` at the start | The hub's global search reset AMCL 230 ms after the bench robot's start pose. Fixed for the bench in `7c4f3d7`, removed again with the bench. |
| Course 4 | Goal 1 stopped 0.14 m short, then `Failed to make progress` and a failed spin | The `stop_ahead` polygon (0.30 m) saw a wall corner at 0.29 m, 0.11 m. The simulator's pose and AMCL's estimate differed by about 0.25 m there. |
| Course 5 | Goal 1 `reached`, 0.06 m and 12.3° by AMCL | Against the simulator's truth the error was about 0.04 m and 2°. AMCL's yaw std was 0.21 rad at that time. |

- The Nav2 controller logged 4.8 to 6.2 Hz instead of 10 Hz while the bench and the live services ran. Load average was 4 to 9 on 4 cores, partly from parallel test builds. `loopback_simulator` alone took about 70 % of one core, and the Nav2 container 54 to 72 %.
- Available memory stayed at or above 700 MB during the courses.
- The keepout mask servers loaded. The keepout route, the stop reasons and the browser check were not run.

## Consequences

- On the robot (P7), measure the controller rate and pick goals away from walls: a goal closer than the stop polygon to a wall in the drive direction cannot be reached.
- The course limits of 0.10 m and 10° are measured on AMCL's pose. AMCL's own uncertainty can exceed them right after a stop.
