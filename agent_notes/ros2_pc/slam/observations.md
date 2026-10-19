# Supervised SLAM in the LiDAR service, CM5, 2026-10-19

Checks from `dev_notes/handoff-slam-ros2_pc.md` on `feat/slam` at `2aa4ee9`, run against the live `navbot-lidar.service` with the real UART0 sensor. State was read with `curl -s localhost:8081/map.json`.

## Setup

Observation: `ros-jazzy-slam-toolbox` `2.8.5-1noble.20260916.104732` is installed. The unit runs `scripts/start_lidar.py --device /dev/ttyAMA0 --ros on --host 0.0.0.0` from `/home/niko/Navbot-ES02`. The venv imports `rclpy`, `slam_toolbox.srv`, `tf2_ros` and `geometry_msgs.msg`.

Evidence: the journal after start shows `SLAM: slam_toolbox started, map dir /home/niko/.local/share/navbot/slam`.

## Checks 1 to 3: node, TF, state

Observation: `/slam_toolbox` is listed and active. `tf2_echo map lds08rr` updates about once per second. `map.json` reports `state: mapping`, a set pose, `pose_age_sec` 0.50, `control.process.state: running`, `restarts: 0`.

Evidence: `ros2 lifecycle get /slam_toolbox` returned "Node not found" while `ros2 node list` showed the node; `ros2 service call /slam_toolbox/get_state lifecycle_msgs/srv/GetState` returned `id=3, label='active'`. The lifecycle CLI result is most likely a stale ROS daemon cache.

Practical consequence: the TF stamp used for the pose age is current (handoff point for check 3 resolved). Use the `get_state` service call when the lifecycle CLI disagrees with the node list.

## Check 5: save

Observation: `POST /slam/save` returned `{"ok":true}` and rewrote `map.posegraph` (21,995,570 bytes) and `map.data` (13,766,688 bytes). slam_toolbox first writes `map.saving.posegraph`, per `slam.log`.

Evidence: `ros2 interface show slam_toolbox/srv/SerializePoseGraph` defines `uint8 result` with `RESULT_SUCCESS=0` and `RESULT_FAILED_TO_WRITE_FILE=255`, which matches the `result != 0` check in `ros_node.py`.

## Check 6: load on restart

Observation: after `systemctl restart`, `slam_params.yaml` contains `map_file_name: "/home/niko/.local/share/navbot/slam/map"` and `map_start_at_dock: true`. `slam.log` shows `Load From File /home/niko/.local/share/navbot/slam/map.posegraph`, then activation. `map.json` went `waiting` → `no_pose` (400 × 309 map) → `mapping` within about 4 s. The pose restarted near the origin (0.29, -0.18 m), as `map_start_at_dock` implies.

Practical consequence: loading through `map_file_name` in mapping mode works with this slam_toolbox version.

## Check 8: crash recovery and stale state

Observation: killing `async_slam_toolbox_node` made `ros2 launch` exit with code 0. The supervisor started a new launch (`pid` changed, `restarts: 1`, `exit_code: 0`), which loaded the saved map again and returned to `mapping`. The recovery (5 s restart delay plus about 3 s start) finished before the 10 s stale threshold, so `stale` was not shown during the kill.

Observation: freezing the node with `kill -STOP` (no exit) produced `state: mapping` up to `map_age_sec` 9.8 and `stale` from 10.8 on. After `kill -CONT` the state returned to `mapping` within 4 s.

Practical consequence: the handoff concern that `ros2 launch` outlives its node did not occur; no change to `slam_command`/`SlamProcess` is needed. A hung node is reported as stale but is not restarted.

## Check 9: stop

Observation: after `systemctl stop navbot-lidar.service`, `pgrep` found no `async_slam_tool` or `ros2 launch slam_toolbox` processes. After start, SLAM ran with a new pid and `restarts: 0`.

## Check 7: delete

Observation: the user pressed **Delete map and start over** in the browser and reported that deleting and building a new map worked.

Evidence: the journal shows `POST /slam/delete` from 192.168.2.224 at 22:43:41 with status 200. Afterwards `map.posegraph` and `map.data` were gone and `slam_params.yaml` was rewritten without `map_file_name` (738 instead of 827 bytes). `control.process.pid` changed from 27067 to 28823 with `restarts: 0`, and `saved_map.exists` was false. A new 83 × 175 map was in `mapping` state.

Practical consequence: a restart caused by delete does not count in `restarts`; that counter only tracks crashes.

## Check 4: motion

Observation: the user moved the sensor by hand and reported that the map grows and the pose arrow follows. No mirrored or rotated map was reported. This is user-reported, without recorded pose numbers.

## Tests

Observation: the requested suite gave 33 passed, 1 failed while the service was running on the default domain. `test_ros_subscriber_receives_demo_and_ctrl_c_exits_cleanly` received `scan_time` 0.20030254 instead of 0.2, which matches the live sensor's `/scan`, not the demo. With `ROS_DOMAIN_ID=83` all 34 tests passed in 15.82 s.

Practical consequence: run the suite on an isolated ROS domain, or with the service stopped.
