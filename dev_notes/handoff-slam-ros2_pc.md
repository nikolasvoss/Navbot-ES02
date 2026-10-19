# Handoff: activate and verify SLAM on the ros2_pc

For the agent working on the ros2_pc after pulling `feat/slam`. The SLAM changes were tested on the dev PC without ROS only. Read `AGENTS.md`, the "Web hub and SLAM" section in `docs/ros2_pc/software/README.md`, and `dev_notes/scope-slam-map-autostart.md` before you change anything. Keep the scope: no odometry, navigation, multiple maps, extra systemd units, or radar/HMMD launcher changes.

## What changed

- `navbot-lidar.service` now starts `ros2 launch slam_toolbox online_async_launch.py` as a supervised child process when ROS is on and `slam_toolbox` is installed. Code: `src/ros2_pc/ros2/src/lds08rr_lidar/lds08rr_lidar/slam.py` (`SlamProcess`, `SlamControl`, `MapStore`) and `service.py` (`build_slam_control`).
- The LiDAR node publishes the static TF `odom` → `lds08rr` and calls `/slam_toolbox/serialize_map`.
- One map lives in `~/.local/share/navbot/slam/` (`map.posegraph`, `map.data`). The generated `slam_params.yaml` there adds `map_file_name` and `map_start_at_dock: true` when the map exists. The log is `slam.log` in the same directory.
- `launch/slam.launch.py` is gone. Do not use it.

## 1. Setup

1. Find out which checkout the service runs from: `systemctl cat navbot-lidar.service | grep ExecStart`. Pull `feat/slam` in that checkout, or rerun `python3 scripts/install_lidar_service.py` from the checkout you pulled.
2. Install the package: `sudo apt install ros-jazzy-slam-toolbox`.
3. Confirm the venv still sees ROS: `~/.venvs/navbot-lidar/bin/python -c "import rclpy, slam_toolbox.srv, tf2_ros, geometry_msgs.msg"`. It must have been created with `--system-site-packages`.
4. Restart the service: `sudo systemctl restart navbot-lidar.service`. In `journalctl -u navbot-lidar.service -n 50`, expect `SLAM: slam_toolbox started, map dir ...`. `slam_toolbox not installed; SLAM disabled` means step 2 or 3 failed.

The service runs from the source tree through `scripts/start_lidar.py`, so no colcon rebuild is needed for it.

## 2. Verify

Run each check and note the actual output. Use `curl -s localhost:8081/map.json | python3 -m json.tool` for the state.

| # | Check | Expected |
|---|---|---|
| 1 | `ros2 node list`, `ros2 lifecycle get /slam_toolbox` (if it is a lifecycle node) | `/slam_toolbox` is present and active |
| 2 | `ros2 run tf2_ros tf2_echo map lds08rr` | the transform updates |
| 3 | `map.json` with the sensor live and still | `state` is `mapping`, `pose` is set, `pose_age_sec` < 1, `control.process.state` is `running` |
| 4 | Move the sensor slowly by hand, then watch `/slam` in the browser | the map grows and the arrow follows the motion |
| 5 | `curl -s -X POST localhost:8081/slam/save` | `{"ok": true}`, and `map.posegraph` plus `map.data` exist. If you get `serialize_map failed with result N`, compare against `ros2 interface show slam_toolbox/srv/SerializePoseGraph` |
| 6 | `sudo systemctl restart navbot-lidar.service`, then look at `slam_params.yaml`, `slam.log` and `/slam` | `map_file_name` is in the params, the log shows the pose graph loading, and the saved map shows up again soon after start |
| 7 | `curl -s -X POST localhost:8081/slam/delete` | both map files are gone, `control.process.pid` changes, and the map rebuilds from empty |
| 8 | `pkill -f async_slam_toolbox_node` | within about 10 s `state` is `stale` ("SLAM not updating"). Then check whether the supervisor restarts SLAM: `control.process.restarts` goes up. Either way, note it |
| 9 | `sudo systemctl stop navbot-lidar.service`, then `pgrep -af slam_toolbox` | no leftover processes. Start the service again afterwards |

## 3. Known unverified points

- **Check 8:** `ros2 launch` may keep running after its node dies. Then the supervisor never sees an exit, and SLAM stays dead until the next service restart. If that happens, make the smallest fix in `slam_command`/`SlamProcess`, for example running `async_slam_toolbox_node` directly when it activates by itself. Extend `test/test_slam.py` to match.
- **Check 5:** the response field `result` (0 = success) is not confirmed.
- **Check 6:** loading through `map_file_name` in mapping mode is not confirmed.
- **Check 3:** the pose age comes from the stamp of the `map` → `lds08rr` lookup. It is not confirmed that this stamp is current.
- **Check 4:** a mirrored or rotated map is expected, because the LiDAR angle direction is uncalibrated. Note it, do not fix it here.

## 4. Report back

- Record every observation with its evidence (command and output) in `agent_notes/ros2_pc/slam/observations.md`. Update the README section where reality differs.
- Run the tests: `~/.venvs/navbot-lidar/bin/python -m pytest -q src/ros2_pc/ros2/src/lds08rr_lidar/test scripts/test_start_lidar.py`.
- Commit on `feat/slam` and push. Do not merge to `main`.
- Keep balance disabled and motor outputs safe. None of this needs the robot firmware.
