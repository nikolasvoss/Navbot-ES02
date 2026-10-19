# Scope: SLAM map persistence, autostart, stale state, radar tunnel

Original Outcome (verbatim, user, 2026-10-19, chat): "Setze Punkte 1 bis 4 um. Idealerweise gibt es nur eine einzige Karte, das reicht mir glaube ich auch erstmal, die jedes Mal geladen wird und die ich aber auch einfach löschen kann, um sie neu zu erzeugen und zu speichern. Ich weiß dass das LiDAR von alleine startet? Geht das auch beim Slam so? Idealerweise möchte ich mich da nicht selbst drum kümmern, sondern es startet einfach. Wenn, wenn es geht."

Referenced points (verbatim, assistant review accepted by the user, 2026-10-19):
1. "Karte speichern fehlt. ... Ein Button „Karte speichern“ im SLAM-Tab würde die fertigen Services `/slam_toolbox/save_map` und `/slam_toolbox/serialize_map` aufrufen"
2. "SLAM lässt sich nicht über die Webseite starten. ... Start/Stop-Button ... oder ein Flag `--slam` an `start_lidar.py`."
3. "Ein abgestürztes SLAM wird nicht erkannt. ... Zustand `stale`, wenn `map_age` deutlich über `map_update_interval` (2 s) liegt, plus eine Prüfung des TF-Stempels."
4. "Der Radar-Tab funktioniert nicht von einem anderen Rechner aus. ... `start_lidar.py` leitet zusätzlich `-L 9090:127.0.0.1:9090` weiter."

Acceptance Criteria:
- Exactly one saved map (slam_toolbox serialized pose graph) in a fixed state directory; SLAM tab button saves/overwrites it.
- On every SLAM start, an existing saved map is loaded and mapping continues on it.
- SLAM tab button deletes the saved map and restarts SLAM with an empty map (with confirmation).
- SLAM starts automatically with the existing LiDAR service whenever ROS is active and slam_toolbox is installed; it is supervised and restarted if it exits; no manual `ros2 launch` needed. Without slam_toolbox the hub still works and shows SLAM as unavailable.
- `/map.json` reports `stale` when the map stops updating; the pose age comes from the TF stamp.
- `start_lidar.py --ssh` also forwards 9090 when the local port is free.
- Dev-PC tests cover the new glue without ROS; ROS-dependent parts documented as unverified.

Approved Approach: the LiDAR service spawns `ros2 launch slam_toolbox online_async_launch.py` as a supervised child process with a generated params file (base yaml + `map_file_name`/`map_start_at_dock` when a map exists). The LiDAR node publishes the static odom → lds08rr TF and calls `/slam_toolbox/serialize_map`. Remove the now redundant `slam.launch.py`.

Explicit Non-Goals: wheel/laser odometry, navigation, multiple maps or map selection, a separate systemd unit for SLAM, changes to radar page code or the HMMD launcher, rosbridge LAN exposure, PGM/YAML map export.
Compatibility / Migration Requirements: NONE

Baseline: feat/slam d8226de, clean tree. Pre-existing tracked TASK_STATE.md from an older task is replaced by this task's state.

## Plan (original)

1. `slam.py`: SlamState stale state + TF-stamp pose age; faster grid packing; MapStore (single map, params file); SlamProcess (supervised child); SlamControl (save/delete/status).
2. `ros_node.py`: static TF odom → lds08rr; `serialize_map(filename)` service client.
3. `service.py`: start SlamControl when ROS + slam_toolbox available; stop on exit.
4. `web.py` + `slam.html`: control status in `/map.json`, POST `/slam/save`, POST `/slam/delete`, buttons, stale label.
5. Move base yaml into the Python package; delete `launch/slam.launch.py`; update `setup.py`, `package.xml`.
6. `scripts/start_lidar.py`: forward 9090 when free; tests.
7. Tests for 1, 4, 6; README section update.

Expected area: `src/ros2_pc/ros2/src/lds08rr_lidar/**`, `scripts/start_lidar.py`, `scripts/test_start_lidar.py`, `docs/ros2_pc/software/README.md`, this file, TASK_STATE.md. Expected extent: roughly 350 to 500 changed lines.

## Amendments

(none yet)
