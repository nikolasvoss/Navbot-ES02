# SLAM module scope

Original Outcome (verbatim, user, 2026-10-15): "implement the slam module as far as you can. work in a separate branch. radar hardware is currently not installed. as with the other sensors, i want to have a webpage, where data and relevant information is shown. for now it is my only point of contact. check if combining the existing radar and lidar websites would be a good idea. that would make things easier for me. just use different tabs or subpages for each module. do not reinvent the wheel. well tested software is preferred over writing everything yourself. new library downloads allowed. reduce unnecessary bloat and keep it simple. ask question when unsure."

Amendment (user, 2026-10-15): hub = lidar Flask app with tabs; engine = slam_toolbox; when finished, give a prompt for Claude running on the ros2_pc.

Acceptance Criteria:
- Branch feat/slam, separate worktree .worktrees/slam.
- One web hub with Lidar, SLAM, Radar tabs; radar tab works without radar hardware (shows disconnected state).
- SLAM tab shows map, pose, and slam status from slam_toolbox via ROS `/map` and TF.
- slam_toolbox launch + config for lidar-only operation.
- Glue code tested without ROS; ROS-dependent parts documented as unverified here.
- Handoff prompt for ros2_pc Claude.

Approved Approach: extend lds08rr_lidar Flask app; slam_toolbox (apt) with static odom frame; no new web framework.
Explicit Non-Goals: wheel odometry, navigation, changes to the radar page code or launcher, systemd unit for SLAM.
Compatibility / Migration Requirements: NONE
Baseline: origin/main 5ff3625
