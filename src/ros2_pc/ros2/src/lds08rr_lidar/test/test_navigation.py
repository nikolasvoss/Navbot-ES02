from pathlib import Path

import pytest
import yaml

from lds08rr_lidar import navigation
from lds08rr_lidar.navigation import NAVIGATION_PARAMS, bench_command, navigation_command

NAV_YAML = yaml.safe_load(NAVIGATION_PARAMS.read_text())
COSTMAPS = {"local_costmap", "global_costmap"}  # started inside controller_server and planner_server
NAV_NODES = {
    "controller_server", "smoother_server", "planner_server", "behavior_server",
    "velocity_smoother", "collision_monitor", "bt_navigator",
}
MASK_NODES = {"filter_mask_server", "costmap_filter_info_server"}


def params(node):
    return NAV_YAML[node]["ros__parameters"]


def test_yaml_holds_only_the_nodes_the_launch_starts():
    assert set(NAV_YAML) == NAV_NODES | COSTMAPS | MASK_NODES
    assert not {"docking_server", "route_server", "waypoint_follower"} & set(NAV_YAML)


def test_controller_follows_the_path_with_rotate_to_heading():
    follow = params("controller_server")["FollowPath"]

    assert follow["plugin"] == "nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController"
    assert params("controller_server")["controller_frequency"] == 10.0
    assert follow["desired_linear_vel"] == 0.3
    assert (follow["lookahead_dist"], follow["min_lookahead_dist"], follow["max_lookahead_dist"]) == (0.3, 0.2, 0.6)
    assert follow["use_rotate_to_heading"] is True
    assert follow["rotate_to_heading_angular_vel"] == 1.0
    assert follow["max_angular_accel"] == 1.5
    assert follow["allow_reversing"] is False


def test_goal_checker_tolerances():
    goal = params("controller_server")["general_goal_checker"]

    assert (goal["xy_goal_tolerance"], goal["yaw_goal_tolerance"]) == (0.10, 0.17)


def test_planner_uses_navfn_with_known_space_only():
    planner = params("planner_server")["GridBased"]

    assert planner["plugin"] == "nav2_navfn_planner::NavfnPlanner"
    assert planner["tolerance"] == 0.10
    assert planner["allow_unknown"] is False


def test_velocity_smoother_limits_and_mode():
    smoother = params("velocity_smoother")

    assert smoother["feedback"] == "OPEN_LOOP"
    assert smoother["smoothing_frequency"] == 20.0
    assert smoother["max_velocity"] == [0.3, 0.0, 1.0]
    assert smoother["min_velocity"] == [-0.15, 0.0, -1.0]
    assert smoother["max_accel"] == [0.3, 0.0, 1.5]
    assert smoother["max_decel"] == [-0.5, 0.0, -1.5]


def test_collision_monitor_zones_and_source():
    monitor = params("collision_monitor")

    assert (monitor["cmd_vel_in_topic"], monitor["cmd_vel_out_topic"]) == ("cmd_vel_smoothed", "cmd_vel")
    assert monitor["source_timeout"] == 0.5
    assert monitor["polygons"] == ["stop_ahead", "slow_ahead"]
    assert monitor["observation_sources"] == ["scan"]
    assert monitor["scan"]["topic"] == "/scan_slam"
    assert monitor["stop_ahead"]["action_type"] == "stop"
    assert monitor["stop_ahead"]["points"] == "[[0.0, -0.12], [0.30, -0.12], [0.30, 0.12], [0.0, 0.12]]"
    assert monitor["slow_ahead"]["action_type"] == "limit"
    assert monitor["slow_ahead"]["slowdown_ratio"] == 0.5
    assert monitor["slow_ahead"]["points"] == "[[0.0, -0.15], [0.55, -0.15], [0.55, 0.15], [0.0, 0.15]]"


def test_local_costmap_is_a_rolling_three_metre_window_on_odom():
    local = NAV_YAML["local_costmap"]["local_costmap"]["ros__parameters"]

    assert local["global_frame"] == "odom"
    assert local["robot_base_frame"] == "base_link"
    assert (local["rolling_window"], local["width"], local["height"]) == (True, 3, 3)
    assert local["resolution"] == 0.05
    assert local["robot_radius"] == 0.10
    assert local["plugins"] == ["obstacle_layer", "inflation_layer"]
    assert local["obstacle_layer"]["scan"]["topic"] == "/scan_slam"
    assert local["inflation_layer"]["inflation_radius"] == 0.25


def test_global_costmap_on_map_with_the_keepout_filter():
    glob = NAV_YAML["global_costmap"]["global_costmap"]["ros__parameters"]

    assert glob["global_frame"] == "map"
    assert glob["robot_radius"] == 0.10
    assert glob["resolution"] == 0.05
    assert glob["plugins"] == ["static_layer", "obstacle_layer", "inflation_layer", "keepout_filter"]
    assert glob["keepout_filter"]["plugin"] == "nav2_costmap_2d::KeepoutFilter"
    assert glob["keepout_filter"]["filter_info_topic"] == "/costmap_filter_info"


def test_behavior_server_has_spin_backup_and_wait():
    behavior = params("behavior_server")

    assert behavior["behavior_plugins"] == ["spin", "backup", "wait"]
    assert behavior["global_frame"] == "odom"
    assert behavior["robot_base_frame"] == "base_link"


def test_navigation_command_with_and_without_keepout_mask(monkeypatch):
    monkeypatch.setattr(navigation, "NAVIGATION_LAUNCH", Path("/cfg/navigation_launch.py"))
    monkeypatch.setattr(navigation, "NAVIGATION_PARAMS", Path("/cfg/navigation.yaml"))

    assert navigation_command(None) == [
        "ros2", "launch", "/cfg/navigation_launch.py", "params_file:=/cfg/navigation.yaml",
    ]
    assert navigation_command(Path("/data/map/keepout.yaml")) == [
        "ros2", "launch", "/cfg/navigation_launch.py", "params_file:=/cfg/navigation.yaml",
        "keepout_mask:=/data/map/keepout.yaml",
    ]


def test_bench_command_uses_the_bench_launch_and_localization_params(monkeypatch):
    monkeypatch.setattr(navigation, "BENCH_LAUNCH", Path("/cfg/bench_launch.py"))
    monkeypatch.setattr(navigation, "LOCALIZATION_PARAMS", Path("/cfg/localization.yaml"))

    assert bench_command(Path("/data/map/map.yaml")) == [
        "ros2", "launch", "/cfg/bench_launch.py", "map:=/data/map/map.yaml", "params_file:=/cfg/localization.yaml",
    ]


def test_launch_files_sit_next_to_the_module():
    assert navigation.NAVIGATION_LAUNCH == Path(navigation.__file__).with_name("navigation_launch.py")
    assert navigation.BENCH_LAUNCH == Path(navigation.__file__).with_name("bench_launch.py")
    assert navigation.NAVIGATION_LAUNCH.is_file()
    assert navigation.BENCH_LAUNCH.is_file()


def test_navigation_launch_node_names_match_the_yaml():
    pytest.importorskip("launch")
    pytest.importorskip("launch_ros")
    from lds08rr_lidar import navigation_launch

    assert set(navigation_launch.NAV_NODES) == NAV_NODES
    assert set(navigation_launch.MASK_NODES) == MASK_NODES
    assert navigation_launch.generate_launch_description() is not None


def test_bench_launch_builds():
    pytest.importorskip("launch")
    pytest.importorskip("launch_ros")
    from lds08rr_lidar import bench_launch

    assert bench_launch.generate_launch_description() is not None
