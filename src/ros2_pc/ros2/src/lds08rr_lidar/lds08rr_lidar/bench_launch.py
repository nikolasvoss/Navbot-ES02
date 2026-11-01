"""Navigation bench: localization on the saved map, the loopback simulator in place of the robot's scans, and a fake robot.

Ends the whole launch when any of the localization processes exits, like localization_launch.py.
"""
import os
import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


PACKAGE_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def generate_launch_description():
    localization = os.path.join(os.path.dirname(os.path.abspath(__file__)), "localization_launch.py")
    return LaunchDescription([
        DeclareLaunchArgument("map"),
        DeclareLaunchArgument("params_file"),
        DeclareLaunchArgument("x", default_value="0.0"),
        DeclareLaunchArgument("y", default_value="0.0"),
        DeclareLaunchArgument("yaw", default_value="0.0"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(localization),
            launch_arguments={
                "map": LaunchConfiguration("map"),
                "params_file": LaunchConfiguration("params_file"),
            }.items(),
        ),
        Node(
            package="nav2_loopback_sim",
            executable="loopback_simulator",
            name="loopback_simulator",
            output="screen",
            parameters=[{
                "base_frame_id": "base_link",
                "scan_frame_id": "base_link",
                "publish_map_odom_tf": False,
                "publish_clock": False,
                "scan_publish_dur": 0.2,
                "scan_range_max": 4.0,
                "scan_angle_increment": 0.017453,
            }],
            remappings=[("scan", "/scan_slam")],
        ),
        ExecuteProcess(
            cmd=[
                sys.executable, "-m", "lds08rr_lidar.bench_robot", "--ros-args",
                "-p", ["x:=", LaunchConfiguration("x")],
                "-p", ["y:=", LaunchConfiguration("y")],
                "-p", ["yaw:=", LaunchConfiguration("yaw")],
            ],
            additional_env={"PYTHONPATH": os.pathsep.join(filter(None, [PACKAGE_ROOT, os.environ.get("PYTHONPATH")]))},
            output="screen",
        ),
    ])
