import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    params = os.path.join(get_package_share_directory("lds08rr_lidar"), "config", "slam_toolbox.yaml")
    slam_launch = os.path.join(get_package_share_directory("slam_toolbox"), "launch", "online_async_launch.py")
    return LaunchDescription([
        # No wheel odometry yet: odom is fixed to the sensor frame and scan matching does the tracking.
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=["--frame-id", "odom", "--child-frame-id", "lds08rr"],
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(slam_launch),
            launch_arguments={"slam_params_file": params, "use_sim_time": "false"}.items(),
        ),
    ])
