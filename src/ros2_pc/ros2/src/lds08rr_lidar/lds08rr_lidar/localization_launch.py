"""Stock Nav2 localization launch that ends when any of its processes exits, so the supervisor restarts it and searches again."""
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
import os


def generate_launch_description():
    stock = os.path.join(get_package_share_directory("nav2_bringup"), "launch", "localization_launch.py")
    return LaunchDescription([
        DeclareLaunchArgument("map"),
        DeclareLaunchArgument("params_file"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(stock),
            launch_arguments={
                "map": LaunchConfiguration("map"),
                "params_file": LaunchConfiguration("params_file"),
                "autostart": "true",
                "use_sim_time": "false",
                "use_composition": "False",
            }.items(),
        ),
        # Nav2's lifecycle manager shuts the other nodes down when AMCL dies but leaves the launch alive.
        RegisterEventHandler(OnProcessExit(on_exit=[EmitEvent(event=Shutdown(reason="a localization process exited"))])),
    ])
