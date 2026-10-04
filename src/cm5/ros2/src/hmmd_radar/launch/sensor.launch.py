from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("port", default_value=""),
            DeclareLaunchArgument("baud_rate", default_value="115200"),
            Node(
                package="hmmd_radar",
                executable="hmmd_sensor",
                name="hmmd_sensor",
                output="screen",
                parameters=[
                    {
                        "port": ParameterValue(LaunchConfiguration("port"), value_type=str),
                        "baud_rate": ParameterValue(LaunchConfiguration("baud_rate"), value_type=int),
                    }
                ],
            ),
        ]
    )
