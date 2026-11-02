"""Nav2 navigation, composed in one container, without the route server, waypoint follower or docking server.

Ends the whole launch when any of its processes exits, like localization_launch.py.
"""
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, GroupAction, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, LoadComposableNodes, SetParameter
from launch_ros.descriptions import ComposableNode, ParameterFile

CONTAINER = "/nav2_container"
NAV_NODES = [
    "controller_server", "smoother_server", "planner_server", "behavior_server",
    "velocity_smoother", "collision_monitor", "bt_navigator",
]
MASK_NODES = ["filter_mask_server", "costmap_filter_info_server"]
KEEPOUT_PARAMS = Path(__file__).with_name("navigation_keepout.yaml")


def launch_setup(context):
    """The container, plus the keepout filter and its mask servers when keepout_mask is not empty."""
    keepout_mask = LaunchConfiguration("keepout_mask").perform(context)
    use_sim_time = LaunchConfiguration("use_sim_time")
    params = ParameterFile(LaunchConfiguration("params_file"), allow_substs=True)
    container_params = [params]
    if keepout_mask:
        container_params.append(ParameterFile(str(KEEPOUT_PARAMS)))
    cmd_vel_nav = [("cmd_vel", "cmd_vel_nav")]

    nav_nodes = [
        ComposableNode(package="nav2_controller", plugin="nav2_controller::ControllerServer",
                       name="controller_server", parameters=[params], remappings=cmd_vel_nav),
        ComposableNode(package="nav2_smoother", plugin="nav2_smoother::SmootherServer",
                       name="smoother_server", parameters=[params]),
        ComposableNode(package="nav2_planner", plugin="nav2_planner::PlannerServer",
                       name="planner_server", parameters=[params]),
        ComposableNode(package="nav2_behaviors", plugin="behavior_server::BehaviorServer",
                       name="behavior_server", parameters=[params], remappings=cmd_vel_nav),
        ComposableNode(package="nav2_velocity_smoother", plugin="nav2_velocity_smoother::VelocitySmoother",
                       name="velocity_smoother", parameters=[params], remappings=cmd_vel_nav),
        ComposableNode(package="nav2_collision_monitor", plugin="nav2_collision_monitor::CollisionMonitor",
                       name="collision_monitor", parameters=[params]),
        ComposableNode(package="nav2_bt_navigator", plugin="nav2_bt_navigator::BtNavigator",
                       name="bt_navigator", parameters=[params]),
        ComposableNode(package="nav2_lifecycle_manager", plugin="nav2_lifecycle_manager::LifecycleManager",
                       name="lifecycle_manager_navigation",
                       parameters=[{"autostart": True, "node_names": NAV_NODES}]),
    ]
    mask_nodes = [
        ComposableNode(package="nav2_map_server", plugin="nav2_map_server::MapServer",
                       name="filter_mask_server", parameters=[ParameterFile(str(KEEPOUT_PARAMS)), {"yaml_filename": keepout_mask}]),
        ComposableNode(package="nav2_map_server", plugin="nav2_map_server::CostmapFilterInfoServer",
                       name="costmap_filter_info_server", parameters=[ParameterFile(str(KEEPOUT_PARAMS))]),
        ComposableNode(package="nav2_lifecycle_manager", plugin="nav2_lifecycle_manager::LifecycleManager",
                       name="lifecycle_manager_costmap_filters",
                       parameters=[{"autostart": True, "node_names": MASK_NODES}]),
    ]
    actions = [
        SetParameter("use_sim_time", use_sim_time),
        ComposableNodeContainer(
            name="nav2_container", namespace="", package="rclcpp_components",
            executable="component_container_isolated", output="screen",
            # The params file on the container reaches the nodes that the servers create themselves, such as
            # local_costmap and global_costmap; parameters on a ComposableNode only reach that node.
            parameters=container_params,
            composable_node_descriptions=nav_nodes,
        ),
    ]
    if keepout_mask:
        actions.append(LoadComposableNodes(target_container=CONTAINER, composable_node_descriptions=mask_nodes))
    return [GroupAction(actions)]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("params_file"),
        DeclareLaunchArgument("keepout_mask", default_value=""),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        OpaqueFunction(function=launch_setup),
        RegisterEventHandler(OnProcessExit(on_exit=[EmitEvent(event=Shutdown(reason="a navigation process exited"))])),
    ])
