"""Nav2 navigation, composed in one container, without the route server, waypoint follower or docking server.

Ends the whole launch when any of its processes exits, like localization_launch.py.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, GroupAction, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import ComposableNodeContainer, LoadComposableNodes, SetParameter
from launch_ros.descriptions import ComposableNode, ParameterFile

CONTAINER = "/nav2_container"
NAV_NODES = [
    "controller_server", "smoother_server", "planner_server", "behavior_server",
    "velocity_smoother", "collision_monitor", "bt_navigator",
]
MASK_NODES = ["filter_mask_server", "costmap_filter_info_server"]


def generate_launch_description():
    params_file = LaunchConfiguration("params_file")
    keepout_mask = LaunchConfiguration("keepout_mask")
    use_sim_time = LaunchConfiguration("use_sim_time")
    params = ParameterFile(params_file, allow_substs=True)
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
                       name="filter_mask_server", parameters=[params, {"yaml_filename": keepout_mask}]),
        ComposableNode(package="nav2_map_server", plugin="nav2_map_server::CostmapFilterInfoServer",
                       name="costmap_filter_info_server", parameters=[params]),
        ComposableNode(package="nav2_lifecycle_manager", plugin="nav2_lifecycle_manager::LifecycleManager",
                       name="lifecycle_manager_costmap_filters",
                       parameters=[{"autostart": True, "node_names": MASK_NODES}]),
    ]
    with_mask = IfCondition(PythonExpression(["'", keepout_mask, "' != ''"]))

    return LaunchDescription([
        DeclareLaunchArgument("params_file"),
        DeclareLaunchArgument("keepout_mask", default_value=""),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        GroupAction([
            SetParameter("use_sim_time", use_sim_time),
            ComposableNodeContainer(
                name="nav2_container", namespace="", package="rclcpp_components",
                executable="component_container_isolated", output="screen",
                # The params file on the container reaches the nodes that the servers create themselves, such as
                # local_costmap and global_costmap; parameters on a ComposableNode only reach that node.
                parameters=[params],
                composable_node_descriptions=nav_nodes,
            ),
            GroupAction(condition=with_mask, actions=[
                LoadComposableNodes(target_container=CONTAINER, composable_node_descriptions=mask_nodes),
            ]),
        ]),
        RegisterEventHandler(OnProcessExit(on_exit=[EmitEvent(event=Shutdown(reason="a navigation process exited"))])),
    ])
