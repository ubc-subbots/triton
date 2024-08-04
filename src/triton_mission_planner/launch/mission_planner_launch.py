from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():

    ld = LaunchDescription()

    mission_planner_root = ComposableNode(
        name='mission_planner',
        namespace='/triton/mission_planner',
        package='triton_mission_planner',
        plugin='triton_mission_planner::MissionPlanner'
    )

    mission_container = ComposableNodeContainer(
        name='mission_container',
        namespace='/triton',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            mission_planner_root
        ],
        output='screen'
    )

    ld.add_action(mission_container)

    return ld
