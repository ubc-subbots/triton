import os
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    ld = LaunchDescription()

    # Get the package share directory
    package_share_directory = get_package_share_directory('triton_mission_planner')
    tree_file_path = os.path.join(package_share_directory, 'config', 'tree.xml')

    mission_planner_root = ComposableNode(
        name='mission_planner',
        namespace='/triton/mission_planner',
        package='triton_mission_planner',
        plugin='triton_mission_planner::MissionPlanner',
        parameters=[{'tree_file_path': tree_file_path}]
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

