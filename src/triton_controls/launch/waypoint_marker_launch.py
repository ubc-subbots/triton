import os

from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    ld = LaunchDescription()

    config = os.path.join(
        get_package_share_directory('triton_controls'),
        'config',
        'waypoint_config.yaml'
    )

    waypoint_marker = ComposableNode(
        name='waypoint_marker',
        namespace='/triton/controls',
        package='triton_controls',
        plugin='triton_controls::WaypointMarker'
        parameters=[config]
    )

    waypoint_container = ComposableNodeContainer(
        name='waypoint_container',
        namespace='/triton',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            waypoint_marker
        ],
        output='screen'
    )

    ld.add_action(waypoint_container)

    return ld
