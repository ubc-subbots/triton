import os

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    ld = LaunchDescription()

    waypoint_marker = Node(
        name='waypoint_marker',
        namespace='/triton/controls',
        package='triton_controls',
        executable='waypoint_marker',
        output='screen',
    )

    ld.add_action(waypoint_marker)

    return ld