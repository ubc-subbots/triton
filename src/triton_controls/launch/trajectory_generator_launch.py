import os

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    ld = LaunchDescription()

    trajectory_generator = Node(
        name='trajectory_generator',
        namespace='/triton/controls',
        package='triton_controls',
        executable='trajectory_generator',
        output='screen',
    )

    ld.add_action(trajectory_generator)

    return ld