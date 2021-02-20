import os
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    ld = LaunchDescription()

    config = os.path.join(
        get_package_share_directory('triton_state'),
        'simple_sequence.yaml'
    )

    state_manager = Node(
        name='state_manager',
        namespace='/triton',
        package='triton_state',
        executable='state_manager',
        output='screen',
        parameters=[config]
    )

    ld.add_action(state_manager)

    return ld