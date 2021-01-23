import os

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    ld = LaunchDescription()

    config = os.path.join(
        get_package_share_directory('triton_state_maintainer'),
        'config',
        'offset.yaml'
    )

    component_one = Node(
        name='component_one',
        namespace='/triton/state_maintainer',
        package='triton_state_maintainer',
        executable='component_one',
        output='screen',
        parameters=[config]
    )

    ld.add_action(component_one)

    return ld 
