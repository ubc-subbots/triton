import os

from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    ld = LaunchDescription()

    config = os.path.join(
        get_package_share_directory('triton_state_maintainer'),
        'config',
        'offset.yaml'
    )

    component_one = ComposableNode(
        name='component_one',
        namespace='/triton/state_maintainer',
        package='triton_state_maintainer',
        plugin='triton_state_maintainer::ComponentOne',
#        parameters=[config]
        parameters=[
        {"pose_offset.pos.x": 10.0},
        {"pose_offset.pos.y": 0.0},
        {"pose_offset.pos.z": 0.0},
        {"pose_offset.orient.w": 0.0},
        {"pose_offset.orient.x": 0.0},
        {"pose_offset.orient.y": 0.0},
        {"pose_offset.orient.z": 0.0}
        ]
    )

    triton_state_maintainer_container = ComposableNodeContainer(
        name='state_maintainer_container',
        namespace='/triton/state_maintainer',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            component_one
        ],
        output='screen'
    )

    ld.add_action(triton_state_maintainer_container)

    return ld 
