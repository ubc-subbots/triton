import launch

from launch_ros.actions import ComposableNodeContainer
from launch_ros.actions import Node


def generate_launch_description():

    ld = launch.LaunchDescription()

    pipeline_manager = Node(
        name='pipeline_manager',
        namespace='/triton',
        package='triton_pipeline',
        executable='pipeline_manager',
        output='screen'
    )

    pipeline_container = ComposableNodeContainer(
        name='pipeline',
        namespace='/triton',
        package='rclcpp_components',
        executable='component_container'
    )

    ld.add_action(pipeline_manager)
    ld.add_action(pipeline_container)

    return ld