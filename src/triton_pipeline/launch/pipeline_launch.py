import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    ld = LaunchDescription()

    config_arg = DeclareLaunchArgument(
            'sequence',
            default_value='example_sequence.yaml',
            description='Sequence config file'
    )

    config = [get_package_share_directory('triton_pipeline'),'/config/', LaunchConfiguration('sequence')]


    pipeline_manager = Node(
        name='pipeline_manager',
        namespace='/triton',
        package='triton_pipeline',
        executable='pipeline_manager',
        output='screen'
    )

    pipeline_sequence_manager = Node(
        name='pipeline_sequence_manager',
        namespace='/triton',
        package='triton_pipeline',
        executable='pipeline_sequence_manager',
        output='screen',
        parameters=[config]
    )

    pipeline_container = ComposableNodeContainer(
        name='pipeline',
        namespace='/triton',
        package='rclcpp_components',
        executable='component_container'
    )

    ld.add_action(pipeline_manager)
    ld.add_action(pipeline_sequence_manager)
    ld.add_action(pipeline_container)

    return ld
