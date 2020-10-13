import os
import subprocess

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    ld = LaunchDescription()

    world_arg = DeclareLaunchArgument(
            'world',
            default_value='empty.world',
            description='gazebo world file')

    ld.add_action(world_arg)

    # We need to add the models and worlds directories to env so gazebo can find them
    triton_gazebo_dir = get_package_share_directory('triton_gazebo')
    os.environ['GAZEBO_MODEL_PATH'] += os.path.join(triton_gazebo_dir, 'gazebo', 'models')
    os.environ['GAZEBO_RESOURCE_PATH'] += os.path.join(triton_gazebo_dir, 'gazebo')

    gazebo_command = ExecuteProcess(
        cmd=['gazebo', '--verbose', ['worlds/', LaunchConfiguration('world')]]
    )

    ld.add_action(gazebo_command)
    return ld