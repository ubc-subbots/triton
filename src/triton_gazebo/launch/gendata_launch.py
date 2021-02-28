import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument, SetEnvironmentVariable, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, EnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import UnlessCondition
from launch_ros.actions import Node

def generate_launch_description():

    ld = LaunchDescription()

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            get_package_share_directory('triton_gazebo') + '/launch/gazebo_launch.py'
        ),
        launch_arguments={'world': 'uc_gendata.world', 'headless': 'true'}.items()
    )

    ld.add_action(gazebo)

    underwater_camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            get_package_share_directory('triton_gazebo') + '/launch/underwater_camera_launch.py'
        )
    )

    ld.add_action(underwater_camera)

    bounding_box_image_saver = Node(
        package="triton_gazebo",
        executable='bounding_box_image_saver.py',
        name='bounding_box_image_saver'
    )

    ld.add_action(bounding_box_image_saver)

    return ld