import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument, SetEnvironmentVariable, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, EnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import UnlessCondition


def generate_launch_description():

    ld = LaunchDescription()

    world_arg = DeclareLaunchArgument(
            'world',
            default_value='empty.world',
            description='Gazebo world file'
    )

    headless_arg = DeclareLaunchArgument(
            'headless',
            default_value='false',
            description="Set to 'true' to run gazebo headless"
    )

    # We need to add the models and worlds directories to env so gazebo can find them
    triton_gazebo_dir = get_package_share_directory('triton_gazebo')

    gmp = 'GAZEBO_MODEL_PATH'
    add_model_path = SetEnvironmentVariable(
        name=gmp, 
        value=[
            EnvironmentVariable(gmp), 
            os.pathsep + os.path.join(triton_gazebo_dir, 'gazebo', 'models')
        ]
    )

    grp = 'GAZEBO_RESOURCE_PATH'
    add_resource_path = SetEnvironmentVariable(
        name=grp, 
        value=[
            EnvironmentVariable(grp), 
            os.pathsep + os.path.join(triton_gazebo_dir, 'gazebo')
        ]
    )

    gazebo_server = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'), 
            'launch', 'gzserver.launch.py')
        ),
        launch_arguments={
            'world': ['worlds/', LaunchConfiguration('world')],
            'verbose': 'true'
            }.items()
    )

    gazebo_client = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'), 
            'launch', 'gzclient.launch.py')
        ),
        condition=UnlessCondition(LaunchConfiguration('headless')),
    )

    ld.add_action(world_arg)
    ld.add_action(headless_arg)
    ld.add_action(add_model_path)
    ld.add_action(add_resource_path)
    ld.add_action(gazebo_server)
    ld.add_action(gazebo_client)
    return ld