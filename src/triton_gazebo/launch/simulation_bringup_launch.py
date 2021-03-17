import os

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    ld = LaunchDescription()

    pkg_share = get_package_share_directory('triton_gazebo')
    urdf_file =  os.path.join(pkg_share, 'gazebo', 'models', 'cube_auv', 'cube_auv.urdf')
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()
    rsp_params = {'robot_description': robot_desc}

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('triton_gazebo'), 'launch', 'gazebo_launch.py')
        ),
        launch_arguments={'world': 'competition.world'}.items()
    )

    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('triton_gazebo'), 'launch', 'rviz_launch.py')
        )
    )

    rviz_timer = TimerAction(
        period=5.,
        actions=[rviz]
    )

    state_publisher = Node(
        package='robot_state_publisher', 
        executable='robot_state_publisher',
        output='screen', 
        parameters=[rsp_params]
    )

    transform_publisher = Node(
        package='triton_gazebo',
        executable='auv_transform_publisher.py',
        name='auv_transform_publisher',
        output='screen'
    )

    ld.add_action(gazebo)
    ld.add_action(rviz_timer)
    ld.add_action(state_publisher)
    ld.add_action(transform_publisher)

    return ld