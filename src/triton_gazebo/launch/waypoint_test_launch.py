import os

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    ld = LaunchDescription()

    ukf_teleop_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('triton_gazebo'), 'launch', 'ukf_teleop_sim_launch.py')
        )
    )

    waypoint_marker = Node(
        package='triton_controls', 
        executable='waypoint_marker',
        output='screen', 
        parameters=[{'use_sim_time': True}]
    )

    waypoint_marker_tester = Node(
        package='triton_controls',
        executable='waypoint_marker_tester.py',
        name='waypoint_marker_tester',
        output='screen', 
        parameters=[{'use_sim_time': True}]
    )

    ld.add_action(ukf_teleop_sim)
    ld.add_action(waypoint_marker)
    ld.add_action(waypoint_marker_tester)

    return ld