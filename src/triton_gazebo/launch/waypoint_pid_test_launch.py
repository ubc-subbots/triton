import os

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    ld = LaunchDescription()

    pid_controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('triton_pid_controller'), 'launch', 'triton_pid_controller_launch.py')
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

    pkg_share = get_package_share_directory('triton_gazebo')
    urdf_file =  os.path.join(pkg_share, 'gazebo', 'models', 'cube_auv', 'model.urdf')
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()
    rsp_params = {'robot_description': robot_desc}

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('triton_gazebo'), 'launch', 'gazebo_launch.py')
        ),
        launch_arguments={'world': 'cube.world'}.items()
    )

    rviz_config_file = os.path.join(
        pkg_share, 'config', 'rviz_ukf_teleop_sim_config.rviz')

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file]
    )

    state_estimator = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('triton_controls'), 'launch', 'state_estimator_launch.py')
        )
    )

    state_publisher = Node(
        package='robot_state_publisher', 
        executable='robot_state_publisher',
        output='screen', 
        parameters=[rsp_params, {'use_sim_time': True}]
    )

    transform_publisher = Node(
        package='triton_controls',
        executable='auv_transform_publisher.py',
        name='auv_transform_publisher',
        output='screen', 
        parameters=[rsp_params, {'use_sim_time': True}]
    )

    thrust_allocator = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('triton_controls'), 'launch', 'thrust_allocator_launch.py')
        )
    )

    gate_detector = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('triton_gate'), 'launch', 'gate_detector_launch.py')
        )
    )

    underwater_camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            get_package_share_directory('triton_gazebo') + '/launch/underwater_camera_launch.py'
        )
    )

    ld.add_action(gazebo)
    # ld.add_action(rviz)
    ld.add_action(thrust_allocator)
    ld.add_action(gate_detector)
    ld.add_action(state_publisher)
    ld.add_action(transform_publisher)
    ld.add_action(underwater_camera)
    ld.add_action(state_estimator)
    ld.add_action(waypoint_marker)
    ld.add_action(waypoint_marker_tester)
    ld.add_action(pid_controller)

    return ld