import os

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    ld = LaunchDescription()

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
        package='triton_controls',
        executable='auv_transform_publisher.py',
        name='auv_transform_publisher',
        output='screen'
    )

    thrust_allocator = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('triton_controls'), 'launch', 'thrust_allocator_launch.py')
        )
    )

    keyboard_teleop = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('triton_teleop'), 'launch', 'keyboard_teleop_launch.py')
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

    yolo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            get_package_share_directory('triton_object_recognition') + '/launch/tiny_yolov4_launch.py'
        )
    )
    
    ld.add_action(gazebo)
    #ld.add_action(rviz_timer)
    ld.add_action(thrust_allocator)
    ld.add_action(keyboard_teleop)
    ld.add_action(gate_detector)
    ld.add_action(state_publisher)
    ld.add_action(transform_publisher)
    ld.add_action(underwater_camera)
    ld.add_action(yolo)

    return ld