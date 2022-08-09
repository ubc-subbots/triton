from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node



def generate_launch_description():

    ld = LaunchDescription()

    config = os.path.join(
        get_package_share_directory('triton_gate'),
        'config',
        'gate_detection.rviz'
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', config]
    )

    camera_test = Node(
        name='camera_test',
        namespace='/triton/camera_test',
        package='triton_camera_test',
        executable='camera_test',
        output='screen',
    )
    gate_detector = ComposableNode(
        name='detector',
        namespace='/triton/gate',
        package='triton_gate',
        plugin='triton_gate::GateDetector',
    )

    gate_container = ComposableNodeContainer(
        name='gate_container',
        namespace='/',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            gate_detector
        ],
        output='screen'
    )

    ld.add_action(gate_container)
    ld.add_action(rviz)
    ld.add_action(camera_test)

    return ld 