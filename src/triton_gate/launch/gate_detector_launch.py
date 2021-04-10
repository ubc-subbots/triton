from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():

    ld = LaunchDescription()

    gate_detector = ComposableNode(
        name='component_one',
        namespace='/triton',
        package='triton_gate',
        plugin='triton_gate::GateDetector'
    )

    gate_container = ComposableNodeContainer(
        name='gate_container',
        namespace='/triton',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            gate_detector
        ],
        output='screen'
    )

    ld.add_action(gate_container)

    return ld 