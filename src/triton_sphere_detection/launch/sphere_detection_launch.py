import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():

    ld = launch.LaunchDescription()

    sphere_detector = ComposableNode(
        name='sphere_detector',
        namespace='/triton',
        package='triton_sphere_detection',
        plugin='triton_sphere_detection::SphereDetector'
    )

    sphere_detector_container = ComposableNodeContainer(
        name='sphere_detector_container',
        namespace='/triton',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            sphere_detector
        ],
        output='screen'
    )

    ld.add_action(sphere_detector_container)

    return ld
