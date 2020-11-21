from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():

    ld = LaunchDescription()

    component_one = ComposableNode(
        name='component_one',
        namespace='/triton/state_maintainer',
        package='triton_state_maintainer',
        plugin='triton_state_maintainer::ComponentOne'
    )

    triton_state_maintainer_container = ComposableNodeContainer(
        name='state_maintainer_container',
        namespace='/triton/state_maintainer',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            component_one
        ],
        output='screen'
    )

    ld.add_action(triton_state_maintainer_container)

    return ld 
