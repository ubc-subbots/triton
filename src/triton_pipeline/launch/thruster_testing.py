from launch import LaunchDescription
from launch_ros.actions import Node, ComposableNodeContainer
from launch.actions import ExecuteProcess
from launch_ros.descriptions import ComposableNode

def generate_launch_description():

    ld = LaunchDescription()

    serial = ComposableNodeContainer(
            name='serial_subscriber_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='triton_controls',
                    plugin='triton_controls::SerialSubscriber',
                    name='serial_subscriber'),
            ],
            output='both',
    ) 

    keyboard_teleop = Node(
        name='key_publisher',
        namespace='/triton',
        package='triton_teleop',
        executable='key_publisher',
        output='screen',
    )

    ld.add_action(keyboard_teleop)
    ld.add_action(serial)

    record = ExecuteProcess(
        cmd=['ros2','bag','record','/camera1/image_raw/compressed'],
        output='screen'
    )

    #ld.add_action(record)

    return ld