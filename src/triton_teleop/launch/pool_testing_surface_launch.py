from launch import LaunchDescription
from launch_ros.actions import Node, ComposableNodeContainer
from launch.actions import ExecuteProcess
from launch_ros.descriptions import ComposableNode

def generate_launch_description():

    ld = LaunchDescription()

    keyboard_teleop = Node(
        name='key_publisher',
        namespace='/triton',
        package='triton_teleop',
        executable='key_publisher',
        output='screen',
    )

    ld.add_action(keyboard_teleop)

    record = ExecuteProcess(
        cmd=['ros2','bag','record','/camera1/image/compressed'],
        output='screen'
    )

    ld.add_action(record)

    return ld