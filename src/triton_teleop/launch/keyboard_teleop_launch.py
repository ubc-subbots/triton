from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    ld = LaunchDescription()

    keyboard_teleop = Node(
        name='keyboard_teleop',
        namespace='/triton/teleop',
        package='triton_teleop',
        executable='keyboard_teleop',
        remappings=[('/triton/teleop/force', '/triton/gazebo_drivers/force')],
        output='screen',
    )

    ld.add_action(keyboard_teleop)

    return ld