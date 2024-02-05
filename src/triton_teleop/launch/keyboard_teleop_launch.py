from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    ld = LaunchDescription()

    # keyboard_teleop = Node(
    #     name='keyboard_teleop',
    #     namespace='/triton/teleop',
    #     package='triton_teleop',
    #     executable='keyboard_teleop',
    #     output='screen',
    # )

    # ld.add_action(keyboard_teleop)

    # joy_listener = Node (
    #     name='joy_node',
    #     package='joy',
    #     # parameters=[
    #     #     {dev : '0'},
    #     #     #{device_name : 'triton_teleop_joy'},
    #     #     {deadzone : 0.1} #, {sticky_buttons : False}
    #     # ]
    # )

    # ld.add_action(joy_listener)

    joystick_teleop = Node(
        name='joystick_teleop',
        namespace='/triton/teleop',
        package='triton_teleop',
        executable='joystick_teleop',
        output='screen',
    )

    ld.add_action(joystick_teleop)

    return ld