from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    ld = LaunchDescription()

    camera_test = Node(
        name='camera_test',
        namespace='/triton/camera_test',
        package='triton_camera_test',
        executable='camera_test',
        output='screen',
    )

    ld.add_action(camera_test)

    return ld