from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    ld = LaunchDescription()

    underwater_camera = Node(
        name='underwater_camera',
        namespace='/triton/drivers/front_camera',
        package='triton_gazebo',
        executable='underwater_camera',
        output='screen'
    )

    ld.add_action(underwater_camera)

    return ld