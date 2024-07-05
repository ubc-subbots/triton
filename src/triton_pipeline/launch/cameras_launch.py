from launch import LaunchDescription
from launch_ros.actions import Node, ComposableNodeContainer
from launch.actions import ExecuteProcess
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    ld = LaunchDescription()

    camera1 = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        namespace='/camera1',
        parameters=[
            os.path.join(
                get_package_share_directory('triton_pipeline'),
                'config',
                'camera1.yaml')
        ],
        remappings=[
            ('/camera1/image_raw', '/triton/drivers/front_camera/image_raw')
        ]
    )

    ld.add_action(camera1)

    camera2 = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        namespace='/camera2',
        parameters=[
            os.path.join(
                get_package_share_directory('triton_pipeline'),
                'config',
                'camera2.yaml')
        ],
        remappings=[
            ('/camera2/image_raw', '/triton/drivers/bottom_camera/image_raw')
        ]
    )

    ld.add_action(camera2)

    return ld