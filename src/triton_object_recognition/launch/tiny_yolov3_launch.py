import os
from ament_index_python.packages import get_package_share_directory
import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():

    ld = launch.LaunchDescription()

    config = os.path.join(
        get_package_share_directory('triton_object_recognition'),
        'config',
        'tiny_yolov3.yaml'
    )

    object_recognizer = ComposableNode(
        name='object_recognizer',
        namespace='/triton/object_recognition',
        package='triton_object_recognition',
        parameters=[config], 
        plugin='triton_object_recognition::ObjectRecognizer'
    )

    object_recognizer_container = ComposableNodeContainer(
        name='object_recognizer_container',
        namespace='/triton/object_recognition',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            object_recognizer
        ],
        output='screen'
    )

    ld.add_action(object_recognizer_container)

    return ld