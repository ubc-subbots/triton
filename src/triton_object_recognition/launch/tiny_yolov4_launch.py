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
        'tiny_yolov4.yaml'
    )

    object_recognizer = ComposableNode(
        name='object_recognizer',
        namespace='/triton',
        package='triton_object_recognition',
        parameters=[config], 
        remappings=[('object_recognizer/out','bbox_pose/in')],
        plugin='triton_object_recognition::ObjectRecognizer'
    )

    object_recognizer_container = ComposableNodeContainer(
        name='object_recognizer_container',
        namespace='/triton',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            object_recognizer
        ],
        output='screen'
    )

    pose_estimation = ComposableNode(
        name='bbox_pose',
        namespace='/triton',
        package='triton_object_recognition',
        parameters=[{'classes': [1.,1.,1.,1.,1.],'intrinsics': [224.,168.,320.,240.],'camera_height': 480, 'camera_width': 640 }], 
        remappings=[('object_recognizer/out','bbox_pose/in')],
        plugin='triton_object_recognition::BoundingBoxPoseEstimation'
    )

    pose_estimation_container = ComposableNodeContainer(
        name='bbox_pose_container',
        namespace='/triton',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            pose_estimation
        ],
        output='screen'
    )

    ld.add_action(object_recognizer_container)
    ld.add_action(pose_estimation_container)

    return ld