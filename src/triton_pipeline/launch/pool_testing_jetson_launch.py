from launch import LaunchDescription
from launch_ros.actions import Node, ComposableNodeContainer
from launch.actions import ExecuteProcess
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    ld = LaunchDescription()

    micro_ros_agent = ExecuteProcess(
        cmd=['docker','run','--net=host','-v','/dev:/dev','--privileged','microros/micro-ros-agent:foxy','serial','--dev','/dev/ttyACM0','-v6','--env','ROS_DOMAIN_ID=42'],
        output='screen'
    )

    #ld.add_action(micro_ros_agent)

    imu = ComposableNodeContainer(
            name='phidget_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='phidgets_spatial',
                    plugin='phidgets::SpatialRosI',
                    name='phidgets_spatial'),
            ],
            output='both',
    )

    #ld.add_action(imu)

    imu_filter = Node(
        package='imu_filter_madgwick',
        node_executable='imu_filter_madgwick_node',
        node_name='imu_filter',
        output='screen',
        parameters=[os.path.join(
            get_package_share_directory('triton_pipeline'),
            'config',
            'imu_filter.yaml')],
    )

    #ld.add_action(imu_filter)

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

    record = ExecuteProcess(
        #cmd=['ros2','bag','record','/imu/data_raw','/camera1/image/compressed','/camera2/image/compressed','-d','60'],
        cmd=['ros2','bag','record','/imu/data_raw','/imu/mag','/imu/data','/camera1/image_raw/compressed','d','60'],
        output='screen'
    )

    #ld.add_action(record)

    pipeline_manager = Node(
        name='pipeline_manager',
        namespace='/triton',
        package='triton_pipeline',
        executable='pipeline_manager',
        output='screen'
    )

    pipeline_sequence_manager = Node(
        name='pipeline_sequence_manager',
        namespace='/triton',
        package='triton_pipeline',
        executable='pipeline_sequence_manager',
        output='screen',
        parameters=[[get_package_share_directory('triton_pipeline'),'/config/', 'pipeline_sequence.yaml']]
    )

    pipeline_container = ComposableNodeContainer(
        name='pipeline',
        namespace='/triton',
        package='rclcpp_components',
        executable='component_container'
    )

    ld.add_action(pipeline_manager)
    ld.add_action(pipeline_sequence_manager)
    ld.add_action(pipeline_container)

    return ld