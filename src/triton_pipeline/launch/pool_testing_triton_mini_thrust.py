from http.server import executable
from launch import LaunchDescription
from launch_ros.actions import Node, ComposableNodeContainer
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import ExecuteProcess
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory
import os

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

    imu_filter = Node(
        package='imu_filter_madgwick',
        node_executable='imu_filter_madgwick_node',
        node_name='imu_filter',
        output='screen',
        parameters=[os.path.join(
            get_package_share_directory('triton_pipeline'),
            'config',
            'imu_filter.yaml')],
        # CHANGE ME
        remappings=[
            ('/imu/data', '/triton/drivers/imu/out')
        ]
    )


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

    gate_detector = ComposableNode(
        name='detector',
        namespace='/triton/gate',
        package='triton_gate',
        plugin='triton_gate::GateDetector',
        parameters=[
            {'debug': False}
        ]
    )

    gate_container = ComposableNodeContainer(
        name='gate_container',
        namespace='/',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            gate_detector
        ],
        output='screen'
    )

    config = os.path.join(
        get_package_share_directory('triton_controls'),
        'config',
        'state_estimator_config_IMU_only.yaml'
    )

    state_estimator = Node(
        name='state_estimator',
        namespace='/triton/controls/ukf',
        package='robot_localization',
        executable='ukf_node',
        output='screen',
        parameters=[config, {'use_sim_time': False}]
    )


    imu_tf = Node(package = "tf2_ros", 
                       executable = "static_transform_publisher",
                       # arguments = "0 0 0 0 1.57079 0 imu_link base_link".split(" "))
                       arguments = "0 0 0 0 0 0 imu_link base_link".split(" "))

    predetermined_thrust = Node(
        package='triton_controls',
        executable='predetermined_thrust.py',
        name='predetermined_thrust',
        output='screen', 
    )

    ld.add_action(serial)
    ld.add_action(predetermined_thrust)
    
    # we don't really need these, but we could use them for data collection
    # uncomment to launch them too
    # ld.add_action(gate_container)
    # ld.add_action(camera1)
    # ld.add_action(camera2)
    # ld.add_action(imu)
    # ld.add_action(imu_filter)
    # ld.add_action(state_estimator)
    # ld.add_action(imu_tf)



    return ld
