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

    # micro_ros_agent = ExecuteProcess(
    #     cmd=['docker','run','--net=host','-v','/dev:/dev','--privileged','microros/micro-ros-agent:foxy','serial','--dev','/dev/ttyACM0','-v6','--env','ROS_DOMAIN_ID=42'],
    #     output='screen'
    # )


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


    record = ExecuteProcess(
        #cmd=['ros2','bag','record','/imu/data_raw','/camera1/image/compressed','/camera2/image/compressed','-d','60'],
        cmd=['ros2','bag','record','/imu/data_raw','/imu/mag','/imu/data','/camera1/image_raw/compressed','d','60'],
        output='screen'
    )


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

    pid_controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('triton_pid_controller'), 'launch', 'triton_pid_controller_launch.py')
        )
    )

    waypoint_marker = Node(
        package='triton_controls', 
        executable='waypoint_marker',
        output='screen', 
        # parameters=[{'use_sim_time': True}]
    )

    ta_config = os.path.join(
        get_package_share_directory('triton_controls'),
        'config',
        'thruster_config_triton_mini.yaml'
    )

    thrust_allocator = Node(
        name='thrust_allocator',
        namespace='/triton/controls',
        package='triton_controls',
        executable='thrust_allocator',
        output='screen',
        parameters=[ta_config]
    )

    gate_detector = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('triton_gate'), 'launch', 'gate_detector_launch.py')
        )
    )

    trajectory_generator = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            get_package_share_directory('triton_controls') + '/launch/trajectory_generator_launch.py'
        )
    )

    keyboard_teleop = Node(
        name='key_publisher',
        namespace='/triton',
        package='triton_teleop',
        executable='key_publisher',
        output='screen',
    )

    # state and transform publisher?

    #ld.add_action(pipeline_manager)
    #ld.add_action(pipeline_sequence_manager)
    #ld.add_action(pipeline_container)
    ld.add_action(serial)
    # ld.add_action(micro_ros_agent)
    ld.add_action(imu)
    ld.add_action(imu_filter)
    ld.add_action(camera1)
    ld.add_action(camera2)
    ld.add_action(pid_controller)
    ld.add_action(waypoint_marker)
    ld.add_action(thrust_allocator)
    ld.add_action(gate_detector)
    # ld.add_action(trajectory_generator) # we use key publisher instead
    #ld.add_action(record)
    ld.add_action(keyboard_teleop)


    return ld
