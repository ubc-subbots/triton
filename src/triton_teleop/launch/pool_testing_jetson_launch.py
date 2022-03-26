from launch import LaunchDescription
from launch_ros.actions import Node, ComposableNodeContainer
from launch.actions import ExecuteProcess
from launch_ros.descriptions import ComposableNode

def generate_launch_description():

    ld = LaunchDescription()

    micro_ros_agent = ExecuteProcess(
        cmd=['sudo','docker','run','-it','--net=host','-v','/dev:/dev','--privileged','microros/micro-ros-agent:foxy','serial','--dev','/dev/ttyACM0','-v6'],
        output='screen'
    )

    ld.add_action(micro_ros_agent)

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

    ld.add_action(imu)

    camera1 = Node(
        package='usb_camera_driver',
        executable='usb_camera_driver_node',
        namespace='/camera1',
        parameters=[
            {'frame_id': 'camera1'},
            {'fps': 24.},
            {'camera_id': 0},
            {'camera_calibration_file': 'file:///opt/ros/foxy/src/ros2_usb_camera/config/camera.yaml'},
            {'image.format': 'png'},
            {'image.png_level': 3},
        ],
    )

    ld.add_action(camera1)

    camera2 = Node(
        package='usb_camera_driver',
        executable='usb_camera_driver_node',
        namespace='/camera2',
        parameters=[
            {'frame_id': 'camera2'},
            {'fps': 24.},
            {'camera_id': 1},
            {'camera_calibration_file': 'file:///opt/ros/foxy/src/ros2_usb_camera/config/camera.yaml'},
            {'image.format': 'png'},
            {'image.png_level': 3},
        ],
    )

    ld.add_action(camera2)

    record = ExecuteProcess(
        cmd=['ros2','bag','record','/imu/data_raw','/camera1/image/compressed','/camera2/image/compressed','-d','60'],
        output='screen'
    )

    ld.add_action(record)

   

    return ld