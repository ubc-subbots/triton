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

    record = ExecuteProcess(
        cmd=['ros2','bag','record','/imu/data_raw','-d','60'],
        output='screen'
    )

    ld.add_action(record)

   

    return ld