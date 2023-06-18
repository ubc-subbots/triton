from launch import LaunchDescription
from launch_ros.actions import Node, ComposableNodeContainer
from launch.actions import ExecuteProcess
from launch_ros.descriptions import ComposableNode

def generate_launch_description():

    ld = LaunchDescription()

    keyboard_teleop = Node(
        name='key_publisher',
        namespace='/triton',
        package='triton_teleop',
        executable='key_publisher',
        output='screen',
    )

    ld.add_action(keyboard_teleop)

    record = ExecuteProcess(
        #cmd=['ros2','bag','record','/imu/data_raw','/camera1/image/compressed','/camera2/image/compressed','-d','60'],
        # cmd=['ros2','bag','record','/imu/data_raw','/imu/mag','/imu/data',
        # '/triton/controls/ukf/odometry/filtered', 
        # '/triton/drivers/front_camera/image_raw', 
        # '/triton/drivers/imu/out', 
        # '/triton/gate/detector/debug/detection', 
        # '/triton/gate/detector/debug/segment', 
        # '/triton/gate/detector/gate_offset', 
        # '/triton/gate/detector/gate_pose', 
        # '/triton/gate/detector/gate_pose_only'],
        cmd=['ros2','bag','record', '-a'],
        output='screen'
    )

    # ld.add_action(record)

    return ld