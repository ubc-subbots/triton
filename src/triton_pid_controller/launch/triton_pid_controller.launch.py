from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
#        Node(
#            package='imu_driver',
#            #namespace='',
#            executable='imu_driver',
#            name='imu_driver'
#        ),
#        Node(
#            package='motor_driver',
            #namespace='',
#            executable='motor_driver',
#            name='motor_driver'
#        ),
        Node(
            package='triton_pid_controller',
            executable='triton_pid_controller',
            name='triton_pid_controller',
            # remappings=[
            #     ('/input/pose', '/turtlesim1/turtle1/pose'),
            #     ('/output/cmd_vel', '/turtlesim2/turtle1/cmd_vel'),
            # ]
        )
    ])
