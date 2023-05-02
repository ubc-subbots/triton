from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    ld = LaunchDescription()

    config = os.path.join(
        get_package_share_directory('triton_pid_controller'),
        'config',
        'pid_files.yaml'
    )
    
    
    pid_controller = Node(
        package='triton_pid_controller',
        namespace='/triton/controls',
        executable='triton_pid_controller',
        name='triton_pid_controller',
        output='screen', 
        parameters=[
            config,
            {'use_sim_time': True}
        ]
    )

    ld.add_action(pid_controller)

    return ld
