from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    ld = LaunchDescription()

    world_arg = DeclareLaunchArgument(
            'world',
            default_value='cube.world',
            description='Gazebo world file'
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            get_package_share_directory('triton_gazebo') + '/launch/gazebo_launch.py'
        ),
        launch_arguments={'world': LaunchConfiguration('world')}.items()
    )

    thrust_allocator = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            get_package_share_directory('triton_controls') + '/launch/thrust_allocator_launch.py'
        )
    )

    keyboard_teleop = Node(
        name='keyboard_teleop',
        namespace='/triton/teleop',
        package='triton_teleop',
        executable='keyboard_teleop',
        output='screen',
    )

    '''
    sim_thrust_mapper = Node(
        name='sim_thrust_mapper',
        namespace='/triton/teleop',
        package='triton_teleop',
        executable='sim_thrust_mapper',
        output='screen',
    )
    '''

    ld.add_action(world_arg)
    ld.add_action(gazebo)
    ld.add_action(thrust_allocator)
    ld.add_action(keyboard_teleop)
    #ld.add_action(sim_thrust_mapper)

    return ld