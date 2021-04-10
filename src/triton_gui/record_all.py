import launch
import os

def generate_launch_description():
    return launch.LaunchDescription([
        launch.actions.ExecuteProcess(
            cmd=['ros2', 'bag', 'record', '-a'],
            output='screen',
            # Below will print something on the terminal to let us know just when the script terminates. 
            # Useful to ensure a proper recording
            log_cmd = 'True'
        )
    ]), os.getpid()