import os
import unittest

import pytest

from geometry_msgs.msg import Wrench
import launch_testing
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


@pytest.mark.rostest
def generate_test_description():
    ld = LaunchDescription()

    pkg_name = 'triton_teleop'
    launch_file_name = 'keyboard_teleop_launch.py'

    launch_action = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory(pkg_name), 'launch', launch_file_name)
        )
    )

    ld.add_action(launch_action)
    ld.add_action(launch_testing.actions.ReadyToTest())
    return ld 

class TestKeyboardTeleopLaunchInit(unittest.TestCase):


    def test_keyboard_teleop_init(self, proc_info, proc_output):
        proc_output.assertWaitFor('Keyboard teleop succesfully started!')


@launch_testing.post_shutdown_test()
class TestKeyboardTeleopLaunchExit(unittest.TestCase):


    def test_exit_code(self, proc_info, proc_output):
        launch_testing.asserts.assertExitCodes(proc_info)