import os
import unittest

import pytest

import launch_testing
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


@pytest.mark.rostest
def generate_test_description():
    ld = LaunchDescription()

    pkg_name = 'triton_gazebo'
    launch_file_name = 'underwater_camera_launch.py'

    launch_action = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory(pkg_name), 'launch', launch_file_name)
        )
    )

    ld.add_action(launch_action)
    ld.add_action(launch_testing.actions.ReadyToTest())
    return ld 

class TestUnderwaterCameraLaunchInit(unittest.TestCase):


    def test_underwater_camera_init(self, proc_info, proc_output):
        proc_output.assertWaitFor('Underwater Camera succesfully started!')


@launch_testing.post_shutdown_test()
class TestExampleLaunchExit(unittest.TestCase):


    def test_exit_code(self, proc_info, proc_output):
        launch_testing.asserts.assertExitCodes(proc_info)
