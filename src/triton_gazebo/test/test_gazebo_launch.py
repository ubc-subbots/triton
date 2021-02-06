import os
import unittest
import subprocess

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
    launch_file_name = 'gazebo_launch.py'

    launch_action = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory(pkg_name), 'launch', launch_file_name)
        )
    )

    ld.add_action(launch_action)
    ld.add_action(launch_testing.actions.ReadyToTest())
    return ld 


class TestGazeboLaunchInit(unittest.TestCase):


    @classmethod
    def tearDownClass(self):
        # gzserver and gzclient aren't always killed on exit
        subprocess.call(['pkill gz'], shell=True, stdout=subprocess.PIPE)


    def test_gazebo_init(self, proc_info, proc_output):
        proc_output.assertWaitFor('Gazebo multi-robot simulator, version 11')
        proc_output.assertWaitFor('Copyright (C) 2012 Open Source Robotics Foundation.')
        proc_output.assertWaitFor('Released under the Apache 2 License.')
        proc_output.assertWaitFor('http://gazebosim.org')


    def test_no_file_not_found_err(self, proc_info, proc_output):
        with self.assertRaises(AssertionError):
            proc_output.assertWaitFor(
                "Could not open file",
                timeout=10
            )


