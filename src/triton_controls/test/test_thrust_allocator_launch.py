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

    pkg_name = 'triton_controls'
    launch_file_name = 'thrust_allocator_launch.py'

    launch_action = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory(pkg_name), 'launch', launch_file_name)
        )
    )

    ld.add_action(launch_action)
    ld.add_action(launch_testing.actions.ReadyToTest())
    return ld 

class TestThrustAllocatorLaunchInit(unittest.TestCase):


    def test_thrust_allocator_init(self, proc_info, proc_output):
        proc_output.assertWaitFor('Thrust Allocator succesfully started!')


class TestThrustAllocator(unittest.TestCase):


    @classmethod
    def setUpClass(cls):
        rclpy.init()


    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()


    def setUp(self):
        self.node = rclpy.create_node('test_node')
        self.forces_pub = self.node.create_publisher(
            Wrench,
            "/triton/controls/input_forces",
            10)


    def tearDown(self):
        self.node.destroy_node()


@launch_testing.post_shutdown_test()
class TestThrustAllocatorLaunchExit(unittest.TestCase):


    def test_exit_code(self, proc_info, proc_output):
        launch_testing.asserts.assertExitCodes(proc_info)