import os
import unittest

import pytest

import rclpy
from geometry_msgs.msg import Wrench
from std_msgs.msg import Float64MultiArray
import launch_testing
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


@pytest.mark.rostest
def generate_test_description():
    ld = LaunchDescription()

    config = os.path.join(
        get_package_share_directory('triton_controls'),
        'test',
        'config',
        'test_thruster_simple_config.yaml'
    )

    thrust_allocator = Node(
        name='thrust_allocator',
        namespace='/',
        package='triton_controls',
        executable='thrust_allocator',
        output='screen',
        parameters=[config]
    )

    ld.add_action(thrust_allocator)
    ld.add_action(launch_testing.actions.ReadyToTest())
    return ld, {
        'thrust_allocator': thrust_allocator
    }


class TestThrustAllocator(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.output_forces_msg = None
        self.node = rclpy.create_node('test_node')
        self.forces_pub = self.node.create_publisher(
            Wrench,
            "/triton/controls/input_forces",
            10)
        self.forces_sub = self.node.create_subscription(
            Float64MultiArray,
            "output_forces",
            lambda msg: self.output_forces_msg = msg,
            10
        )

    def tearDown(self):
        self.node.destroy_node()

    def test_x_force(self, proc_output, thrust_allocator):
        msg = Wrench()
        msg.force.x = 1.0
        self.forces_pub.publish(msg)
        while rclpy.ok():
            rclpy.spin_once(self.node, timeout_sec=0.2)
            if self.output_forces_msg is not None:
                print(msg)
            


