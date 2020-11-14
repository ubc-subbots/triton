import os
import time
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
    """
    This tests the thrust allocation system on a simple 6 thruster configuration
    which contains 2 thrusters for each axis, where each is symmetrical about the origin
    """

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def callback(self, msg):
        self.output_forces_msg = msg

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
            self.callback,
            10
        )

    def tearDown(self):
        self.node.destroy_node()

    def test_x_force(self, proc_output, thrust_allocator):
        """
        This tests that the force is divided equally to both x-contributuing thrusters,
        and that no force is given to the y or z-contributing thrusters
        """
        msg = Wrench()
        msg.force.x = 1.0
        while rclpy.ok():
            self.forces_pub.publish(msg)
            time.sleep(0.5)
            rclpy.spin_once(self.node, timeout_sec=0.2)
            if self.output_forces_msg is not None:
                data = self.output_forces_msg.data
                self.assertAlmostEqual(data[0], data[1])
                self.assertAlmostEqual(data[0]+data[1], 1.0)
                self.assertAlmostEqual(data[2], 0)
                self.assertAlmostEqual(data[3], 0)
                self.assertAlmostEqual(data[4], 0)
                self.assertAlmostEqual(data[5], 0)
                break
            
    def test_y_force(self, proc_output, thrust_allocator):
        """
        This tests that the force is divided equally to both y-contributuing thrusters,
        and that no force is given to the x or z-contributing thrusters
        """
        msg = Wrench()
        msg.force.y = 1.0
        while rclpy.ok():
            self.forces_pub.publish(msg)
            time.sleep(0.5)
            rclpy.spin_once(self.node, timeout_sec=0.2)
            if self.output_forces_msg is not None:
                data = self.output_forces_msg.data
                self.assertAlmostEqual(data[2], data[3])
                self.assertAlmostEqual(data[2]+data[3], 1.0)
                self.assertAlmostEqual(data[0], 0)
                self.assertAlmostEqual(data[1], 0)
                self.assertAlmostEqual(data[4], 0)
                self.assertAlmostEqual(data[5], 0)
                break
            
    def test_z_force(self, proc_output, thrust_allocator):
        """
        This tests that the force is divided equally to both z-contributuing thrusters,
        and that no force is given to the x or y-contributing thrusters
        """
        msg = Wrench()
        msg.force.z = 1.0
        while rclpy.ok():
            self.forces_pub.publish(msg)
            time.sleep(0.5)
            rclpy.spin_once(self.node, timeout_sec=0.2)
            if self.output_forces_msg is not None:
                data = self.output_forces_msg.data
                self.assertAlmostEqual(data[4], data[5])
                self.assertAlmostEqual(data[4]+data[5], 1.0)
                self.assertAlmostEqual(data[0], 0)
                self.assertAlmostEqual(data[1], 0)
                self.assertAlmostEqual(data[2], 0)
                self.assertAlmostEqual(data[3], 0)
                break

    def test_z_torque(self, proc_output, thrust_allocator):
        """
        This tests that the torque is divided equally between x,y contributing 
        thrusters and that for a pair of thrusters on an axis, one has negative
        magnitude of the other. Also tests that the sum of torques adds to 1 and
        no force is given to the z-contributing thrusters
        """
        msg = Wrench()
        msg.torque.z = 1.0
        while rclpy.ok():
            self.forces_pub.publish(msg)
            time.sleep(0.5)
            rclpy.spin_once(self.node, timeout_sec=0.2)
            if self.output_forces_msg is not None:
                data = self.output_forces_msg.data
                l1,l2,l3,l4 = 1.0,-1.0,1.0,-1.0
                self.assertAlmostEqual(data[0], -data[1])
                self.assertAlmostEqual(data[2], -data[3])
                self.assertAlmostEqual(data[0]*l1+data[1]*l2+data[2]*l3+data[3]*l4, 1.0)
                self.assertAlmostEqual(data[4],0)
                self.assertAlmostEqual(data[5],0)
                break

    def test_x_torque(self, proc_output, thrust_allocator):
        """
        This tests that the torque is divided equally between z-contributing 
        thrusters and that one has negative magnitude of the other. Also tests 
        that the sum of torques adds to 1 and no force is given to the  x or y
        contributing thrusters
        """
        msg = Wrench()
        msg.torque.x = 1.0
        while rclpy.ok():
            self.forces_pub.publish(msg)
            time.sleep(0.5)
            rclpy.spin_once(self.node, timeout_sec=0.2)
            if self.output_forces_msg is not None:
                data = self.output_forces_msg.data
                l5,l6= 1.0,-1.0
                self.assertAlmostEqual(data[4], -data[5])
                self.assertAlmostEqual(data[4]*l5+data[5]*l6, 1.0)
                self.assertAlmostEqual(data[0],0)
                self.assertAlmostEqual(data[1],0)
                self.assertAlmostEqual(data[2],0)
                self.assertAlmostEqual(data[3],0)
                break

    def test_y_torque(self, proc_output, thrust_allocator):
        """
        This tests that no torque can be produced on this axis
        """
        msg = Wrench()
        msg.torque.y = 1.0
        while rclpy.ok():
            self.forces_pub.publish(msg)
            time.sleep(0.5)
            rclpy.spin_once(self.node, timeout_sec=0.2)
            if self.output_forces_msg is not None:
                data = self.output_forces_msg.data
                self.assertAlmostEqual(data[0],0)
                self.assertAlmostEqual(data[1],0)
                self.assertAlmostEqual(data[2],0)
                self.assertAlmostEqual(data[3],0)
                self.assertAlmostEqual(data[4],0)
                self.assertAlmostEqual(data[5],0)
                break

