import unittest
import pytest

import launch_testing
import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.actions import Node

from triton_interfaces.action import RunPipeline
from triton_interfaces.srv import ConfigurePipeline
from triton_interfaces.msg import PipelineType, PipelineFeedback

import rclpy
from rclpy.action import ActionClient


@pytest.mark.rostest
def generate_test_description():
    ld = launch.LaunchDescription()

    pipeline_manager = Node(
        name='pipeline_manager',
        namespace='/triton',
        package='triton_pipeline',
        executable='pipeline_manager',
        output='screen'
    )

    pipeline_container = ComposableNodeContainer(
        name='pipeline',
        namespace='/triton',
        package='rclcpp_components',
        executable='component_container'
    )

    ld.add_action(pipeline_manager)
    ld.add_action(pipeline_container)
    ld.add_action(launch_testing.actions.ReadyToTest())

    return ld, {
        'pipeline_manager': pipeline_manager,
        'pipeline_container': pipeline_container
    }


class TestPipeline(unittest.TestCase):


    @classmethod
    def setUpClass(cls):
        rclpy.init()


    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()


    def setUp(self):
        self.node = rclpy.create_node('test_node')

        self.configure_client = self.node.create_client(
            ConfigurePipeline,
            '/triton/configure_pipeline'
        )
        while not self.configure_client.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().warn('Configure pipeline service not available, waiting again...')

        self.run_client = ActionClient(
            self.node,
            RunPipeline,
            '/triton/run_pipeline'
        )
        while not self.run_client.wait_for_server(timeout_sec=1.0):
            self.node.get_logger().warn('Run pipeline service not available, waiting again...')


    def tearDown(self):
        self.run_client.destroy()
        self.node.destroy_node()


    def test_run_no_configure(self, pipeline_manager, proc_info, proc_output):
        goal_msg = RunPipeline.Goal()
        goal_msg.input = 0
        future = self.run_client.send_goal_async(goal_msg)

        future_count = 0;
        while rclpy.ok():
            rclpy.spin_once(self.node)
            if future.done():
                if future_count == 0:
                    future = future.result().get_result_async()
                future_count += 1
                if future_count == 2:
                    result = future.result().result
                    if result.output == 1:
                        proc_output.assertWaitFor(
                            expected_output='Pipeline is not configured')
                    else:
                        self.fail('Should abort due to unconfigured pipeline')
                    break


    def test_run_comp_not_valid(self, pipeline_manager, proc_info, proc_output):
        component = 'example::DoesNotExist'

        req = ConfigurePipeline.Request()
        pipeline_type = PipelineType()
        test_type = 'example'
        pipeline_type.type = test_type
        req.pipeline_type = pipeline_type
        test_file_name = 'test_run_comp_not_valid'
        req.config_file_name = test_file_name;
        self.configure_client.call_async(req)

        goal_msg = RunPipeline.Goal()
        goal_msg.input = 0
        future = self.run_client.send_goal_async(goal_msg)

        future_count = 0;
        while rclpy.ok():
            rclpy.spin_once(self.node)
            if future.done():
                if future_count == 0:
                    future = future.result().get_result_async()
                future_count += 1
                if future_count == 2:
                    result = future.result().result
                    if result.output == 1:
                        proc_output.assertWaitFor(
                            expected_output='The component {} was not loaded successfully'.format(component))
                    else:
                        self.fail('Should abort from component not able to load')
                    break


    def test_run_pkg_not_valid(self, pipeline_manager, proc_info, proc_output):
        component = 'triton_does_not_exist'

        req = ConfigurePipeline.Request()
        pipeline_type = PipelineType()
        test_type = 'example'
        pipeline_type.type = test_type
        req.pipeline_type = pipeline_type
        test_file_name = 'test_run_pkg_not_valid'
        req.config_file_name = test_file_name;
        self.configure_client.call_async(req)

        goal_msg = RunPipeline.Goal()
        goal_msg.input = 0
        future = self.run_client.send_goal_async(goal_msg)

        future_count = 0;
        while rclpy.ok():
            rclpy.spin_once(self.node)
            if future.done():
                if future_count == 0:
                    future = future.result().get_result_async()
                future_count += 1
                if future_count == 2:
                    result = future.result().result
                    if result.output == 1:
                        proc_output.assertWaitFor(
                            expected_output='The component {} was not loaded successfully'.format(component))
                    else:
                        self.fail('Should abort from component not able to load')
                    break

