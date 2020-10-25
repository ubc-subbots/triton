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


    def tearDown(self):
        self.node.destroy_node()


    def test_configure_invalid_type(self, pipeline_manager, proc_info, proc_output):
        req = ConfigurePipeline.Request()
        pipeline_type = PipelineType()
        test_type = 'no_exist'
        pipeline_type.type = test_type
        req.pipeline_type = pipeline_type
        future = self.configure_client.call_async(req)
        while rclpy.ok():
            rclpy.spin_once(self.node)
            if future.done():
                res = future.result()
                if res.success is False:
                    proc_output.assertWaitFor(
                        expected_output='Configuration of pipeline type "{}"' \
                                        ' is not allowed, no such type'.format(test_type),
                        process=pipeline_manager
                    )
                else:
                    self.fail('Expected configure pipeline service to fail' \
                                ' due to non existent type')
                break


    def test_configure_valid_type_no_yaml(self, pipeline_manager, proc_info, proc_output):
        req = ConfigurePipeline.Request()
        pipeline_type = PipelineType()
        test_type = 'example'
        pipeline_type.type = test_type
        req.pipeline_type = pipeline_type
        test_file_name = 'test_no_yaml'
        req.config_file_name = test_file_name;
        future = self.configure_client.call_async(req)
        while rclpy.ok():
            rclpy.spin_once(self.node)
            if future.done():
                res = future.result()
                if res.success is False:
                    proc_output.assertWaitFor(
                        expected_output='Could not find {}.yaml'.format(test_file_name))
                else:
                    self.fail('Expected configure pipeline service to fail' \
                                ' due to no yaml associated to pipeline type')
                break


    def test_configure_valid_type_bad_yaml(self, pipeline_manager, proc_info, proc_output):
        req = ConfigurePipeline.Request()
        pipeline_type = PipelineType()
        test_type = 'example'
        pipeline_type.type = test_type
        req.pipeline_type = pipeline_type
        test_file_name = 'test_bad_yaml'
        req.config_file_name = test_file_name;
        future = self.configure_client.call_async(req)
        while rclpy.ok():
            rclpy.spin_once(self.node)
            if future.done():
                res = future.result()
                if res.success is False:
                    proc_output.assertWaitFor(
                        expected_output='Could not parse {}.yaml'.format(test_file_name))
                else:
                    self.fail('Expected configure pipeline service to fail'
                                ' due to bad yaml associated to pipeline type')
                break


    def test_configure_valid_type_yaml_no_namespace(self, pipeline_manager, proc_info, proc_output):
        req = ConfigurePipeline.Request()
        pipeline_type = PipelineType()
        test_type = 'example'
        pipeline_type.type = test_type
        req.pipeline_type = pipeline_type
        test_file_name = 'test_no_namespace'
        req.config_file_name = test_file_name;
        future = self.configure_client.call_async(req)
        while rclpy.ok():
            rclpy.spin_once(self.node)
            if future.done():
                res = future.result()
                if res.success is False:
                    proc_output.assertWaitFor(
                        expected_output='Pipeline config YAML needs pipeline namespace')
                else:
                    self.fail('Expected configure pipeline service to fail'
                                ' due to no pipeline parameter namespace in yaml'
                                ' associated to pipeline type')
                break


    def test_configure_valid_type_yaml_no_param(self, pipeline_manager, proc_info, proc_output):
        param = 'no_param_for_this'

        req = ConfigurePipeline.Request()
        pipeline_type = PipelineType()
        test_type = 'example'
        pipeline_type.type = test_type
        req.pipeline_type = pipeline_type
        test_file_name = 'test_no_param'
        req.config_file_name = test_file_name;
        future = self.configure_client.call_async(req)
        while rclpy.ok():
            rclpy.spin_once(self.node)
            if future.done():
                res = future.result()
                if res.success is False:
                    proc_output.assertWaitFor(
                        expected_output='Could not get the pipeline parameter "{}", does not exist'.format(param))
                else:
                    self.fail('Expected configure pipeline service to fail'
                                ' due to no parameter for yaml item'
                                ' associated to pipeline type')
                break

