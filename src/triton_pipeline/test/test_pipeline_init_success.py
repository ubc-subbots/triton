import unittest
import pytest

import launch
import launch_testing
from launch_ros.actions import ComposableNodeContainer
from launch_ros.actions import Node


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


class TestPipelineInitSuccess(unittest.TestCase):

    def test_succesful_init(self, pipeline_manager, proc_info, proc_output):
        proc_output.assertWaitFor(
            expected_output='Pipeline manager succesfully started!',
            process=pipeline_manager
        )