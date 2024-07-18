import unittest
import time
import pytest

import launch
import launch_testing
from launch_ros.actions import ComposableNodeContainer
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory

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

    ld.add_action(pipeline_manager)
    ld.add_action(launch_testing.actions.ReadyToTest())

    return ld, {
        'pipeline_manager': pipeline_manager,
    }

class TestPipelineInitUnsuccess(unittest.TestCase):

    def test_unsuccessful_init(self, pipeline_manager, proc_output):
        for _ in range(5):
            proc_output.assertWaitFor(
                expected_output='Pipeline loading service not available, waiting again...',
                process=pipeline_manager
            )
            time.sleep(1.1)