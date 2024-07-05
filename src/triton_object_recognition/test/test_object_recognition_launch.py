import os
import unittest

import pytest

import launch_testing
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.actions import  ExecuteProcess, TimerAction 
from ament_index_python.packages import get_package_share_directory


@pytest.mark.rostest
def generate_test_description():
    ld = LaunchDescription()

    pkg_name = 'triton_object_recognition'
    component = 'triton_object_recognition::ObjectRecognizer'

    config = os.path.join(
        get_package_share_directory('triton_object_recognition'),
        'config',
        'tiny_yolov3.yaml'
    )

    object_recognizer_container = ComposableNodeContainer(
        name='object_recognizer_container',
        namespace='/',
        package='rclcpp_components',
        executable='component_container',
        output='screen'
    )

    # There is a bug when using launch_test with ComposableNode, need to use cli
    load_processes = {}
    load_actions = []
    
    load_component = ExecuteProcess(
            cmd=['ros2', 'component', 'load' ,'/object_recognizer_container', pkg_name, component]
        )
    load_processes["objectrecognizer"] = load_component
    load_actions.append(load_component)
    # Delay so container can start up before loading
    delayed_load = TimerAction(
        period=3.0,
        actions=load_actions
    )

    ld.add_action(object_recognizer_container)
    ld.add_action(delayed_load)
    ld.add_action(launch_testing.actions.ReadyToTest())
    return ld, load_processes


class TestObjectRecognitionLaunchInit(unittest.TestCase):


    def test_object_recognizer_init(self, proc_info, proc_output, objectrecognizer):
        proc_output.assertWaitFor('Object Recognizer successfully started!',timeout=900)
        proc_info.assertWaitForShutdown(process=objectrecognizer)


@launch_testing.post_shutdown_test()
class TestObjectRecognitionLaunchExit(unittest.TestCase):


    def test_exit_code(self, proc_info, proc_output):
        launch_testing.asserts.assertExitCodes(proc_info)
