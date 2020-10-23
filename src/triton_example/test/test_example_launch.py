import os
import unittest

import pytest

import launch_testing
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch.actions import  ExecuteProcess, TimerAction 
from ament_index_python.packages import get_package_share_directory


@pytest.mark.rostest
def generate_test_description():
    ld = LaunchDescription()

    pkg_name = 'triton_example'
    components = ['example::ComponentOne', 'example::ComponentTwo']

    example_container = ComposableNodeContainer(
        name='example_container',
        namespace='/',
        package='rclcpp_components',
        executable='component_container',
        output='screen'
    )

    # There is a bug when using launch_test with ComposableNode, need to use cli
    load_processes = {}
    load_actions = []
    for component in components:
        load_component = ExecuteProcess(
            cmd=['ros2', 'component', 'load' ,'/example_container', pkg_name, component]
        )
        name = component[component.index("::")+2:].lower()
        load_processes[name] = load_component
        load_actions.append(load_component)
    # Delay so container can start up before loading
    delayed_load = TimerAction(
        period=3.0,
        actions=load_actions
    )

    ld.add_action(example_container)
    ld.add_action(delayed_load)
    ld.add_action(launch_testing.actions.ReadyToTest())
    return ld, load_processes


class TestExampleLaunchInit(unittest.TestCase):


    def test_component_one_init(self, proc_info, proc_output, componentone):
        proc_output.assertWaitFor('Component One succesfully started!')
        proc_info.assertWaitForShutdown(process=componentone)


    def test_component_two_init(self, proc_info, proc_output, componenttwo):
        proc_output.assertWaitFor('Component Two succesfully started!')
        proc_info.assertWaitForShutdown(process=componenttwo)


@launch_testing.post_shutdown_test()
class TestExampleLaunchExit(unittest.TestCase):


    def test_exit_code(self, proc_info, proc_output):
        launch_testing.asserts.assertExitCodes(proc_info)