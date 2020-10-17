import os
import time
import yaml
import re

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.executors import MultiThreadedExecutor
from ament_index_python.packages import get_package_share_directory

from triton_interfaces.action import RunPipeline
from triton_interfaces.srv import ConfigurePipeline
from triton_interfaces.msg import PipelineType, PipelineFeedback
from composition_interfaces.srv import LoadNode, UnloadNode, ListNodes


class PipelineManager(Node):

    """
    Pipeline Manager.

    Responsible for configuring the running the pipeline
    """

    """
    Pipeline manager constructor

    Declares the parameters for the pipeline manager, and sets
    up the neccessary action and service servers, and topic
    publishers and subscribrers.
    """
    def __init__(self):
        super().__init__('pipeline_manager')

        self.nodes_in_pipeline = []
        self.pipeline_success = False
        self.pipeline_abort = False
        self.pipeline_feedback_msg = ""
        # This will add all attributes starting with 'TYPE' to the list
        self.pipeline_types = []
        for typename in re.findall(r'TYPE_+.*', PipelineType.__doc__):
            self.pipeline_types.append(getattr(PipelineType, str(typename)))
        
        self.declare_parameters(
            namespace='pipeline',
            parameters=[
                ('components', ['']),
                ('remap_rules', ['']),
                ('pkg_name', ''),
                ('namespace', '')
        ])

        self.feedback_sub = self.create_subscription(
            PipelineFeedback,
            '/triton/pipeline_feedback',
            self.feedback_callback,
            10
        )

        self.pipeline_loading_client = self.create_client(
                LoadNode, 
                '/triton/pipeline/_container/load_node'
        )
        while not self.pipeline_loading_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Pipeline loading service not available, waiting again...')

        self.pipeline_unloading_client = self.create_client(
                UnloadNode, 
                '/triton/pipeline/_container/unload_node'
        )
        while not self.pipeline_unloading_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Pipeline unloading service not available, waiting again...')

        self.pipeline_listing_client = self.create_client(
                ListNodes, 
                '/triton/pipeline/_container/list_nodes'
        )
        while not self.pipeline_listing_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Pipeline listing service not available, waiting again...')

        self.pipeline_running_server = ActionServer(
            self, 
            RunPipeline, 
            'run_pipeline',
            self.run_pipeline
        )

        self.pipeline_configuring_server = self.create_service(
            ConfigurePipeline,
            'configure_pipeline',
            self.configure_pipeline
        )
            
    """
    Callback function for the pipeline feedback subscriber

    Given the pipeline feedback message, sets the manager state related
    to the success/failure of the pipeline

    @param msg A PipelineFeedback message
    """
    def feedback_callback(self, msg):
        self.pipeline_abort = msg.abort
        self.pipeline_success = msg.success
        self.pipeline_feedback_msg = msg.message


    """
    Callback function for the configure pipeline service

    Given the pipeline type in the request, locates and parses the
    associated config file and sets the parameters of the pipeline
    manager to be those given in the config file. Fails if the pipeline
    type is not a valid type, the config file is ill formatted or the 
    parameters in the config file cannot be set.

    @param request A service request
    @param response A service response
    """
    def configure_pipeline(self, request, response):
        pipeline_type = request.pipeline_type.type
        self.get_logger().info('Configuring pipeline for {}...'.format(pipeline_type))
        if pipeline_type not in self.pipeline_types:
            self.get_logger().warn(
                'Configuration of pipeline type "{}" is not allowed, no such type '.format(pipeline_type)
            )
            response.success = False
        else:
            manager_dir = get_package_share_directory('triton_pipeline')
            config_path = os.path.join(manager_dir, '{}.yaml'.format(pipeline_type))
            config_yaml = None
            with open(config_path, 'r') as stream:
                try:
                    config_yaml = yaml.safe_load(stream)
                except yaml.YAMLError as e:
                    self.get_logger().warn('Could not parse {}.yaml'.format(pipeline_type))
                    self.get_logger().error(str(e))
            if config_yaml is not None:
                response.success = self._load_params_from_yaml(config_yaml)
            else:
                response.success = False
        if response.success is True:
            self.get_logger().info('Pipeline configured for {}'.format(pipeline_type))
        return response


    """
    Callback function for the run pipeline action

    Loads the components needed for the pipeline type which the pipeline
    manager has been configured for into the pipeline, then waits until 
    the pipeline manager recieves feedback indicating that the loaded 
    pipeline has succeded or has had to abort. Succeeds or aborts the 
    goal handle accordingle then returns the output value using the 
    goal handle.

    @param goal_handle An action goal handle
    """
    def run_pipeline(self, goal_handle):
        self.get_logger().info('Running pipeline...')
        self.pipeline_success = False
        self.pipeline_abort = False
        self.pipeline_feedback_msg = ""
        pipeline_components = self.get_parameter('pipeline.components').value
        for component in pipeline_components:
            self._load_component(component)
        feedback = RunPipeline.Feedback()
        prev_msg = ""
        while not self.pipeline_success and not self.pipeline_abort:
            time.sleep(1)
            if len(self.pipeline_feedback_msg) > 0 and self.pipeline_feedback_msg is not prev_msg:
                # The success message may not always print due to race condition with feedback callback
                feedback.status = self.pipeline_feedback_msg
                self.get_logger().info(self.pipeline_feedback_msg)
                goal_handle.publish_feedback(feedback)
                prev_msg = self.pipeline_feedback_msg
        if self.pipeline_abort:
            goal_handle.abort()
        else: 
            goal_handle.succeed()
        self._unload_components()
        res = RunPipeline.Result()
        res.output = 0


    """
    Loads a component into the pipeline

    Uses the load node service the component container provides
    to load the node into the pipeline synchronously. Employs 
    busy waiting to recieve the response of whether or not the
    node was successfully loaded.

    @param component A node component string (i.e 'package::NodeName')
    """
    def _load_component(self, component):
        req = LoadNode.Request()
        req.package_name = self.get_parameter('pipeline.pkg_name').value
        req.plugin_name = component
        req.node_namespace = self.get_parameter('pipeline.namespace').value
        req.remap_rules = self.get_parameter('pipeline.remap_rules').value
        future = self.pipeline_loading_client.call_async(req)
        res = None
        def callback(future):
            nonlocal res
            res = future.result()
        future.add_done_callback(callback)
        while rclpy.ok() and res is None:
            time.sleep(0.1)
        if not res.success:
            self.get_logger().warn('The component {} was not loaded succesfully'
                                    .format(component))
        else:
            node_name = res.full_node_name
            self.nodes_in_pipeline.append(node_name)
            self.get_logger().info(node_name + ' loaded succesfully')


    """
    Unloads all the components from the pipeline

    Uses the list nodes service the component container provides
    to determine the names and ids of the nodes currently loaded
    in the pipeline. Employs busy waiting to recieve the response 
    about the listed nodes. Then, using the unique id associated to
    each node listed, unloads that node.
    """
    def _unload_components(self):
        req = ListNodes.Request()
        future = self.pipeline_listing_client.call_async(req)
        res = None
        def list_callback(future):
            nonlocal res
            res = future.result()
        future.add_done_callback(list_callback)
        while rclpy.ok() and res is None:
            time.sleep(0.1)
        for i in range(len(res.unique_ids)):
            unique_id = res.unique_ids[i]
            node_name = res.full_node_names[i]
            self._unload_component(unique_id, node_name)


    """
    Unloads a single component from the pipeline

    Uses the unload node service the component container provides
    to unload the node from the pipeline. Employs busy waiting to
    recieve the response of whether or not the node was unloaded 
    successfully.

    @param unique_id A unique identifer for the node
    @param node_name The namespaced node name string (i.e '/ns/node_name')

    """
    def _unload_component(self, unique_id, node_name):
        req = UnloadNode.Request()
        req.unique_id = unique_id
        future = self.pipeline_unloading_client.call_async(req)
        res = None
        def unload_callback(future):
            nonlocal res
            res = future.result()
        future.add_done_callback(unload_callback)
        while rclpy.ok() and res is None:
            time.sleep(0.1)
        if not res.success:
            self.get_logger().warn('{} was not unloaded succesfully'.format(node_name))
        else:
            self.get_logger().info('{} was unloaded succesfully'.format(node_name))


    """
    Sets the parameters for the pipeline manager from the config yaml

    Gets the pipeline parameters from the config yaml, turns them into
    parameter objects and sets them for the pipeline manager. Fails if 
    any of the parameters are ill formatted

    @param config_yaml A yaml structure object
    @return True if the parameters were loaded successfully, else False
    """
    def _load_params_from_yaml(self, config_yaml):
        pipeline_params = self._get_pipeline_params(config_yaml)
        if pipeline_params is not None:
            params_list = []
            for param_name, param_val in pipeline_params.items():
                try:
                    params_list.append(self._create_param_object(param_name, param_val))
                except rclpy.exceptions.ParameterException as e:
                    self.get_logger().warn('Could not get the pipeline parameter "{}", does not exist'
                                            .format(param_name))
                    self.get_logger().error(str(e))
            try:
                self.set_parameters(params_list)
                return True
            except rclpy.exceptions.ParameterException as e:
                self.get_logger().warn('Could not set pipeline parameters')
                self.get_logger().error(str(e))
                return False
        else:
            self.get_logger().warn('Pipeline config YAML needs pipeline namespace')
            return False


    """
    Traverses the config yaml until it finds the sub yaml
    structure under the key 'pipeline'

    @param config_yaml A yaml structure object
    @returns The pipeline parameter yaml object, or None if it doesn't exist
    """
    def _get_pipeline_params(self, config_yaml):
        for key, value in config_yaml.items():
            if key == "pipeline":
                return value
            elif isinstance(value, dict):
                return self._get_pipeline_params(value)
        return None


    """
    Creates a parameter object of the given name with the given value

    @param param_name Name of the parameter
    @param param_value Value of the parameter
    @returns A rclpy param object with given name, value and inferred type
    """
    def _create_param_object(self, param_name, param_val):
            param_name = 'pipeline.' + param_name
            curr_param_val = self.get_parameter(param_name).value
            param_type = rclpy.Parameter.Type.from_parameter_value(curr_param_val)
            new_param = rclpy.parameter.Parameter(
                param_name,
                param_type,
                param_val
            )
            return new_param
    
"""
Executes the pipeline manager node

Uses a multi threaded executor for the pipeline manager and 
continuously spins until stopped, then destroys the pipeline
manager and shuts down the infastructure
"""
def main(args=None):
    rclpy.init(args=args)
    pipeline_manager = PipelineManager()
    executor = MultiThreadedExecutor()
    executor.add_node(pipeline_manager)
    executor.spin()
    pipeline_manager.destroy_node()
    executor.shutdown()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
