import yaml

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from triton_interfaces.srv import ConfigurePipeline
from triton_interfaces.msg import PipelineType
from triton_interfaces.action import RunPipeline
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType

class StateManager(Node):

    def __init__(self):
        super().__init__('state_manager')

        self.declare_parameters(
            namespace='',
            parameters=[
                ('pipelines', []),
                ('parameters', []),
        ])

        pipelines_params, parameters_params = self.get_parameters(['pipelines', 'parameters'])

        self.pipelines = pipelines_params.get_parameter_value().string_array_value
        self.parameters = list(map(
            lambda x: yaml.load(x),
            (parameters_params.get_parameter_value().string_array_value)
        ))

        if len(self.pipelines) != len(self.parameters):
            self.get_logger().warn("Number of pipelines does not equal number of parameters")

        self.get_logger().info(str(self.pipelines))
        self.get_logger().info(str(self.parameters))

        self.configure_client = self.create_client(
            ConfigurePipeline, '/triton/configure_pipeline')

        self.run_client = ActionClient(
            self, RunPipeline, '/triton/run_pipeline')

        while not self.run_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn(
                'Run action not available, make sure you have launched the pipeline...')

        while not self.configure_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn(
                'Configure service not available, make sure you have launched the pipeline...')

        self.get_logger().info('State Manager successfully started!')

    def run(self):
        for i in range(len(self.pipelines)):
            pipeline = self.pipelines[i]
            parameters = self._create_params_list(self.parameters[i])
            future = self._send_configure_request(pipeline)
            success = False
            # Configure pipeline 
            while rclpy.ok():
                rclpy.spin_once(self)
                if future.done():
                    success = future.result().success
                    break
            if not success:
                self.get_logger().error("Error while configuring pipeline")
                break
            # Run pipeline
            future = self._run_current_pipeline()
            while rclpy.ok():
                rclpy.spin_once(self)
                if future.done():
                    future = future.result().get_result_async()
                    break
            # Get pipeline result
            while rclpy.ok():
                rclpy.spin_once(self)
                if future.done():
                    result = future.result().result
                    self.get_logger().info(str(result))
                    break

    def _create_params_list(self, params):
        params_list = []
        for param_name, param_value in params.items():
            param = Parameter()
            param.name = param_name
            param.value = self._parse_param_value(param_value)
            params_list.append(param)
        self.get_logger().info(str(params_list))
        return params_list

    def _parse_param_value(self, value):
        param_value = ParameterValue()
        param_type = self._parse_param_type(value)
        param_value.type = param_type
        if param_type == ParameterType.PARAMETER_BOOL:
            param_value.bool_value = value
        elif param_type == ParameterType.PARAMETER_INTEGER:
            param_value.integer_value = value
        elif param_type == ParameterType.PARAMETER_DOUBLE:
            param_value.double_value = value
        elif param_type == ParameterType.PARAMETER_STRING:
            param_value.string_value = value
        elif param_type == ParameterType.PARAMETER_BYTE_ARRAY:
            param_value.byte_array_value = value
        elif param_type == ParameterType.PARAMETER_BOOL_ARRAY:
            param_value.bool_array_value= value
        elif param_type == ParameterType.PARAMETER_INTEGER_ARRAY:
            param_value.integer_array_value = value
        elif param_type == ParameterType.PARAMETER_DOUBLE_ARRAY:
            param_value.double_array_value= value
        elif param_type == ParameterType.PARAMETER_STRING_ARRAY:
            param_value.string_array_value = value
        return param_value

    def _parse_param_type(self, value):
        if isinstance(value, bool):
            return ParameterType.PARAMETER_BOOL
        elif isinstance(value, int):
            return ParameterType.PARAMETER_INTEGER
        elif isinstance(value, float):
            return ParameterType.PARAMETER_DOUBLE
        elif isinstance(value, str):
            return ParameterType.PARAMETER_STRING
        elif isinstance(value, list) and (len(value) > 0):
            if isinstance(value[0], bytes):
                return ParameterType.PARAMETER_BYTE_ARRAY
            elif isinstance(value[0], bool):
                return ParameterType.PARAMETER_BOOL_ARRAY
            elif isinstance(value[0], int):
                return ParameterType.PARAMETER_INTEGER_ARRAY
            elif isinstance(value[0], float):
                return ParameterType.PARAMETER_DOUBLE_ARRAY
            elif isinstance(value[0], str):
                return ParameterType.PARAMETER_STRING_ARRAY
        else: 
            return ParameterType.PARAMETER_BOOL # Default type

    def _send_configure_request(self, next_type):
        req = ConfigurePipeline.Request()
        pipeline_type = PipelineType()
        pipeline_type.type = next_type
        req.pipeline_type = pipeline_type
        return self.configure_client.call_async(req)

    def _run_current_pipeline(self):
        goal_msg = RunPipeline.Goal()
        goal_msg.input = 0
        return self.run_client.send_goal_async(goal_msg)

def main():
    rclpy.init()
    state_manager = StateManager()
    state_manager.run()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
