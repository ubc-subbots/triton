import yaml

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from triton_interfaces.srv import ConfigurePipeline
from triton_interfaces.msg import PipelineType
from triton_interfaces.action import RunPipeline
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType

class PipelineSequenceManager(Node):

    def __init__(self):
        super().__init__('pipeline_sequence_manager')

        self.declare_parameters(
            namespace='',
            parameters=[
                ('pipeline_sequence', []),
        ])

        self.pipelines = self.get_parameter('pipeline_sequence').get_parameter_value().string_array_value

        self.get_logger().info('Pipeline Sequence ' + str(self.pipelines))

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

        self.get_logger().info('Pipeline Sequence Manager successfully started!')

    def run(self):
        """
        Runs the pipeline sequence

        For each pipeline type in the sequence, configures the pipeline with the type,
        then runs the pipeline and gets the result.
        """
        for i in range(len(self.pipelines)):
            pipeline = self.pipelines[i]
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
                    break

    def _send_configure_request(self, next_type):
        """
        Helper for sending the ConfigurePipeline request

        @param next_type: The next pipeline type to configure
        """
        req = ConfigurePipeline.Request()
        pipeline_type = PipelineType()
        pipeline_type.type = next_type
        req.pipeline_type = pipeline_type
        return self.configure_client.call_async(req)

    def _run_current_pipeline(self):
        """
        Helper for running the current pipeline
        """
        goal_msg = RunPipeline.Goal()
        goal_msg.input = 0
        return self.run_client.send_goal_async(goal_msg)

def main():
    rclpy.init()
    state_manager = PipelineSequenceManager()
    state_manager.run()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
