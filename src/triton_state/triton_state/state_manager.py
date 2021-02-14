import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from triton_interfaces.srv import ConfigurePipeline
from triton_interfaces.msg import PipelineType
from triton_interfaces.action import RunPipeline


class StateManager(Node):

    def __init__(self):
        super().__init__('state_manager')
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

    def send_configure_request(self, next_type):
        req = ConfigurePipeline.Request()
        pipeline_type = PipelineType()
        pipeline_type.type = next_type
        req.pipeline_type = pipeline_type
        return self.configure_client.call_async(req)

    def run_current_pipeline(self):
        goal_msg = RunPipeline.Goal()
        goal_msg.input = 0
        return self.run_client.send_goal_async(goal_msg)

    def run(self):
        pipeline_type_sequence = ['example', 'example', 'example']
        for pipeline_type in pipeline_type_sequence:
            future = self.send_configure_request(pipeline_type)
            result = False
            # Configure pipeline 
            while rclpy.ok():
                rclpy.spin_once(self)
                if future.done():
                    res = future.result()
                    self.get_logger().info(str(future.result().success))
                    break
            # Run pipeline
            future = self.run_current_pipeline()
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

def main():
    rclpy.init()
    state_manager = StateManager()
    state_manager.run()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
