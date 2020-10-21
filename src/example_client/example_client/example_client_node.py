import rclpy
from rclpy.node import Node

from triton_interfaces.srv import ConfigurePipeline
from triton_interfaces.msg import PipelineType


class ExampleClient(Node):


    def __init__(self):
        super().__init__('example_client')
        self.configure_client = self.create_client(ConfigurePipeline, '/triton/configure_pipeline')
        while not self.configure_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Configure service not available, make sure you have launched the pipeline...')


    def send_configure_request(self, next_type):
        req = ConfigurePipeline.Request()
        pipeline_type = PipelineType()
        pipeline_type.type = next_type
        req.pipeline_type = pipeline_type
        return self.configure_client.call_async(req)


    def run(self):
        pipeline_type_sequence = ['example', 'example', 'example']
        for pipeline_type in pipeline_type_sequence:
            future = self.send_configure_request(pipeline_type)
            while rclpy.ok():
                rclpy.spin_once(self)
                if future.done():
                    res = future.result()
                    print(future.result())
                    break


def main():
    rclpy.init()
    example_client = ExampleClient()
    example_client.run()
    example_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
