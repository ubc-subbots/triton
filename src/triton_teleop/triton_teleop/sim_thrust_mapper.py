import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Wrench
from std_msgs.msg import Float64MultiArray


class SimThrustMapper(Node):
    """
    Simulation thrust mapper
    """

    def __init__(self):
        super().__init__('sim_thrust_mapper')

        self.thrust_allocator_sub = self.create_subscription(
            Float64MultiArray,
            '/triton/controls/output_forces',
            self.thrust_allocator_callback,
            10
        )

        self.thrust_pub_dict = {
            1: self.create_publisher(
                Wrench,
                '/triton/gazebo_drivers/thruster_1',
                10),
            2: self.create_publisher(
                Wrench,
                '/triton/gazebo_drivers/thruster_2',
                10),
            3: self.create_publisher(
                Wrench,
                '/triton/gazebo_drivers/thruster_3',
                10),
            4: self.create_publisher(
                Wrench,
                '/triton/gazebo_drivers/thruster_4',
                10),
            5: self.create_publisher(
                Wrench,
                '/triton/gazebo_drivers/thruster_5',
                10),
            6: self.create_publisher(
                Wrench,
                '/triton/gazebo_drivers/thruster_6',
                10),
        }

        self.get_logger().info('Simulation thrust mapper succesfully started!')
    
    def thrust_allocator_callback(self, msg):
        for i in range(1,7):
            wrench_msg = Wrench()
            wrench_msg.force.x = msg.data[i-1]
            self.thrust_pub_dict[i].publish(wrench_msg)


def main(args=None):
    rclpy.init(args=args)

    sim_thrust_mapper = SimThrustMapper()
    rclpy.spin(sim_thrust_mapper)

    sim_thrust_mapper.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
