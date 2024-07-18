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

        self.declare_parameter('num_thrusters', 6)

        self.num_thrusters = self.get_parameter('num_thrusters').get_parameter_value().integer_value

        self.thrust_allocator_sub = self.create_subscription(
            Float64MultiArray,
            '/triton/controls/output_forces',
            self.thrust_allocator_callback,
            10
        )

        self.thrust_pub_dict = {
            num: self.create_publisher(
                Wrench,
                '/triton/gazebo_drivers/thruster_{}'.format(num),
                10)
            for num in range(1, self.num_thrusters+1)
        }
        self.get_logger().info('Simulation thrust mapper succesfully started!')
    
    def thrust_allocator_callback(self, msg):
        if len(msg.data) != self.num_thrusters:
            self.get_logger.warn("Controller output thruster count doesn't match simulation thruster count!")
        else:
            for i in range(1,self.num_thrusters+1):
                wrench_msg = Wrench()
                wrench_msg.force.z = msg.data[i-1]
                self.thrust_pub_dict[i].publish(wrench_msg)

def main(args=None):
    rclpy.init(args=args)

    sim_thrust_mapper = SimThrustMapper()
    rclpy.spin(sim_thrust_mapper)

    sim_thrust_mapper.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
