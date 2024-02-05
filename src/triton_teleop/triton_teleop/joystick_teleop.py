import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Wrench
from sensor_msgs.msg import Joy

# heavy inspo from https://github.com/khengyun/ros2_joystick_py/blob/main/joy.py

class JoystickTeleop(Node):
    """
    Joystick teleop controllers
    """
        
    def joy_callback(self, cmd):
        """
        Handles joystick movement

        @param msg: They joystick command
        """

        msg = Wrench()
        
        #bottom LR buttons on top of the joystick == roll in vertical plane
        if cmd.buttons[2]:
            msg.torque.x = self.torque_mags[0]
        elif cmd.buttons[3]:
            msg.torque.x = -self.torque_mags[0]
        
        #small top joystick x == turn in horizontal plane
        if cmd.axes[4] > 0:
            msg.torque.z = self.torque_mags[2]
        elif cmd.axes[4] < 0:
            msg.torque.z = -self.torque_mags[2]
        
        #y-axis of joystick == forward/back
        if cmd.axes[1] > 0:
            msg.force.x = self.force_mags[0]
        elif cmd.axes[1] < 0:
            msg.force.x = -self.force_mags[0]
        
        # x-axis of joystick == crabbing left/right
        if cmd.axes[0] > 0:
            msg.force.y = self.force_mags[1]
        elif cmd.axes[0] < 0:
            msg.force.y = -self.force_mags[1]

        # small top joystick == raise/lower
        if cmd.axes[5] > 0: 
            msg.force.z = self.force_mags[2]
        elif cmd.axes[5] < 0: 
            msg.force.z = -self.force_mags[2]


        self.force_pub.publish(msg)


    def __init__(self):
        super().__init__('joystick_teleop')

        self.force_mags = [15.0, 15.0, 15.0]  # [x,y,z]
        self.torque_mags = [15.0, 0.0, 15.0]  # [x,y,z]

        self.force_pub = self.create_publisher(
            Wrench,
            '/triton/controls/input_forces',
            10
        )

        self.joy_cmds = self.create_subscription(
            Joy,
            '/joy',
            lambda cmd: self.joy_callback(cmd),
            10
        )

        # self._start()

        self.get_logger().info('Joystick teleop succesfully started!')

def main(args=None):
    rclpy.init(args=args)
    joystick_teleop = JoystickTeleop()
    try:
        rclpy.spin(joystick_teleop)
    except KeyboardInterrupt:
        pass # To force exit code 0

    joystick_teleop.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()