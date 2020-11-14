import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Wrench
from pynput import keyboard


class KeyboardTeleop(Node):
    """
    Keyboard teleop controller
    """

    def __init__(self):
        super().__init__('keyboard_teleop')
        
        self.force_pub = self.create_publisher(
            Wrench,
            '/triton/teleop/force', 
            10
        )  

        self._start()

        self.get_logger().info('Keyboard teleop succesfully started!')


    def _start(self):
        """
        Sets up the keyboard listeners
        """
        self.listener = keyboard.Listener(
            on_press=self._on_press,
            on_release=self._on_release
        )
        self.listener.start()


    def _on_press(self, key):
        """
        Handles key presses

        @param key: They character of the key pressed
        """
        self.get_logger().info("KEY PRESSED: {}\n".format(key))
        # TODO: implement this

        # Publishing example
        msg = Wrench()
        msg.force.x = 1.0
        self.force_pub.publish(msg)


    def _on_release(self, key):
        """
        Handles key releases

        @param key: They character of the key released
        """
        self.get_logger().info("KEY RELEASED: {}\n".format(key))
        # TODO: implement this


def main(args=None):
    rclpy.init(args=args)

    keyboard_teleop = KeyboardTeleop()
    rclpy.spin(keyboard_teleop)

    keyboard_teleop.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()