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

        self.force_mags = [15.0, 15.0, 15.0]  # [x,y,z]
        self.torque_mags = [15.0, 0.0, 15.0]  # [x,y,z]

        self.force_pub = self.create_publisher(
            Wrench,
            '/triton/controls/input_forces',
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
        msg = Wrench()
        if key == keyboard.Key.up:
            msg.torque.x = -self.torque_mags[0]
        elif key == keyboard.Key.down:
            msg.torque.x = self.torque_mags[0]
        elif key == keyboard.Key.left:
            msg.torque.z = self.torque_mags[2]
        elif key == keyboard.Key.right:
            msg.torque.z = -self.torque_mags[2]
        elif 'char' in dir(key):
            if key.char == 'w':
                msg.force.x = self.force_mags[0]
            elif key.char == 's':
                msg.force.x = -self.force_mags[0]
            elif key.char == 'a':
                msg.force.y = self.force_mags[1]
            elif key.char == 'd':
                msg.force.y = -self.force_mags[1]
            elif key.char == 'q':
                msg.force.z = self.force_mags[2]
            elif key.char == 'z':
                msg.force.z = -self.force_mags[2]
        self.force_pub.publish(msg)

    def _on_release(self, key):
        """
        Handles key releases

        @param key: They character of the key released
        """
        msg = Wrench()
        if key == keyboard.Key.up:
            msg.torque.x = 0.0
        elif key == keyboard.Key.down:
            msg.torque.x = 0.0
        elif key == keyboard.Key.left:
            msg.torque.z = 0.0
        elif key == keyboard.Key.right:
            msg.torque.z = 0.0
        elif 'char' in dir(key):
            if key.char == 'w':
                msg.force.x = 0.0
            elif key.char == 's':
                msg.force.x = 0.0
            elif key.char == 'a':
                msg.force.y = 0.0
            elif key.char == 'd':
                msg.force.y = 0.0
            elif key.char == 'q':
                msg.force.z = 0.0
            elif key.char == 'z':
                msg.force.z = 0.0
        self.force_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    keyboard_teleop = KeyboardTeleop()
    try:
        rclpy.spin(keyboard_teleop)
    except KeyboardInterrupt:
        pass # To force exit code 0
    rclpy.shutdown()


if __name__ == '__main__':
    main()
