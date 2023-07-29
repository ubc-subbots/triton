from codecs import EncodedFile
import rclpy
from rclpy.node import Node
from pynput import keyboard

from std_msgs.msg import String, Int32, UInt32

THRUSTER_DATA_BIT_SIZE = 5

T1_SHIFT_VAL = 0*THRUSTER_DATA_BIT_SIZE
T2_SHIFT_VAL = 1*THRUSTER_DATA_BIT_SIZE
T3_SHIFT_VAL = 2*THRUSTER_DATA_BIT_SIZE
T4_SHIFT_VAL = 3*THRUSTER_DATA_BIT_SIZE
T5_SHIFT_VAL = 4*THRUSTER_DATA_BIT_SIZE
T6_SHIFT_VAL = 5*THRUSTER_DATA_BIT_SIZE

ENCODE_OFFSET = 2**(THRUSTER_DATA_BIT_SIZE-1)

CHAR_TO_THRUSTER = {
    # key: [(thruster, level change)+]
    "q": [('1', 1), ('2', 1)], # up
    "e": [('1', -1), ('2', -1)], # down
    "w": [('3', 1), ('4', 1)],
    "s": [('3', -1), ('4', -1)],
    "a": [('5', -1)],
    "d": [('5', 1)],

    "r": [('1', 1)],
    "f": [('1', -1)],
    "t": [('2', 1)],
    "g": [('2', -1)],
    "y": [('3', 1)],
    "h": [('3', -1)],
    "u": [('4', 1)],
    "j": [('4', -1)],
    "i": [('5', 1)],
    "k": [('5', -1)],
}

def encode_msg(thruster_power_level):
    t6_bits = (thruster_power_level["x"]+ENCODE_OFFSET) << T6_SHIFT_VAL
    t5_bits = (thruster_power_level["5"]+ENCODE_OFFSET) << T5_SHIFT_VAL # M
    t4_bits = (thruster_power_level["4"]+ENCODE_OFFSET) << T4_SHIFT_VAL # F
    t3_bits = (thruster_power_level["3"]+ENCODE_OFFSET) << T3_SHIFT_VAL # FL
    t2_bits = (thruster_power_level["2"]+ENCODE_OFFSET) << T2_SHIFT_VAL # |
    t1_bits = (thruster_power_level["1"]+ENCODE_OFFSET) << T1_SHIFT_VAL # |L

    return (0b10101010000000000000000000000000 | t5_bits | t4_bits | t3_bits | t2_bits | t1_bits)


class KeyPublisher(Node):

    def __init__(self):
        super().__init__('key_publisher')
        self.publisher_ = self.create_publisher(
            UInt32,
            #'/triton/controls/key',
            '/motor_control', # What the teensy expects
            10
        )
        self._start()

        # States
        self.thruster_power_level = {}
        self.thruster_power_level["1"] = 0
        self.thruster_power_level["2"] = 0
        self.thruster_power_level["3"] = 0
        self.thruster_power_level["4"] = 0
        self.thruster_power_level["5"] = 0
        self.thruster_power_level["x"] = 0

        self.get_logger().info('Key publisher succesfully started!')


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
        if key == keyboard.Key.space:
            self.thruster_power_level["1"] = 0
            self.thruster_power_level["2"] = 0
            self.thruster_power_level["3"] = 0
            self.thruster_power_level["4"] = 0
            self.thruster_power_level["5"] = 0
            self.thruster_power_level["x"] = 0
        elif 'char' in dir(key):
            if key.char in CHAR_TO_THRUSTER:
                for thruster, level in CHAR_TO_THRUSTER[key.char]:
                    if ((self.thruster_power_level[thruster] < 15 and level > 0) or
                        (self.thruster_power_level[thruster] > -16 and level < 0)):
                        self.thruster_power_level[thruster] += level
        msg = UInt32()
        msg.data = encode_msg(self.thruster_power_level)
        self.publisher_.publish(msg)

    def _on_release(self, key):
        """
        Handles key releases
        @param key: They character of the key released
        """
        msg = UInt32()
        msg.data = encode_msg(self.thruster_power_level)
        self.publisher_.publish(msg)



def main(args=None):
    rclpy.init(args=args)

    key_publisher = KeyPublisher()

    try:
        rclpy.spin(key_publisher)
    except KeyboardInterrupt:
        pass

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    key_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()