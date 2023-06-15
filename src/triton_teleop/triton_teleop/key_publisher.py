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
    "q": [('bl', -1), ('br', 1)],
    "e": [('bl', 1), ('br', -1)],
    "x": [('tl', -1), ('tr', -1)],
    "c": [('tl', 1), ('tr', 1)],
    "w": [('bl', 1), ('br', 1)],
    "s": [('bl', -1), ('br', -1)],
    "a": [('m', -1)],
    "d": [('m', 1)],

    "r": [('tl', 1)],
    "f": [('tl', -1)],
    "t": [('bl', 1)],
    "g": [('bl', -1)],
    "y": [('m', 1)],
    "h": [('m', -1)],
    "u": [('br', 1)],
    "j": [('br', -1)],
    "i": [('bl', 1)],
    "k": [('bl', -1)],
}

def encode_msg(thruster_power_level):
    t6_bits = (thruster_power_level["x"]+ENCODE_OFFSET) << T6_SHIFT_VAL
    t5_bits = (thruster_power_level["bl"]+ENCODE_OFFSET) << T5_SHIFT_VAL
    t4_bits = (thruster_power_level["m"]+ENCODE_OFFSET) << T4_SHIFT_VAL
    t3_bits = (thruster_power_level["tl"]+ENCODE_OFFSET) << T3_SHIFT_VAL
    t2_bits = (thruster_power_level["br"]+ENCODE_OFFSET) << T2_SHIFT_VAL
    t1_bits = (thruster_power_level["tr"]+ENCODE_OFFSET) << T1_SHIFT_VAL

    return (t6_bits | t5_bits | t4_bits | t3_bits | t2_bits | t1_bits)


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
        self.thruster_power_level["tl"] = 0
        self.thruster_power_level["tr"] = 0
        self.thruster_power_level["bl"] = 0
        self.thruster_power_level["br"] = 0
        self.thruster_power_level["m"] = 0
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
            self.thruster_power_level["tl"] = 0
            self.thruster_power_level["tr"] = 0
            self.thruster_power_level["bl"] = 0
            self.thruster_power_level["br"] = 0
            self.thruster_power_level["m"] = 0
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