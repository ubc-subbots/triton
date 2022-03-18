from codecs import EncodedFile
import rclpy
from rclpy.node import Node
from pynput import keyboard

from std_msgs.msg import String, Int32

THRUSTER_DATA_BIT_SIZE = 5

TLF_SHIFT_VAL = 0*THRUSTER_DATA_BIT_SIZE
TLT_SHIFT_VAL = 1*THRUSTER_DATA_BIT_SIZE
TLB_SHIFT_VAL = 2*THRUSTER_DATA_BIT_SIZE
TRF_SHIFT_VAL = 3*THRUSTER_DATA_BIT_SIZE
TRT_SHIFT_VAL = 4*THRUSTER_DATA_BIT_SIZE
TRB_SHIFT_VAL = 5*THRUSTER_DATA_BIT_SIZE

ENCODE_OFFSET = 2**(THRUSTER_DATA_BIT_SIZE-1)

def encode_msg(thruster_power_level):
    tlf_bits = (thruster_power_level["tlf"]+ENCODE_OFFSET) << TLF_SHIFT_VAL
    tlt_bits = (thruster_power_level["tlt"]+ENCODE_OFFSET) << TLT_SHIFT_VAL
    tlb_bits = (thruster_power_level["tlb"]+ENCODE_OFFSET) << TLB_SHIFT_VAL
    trf_bits = (thruster_power_level["trf"]+ENCODE_OFFSET) << TRF_SHIFT_VAL
    trt_bits = (thruster_power_level["trt"]+ENCODE_OFFSET) << TRT_SHIFT_VAL
    trb_bits = (thruster_power_level["trb"]+ENCODE_OFFSET) << TRB_SHIFT_VAL

    return (tlf_bits | tlt_bits | tlb_bits | trf_bits | trt_bits | trb_bits)


class KeyPublisher(Node):

    def __init__(self):
        super().__init__('key_publisher')
        self.publisher_ = self.create_publisher(
            Int32,
            '/triton/controls/key',
            10
        )
        self._start()

        # States
        self.thruster_power_level = {}
        self.thruster_power_level["tlf"] = 0
        self.thruster_power_level["tlt"] = 0
        self.thruster_power_level["tlb"] = 0
        self.thruster_power_level["trf"] = 0
        self.thruster_power_level["trt"] = 0
        self.thruster_power_level["trb"] = 0

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
            self.thruster_power_level["tlf"] = 0
            self.thruster_power_level["tlt"] = 0
            self.thruster_power_level["tlb"] = 0
            self.thruster_power_level["trf"] = 0
            self.thruster_power_level["trt"] = 0
            self.thruster_power_level["trb"] = 0
        elif 'char' in dir(key):
            if key.char == 'r':
                if self.thruster_power_level["tlt"] != 15:
                    self.thruster_power_level["tlt"] += 1
            elif key.char == 't':
                if self.thruster_power_level["tlt"] != -16:
                    self.thruster_power_level["tlt"] -= 1
            elif key.char == 'f':
                if self.thruster_power_level["tlf"] != 15:
                    self.thruster_power_level["tlf"] += 1
            elif key.char == 'g':
                if self.thruster_power_level["tlf"] != -16:
                    self.thruster_power_level["tlf"] -= 1
            elif key.char == 'v':
                if self.thruster_power_level["tlb"] != 15:
                    self.thruster_power_level["tlb"] += 1
            elif key.char == 'b':
                if self.thruster_power_level["tlb"] != -16:
                    self.thruster_power_level["tlb"] -= 1
            elif key.char == 'y':
                if self.thruster_power_level["trt"] != 15:
                    self.thruster_power_level["trt"] += 1
            elif key.char == 'u':
                if self.thruster_power_level["trt"] != -16:
                    self.thruster_power_level["trt"] -= 1
            elif key.char == 'h':
                if self.thruster_power_level["trf"] != 15:
                    self.thruster_power_level["trf"] += 1
            elif key.char == 'j':
                if self.thruster_power_level["trf"] != -16:
                    self.thruster_power_level["trf"] -= 1
            elif key.char == 'n':
                if self.thruster_power_level["trb"] != 15:
                    self.thruster_power_level["trb"] += 1
            elif key.char == 'm':
                if self.thruster_power_level["trb"] != -16:
                    self.thruster_power_level["trb"] -= 1

        msg = Int32()
        msg.data = encode_msg(self.thruster_power_level)
        self.publisher_.publish(msg)

    def _on_release(self, key):
        """
        Handles key releases
        @param key: They character of the key released
        """
        msg = Int32()
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