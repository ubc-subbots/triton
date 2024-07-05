#! /usr/bin/env python3
import rclpy
from rclpy.node import Node
from triton_interfaces.msg import Waypoint
from std_msgs.msg import String
from std_msgs.msg import String, Int32, UInt32

THRUSTER_DATA_BIT_SIZE = 5

T1_SHIFT_VAL = 0*THRUSTER_DATA_BIT_SIZE
T2_SHIFT_VAL = 1*THRUSTER_DATA_BIT_SIZE
T3_SHIFT_VAL = 2*THRUSTER_DATA_BIT_SIZE
T4_SHIFT_VAL = 3*THRUSTER_DATA_BIT_SIZE
T5_SHIFT_VAL = 4*THRUSTER_DATA_BIT_SIZE
T6_SHIFT_VAL = 5*THRUSTER_DATA_BIT_SIZE

ENCODE_OFFSET = 2**(THRUSTER_DATA_BIT_SIZE-1)

def encode_msg(thruster_power_level):
    # t6_bits = (thruster_power_level["6"]+ENCODE_OFFSET) << T6_SHIFT_VAL
    t6_bits = (0+ENCODE_OFFSET) << T6_SHIFT_VAL
    t5_bits = (thruster_power_level["5"]+ENCODE_OFFSET) << T5_SHIFT_VAL
    t4_bits = (thruster_power_level["4"]+ENCODE_OFFSET) << T4_SHIFT_VAL
    t3_bits = (thruster_power_level["3"]+ENCODE_OFFSET) << T3_SHIFT_VAL
    t2_bits = (thruster_power_level["2"]+ENCODE_OFFSET) << T2_SHIFT_VAL
    t1_bits = (thruster_power_level["1"]+ENCODE_OFFSET) << T1_SHIFT_VAL

    return (t6_bits | t5_bits | t4_bits | t3_bits | t2_bits | t1_bits)

DEFAULT_EFFORT = 15 #vroom

# from -16 to 15 inclusive
# 0 is stationary
# 15 is max forward
# TODO: mapping
STATIONARY = {
    "1": 0,
    "2": 0,
    "3": 0,
    "4": 0,
    "5": 0,
}
FORWARD = {
    "3": DEFAULT_EFFORT,
    "4": DEFAULT_EFFORT,
    "1": 0,
    "2": 0,
    "5": 0,
}
BACKWARD = {
    "3": -DEFAULT_EFFORT,
    "4": -DEFAULT_EFFORT,
    "1": 0,
    "2": 0,
    "5": 0,
}
RIGHT_TURN = {
    "3": DEFAULT_EFFORT,
    "4": -DEFAULT_EFFORT,
    "1": 0,
    "2": 0,
    "5": 0,
}
LEFT_TURN = {
    "3": -DEFAULT_EFFORT,
    "4": DEFAULT_EFFORT,
    "1": 0,
    "2": 0,
    "5": 0,
}
RIGHT_SWAY = {
    "1": 0,
    "2": 0,
    "3": 0,
    "4": 0,
    "5": DEFAULT_EFFORT,
}
LEFT_SWAY = {
    "1": 0,
    "2": 0,
    "3": 0,
    "4": 0,
    "5": -DEFAULT_EFFORT,
}
UPWARD = {
    "1": DEFAULT_EFFORT,
    "2": DEFAULT_EFFORT,
    "3": 0,
    "4": 0,
    "5": 0,
}
DOWNWARD = {
    "1": -DEFAULT_EFFORT,
    "2": -DEFAULT_EFFORT,
    "3": 0,
    "4": 0,
    "5": 0,
}

# Levels of each thruster
# duration is seconds
thrusts = [
    
    # we start by waiting 5 secs for the other nodes like the serial subscriber to be started
    {
        'levels': STATIONARY,
        'duration': 5,
    },
    # change the below
    {
        'levels': STATIONARY,
        'duration': 5,
    },
    {
        'levels': FORWARD,
        'duration': 5,
    },
    {
        'levels': BACKWARD,
        'duration': 5,
    },
    {
        'levels': RIGHT_SWAY,
        'duration': 3,
    },
    # they can be custom, too
    {
        'levels': {
            '1': 1,
            '2': 2,
            '3': 3,
            '4': 0,
            '5': -1
        },
        'duration': 3,
    },
]


class PredeterminedThrust(Node):

    def __init__(self):
        super().__init__('predetermined_thrust')

        self.current_index = 0
        self.last_time = self.get_clock().now()

        # check time every 0.1 second
        self.timer = self.create_timer(0.1, self.callback)

        self.publisher_ = self.create_publisher(
            UInt32,
            #'/triton/controls/key',
            '/motor_control', # What the teensy expects
            10
        )

        self.get_logger().info('Predetermined thrust node succesfully started!')

        msg = UInt32()
        msg.data = encode_msg(thrusts[self.current_index]['levels'])
        self.publisher_.publish(msg)


    def callback(self):

        if self.current_index + 1 >= len(thrusts):
            self.get_logger().info("All thrusts published reached!")
            return

        move_on = (self.get_clock().now() - self.last_time > rclpy.time.Duration(seconds=thrusts[self.current_index]['duration']))
        if move_on:
            self.current_index += 1
            msg = UInt32()
            msg.data = encode_msg(thrusts[self.current_index]['levels'])
            self.publisher_.publish(msg)
            self.last_time = self.get_clock().now()






def main(args=None):
    rclpy.init(args=args)
    node = PredeterminedThrust()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


if __name__ == '__main__':
    main()
