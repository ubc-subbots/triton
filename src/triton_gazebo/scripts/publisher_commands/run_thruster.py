import os
import time

thruster_values = [0.001, 0.002, 0.003, 0.004, 0.005, 0.006]

def main():
    print('Thruster force commands will start shortly')
    time.sleep(0.5)
    move_cw(1)
    stop()
    time.sleep(0.7)
    move_ccw(1)
    time.sleep(1)
    move_up(0.4)
    stop()


def thruster_force(one, two, three, four, five, six):
    one = str(one)
    two = str(two)
    three = str(three)
    four = str(four)
    five = str(five)
    six = str(six)
    vector = "{" + one + "," + two + "," + three + "," + four + "," + five + "," + six + "}"
    os.system(
        'ros2 topic pub /triton_gazebo/thruster_vector std_msgs/Float64MultiArray "data: ' + vector + '" -1')


def move_up(time_sec):
    thruster_force(0.001, 0.002, 0.003, 0.004, 120.05, 120.1)
    time.sleep(time_sec)
    stay_afloat()


def stay_afloat():
    thruster_force(0.001, 0.002, 0.003, 0.004, 90.05, 90.1)


def move_cw(time_sec):
    thruster_force(76.501, 0.00023134, 76.501, 0.0002414, 0.00034151, 0.000918)
    time.sleep(time_sec)
    stop()


def move_ccw(time_sec):
    thruster_force(0.001, 75.001, 0.003, 75.002, 0.000321, 0.00002314)
    time.sleep(time_sec)
    stop()


def move_up_then_spin():
    move_up(3)
    move_ccw(5)


def stop():
    thruster_force(0.0001, 0.0002, 0.0003, 0.0004, 0.0005, 0.0006)


if __name__ == "__main__":
    main()
