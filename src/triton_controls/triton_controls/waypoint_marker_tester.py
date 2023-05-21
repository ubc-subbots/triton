#! /usr/bin/env python3
import rclpy
from rclpy.node import Node
from triton_interfaces.msg import Waypoint
from std_msgs.msg import String
import math

def quaternion_from_euler(roll, pitch, yaw):
    # https://gist.github.com/salmagro/2e698ad4fbf9dae40244769c5ab74434
    """
    Converts euler roll, pitch, yaw to quaternion (w in last place)
    quat = [w, x, y, z]
    Bellow should be replaced when porting for ROS 2 Python tf_conversions is done.
    """
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    q = [0] * 4
    q[0] = cy * cp * cr + sy * sp * sr
    q[1] = cy * cp * sr - sy * sp * cr
    q[2] = sy * cp * sr + cy * sp * cr
    q[3] = sy * cp * cr - cy * sp * sr

    return q

STABILIZE = 0
PASSTHROUGH = 1

target_poses = [
    {
        'pose': {
            'position': {
                'x': 0.0,
                'y': 0.0,
                'z': 3.0
            },
            'orientation_rpy': {
                'r': 0,
                'p':0,
                'y': -0.7
            },
            'orientation': {
                'x': 0.0,
                'y': 0.0,
                'z': 1.0,
                'w': -0.7,
            }
        },
        'distance': {
            'position': {
                'x': 0.5,
                'y': 0.5,
                'z': 0.5
            },
            'orientation_rpy': {
                'r': 0,
                'p':0,
                'y': 0.1
            },
            'orientation': {
                'x': 0.1,
                'y': 0.1,
                'z': 0.05,
                'w': 0.05,
            }
        },
        'type': STABILIZE,
        'duration': 5.0
    },
    # Move forward by 1 unit in x
    {
        'pose': {
            'position': {
                'x': 5.0,
                'y': 0.0,
                'z': 5.0
            },
            'orientation_rpy': {
                'r': 0,
                'p':0,
                'y': 0
            },
            'orientation': {
                'x': 0.0,
                'y': 0.0,
                'z': 0.0,
                'w': 1.0,
            }
        },
        'distance': {
            'position': {
                'x': 0.5,
                'y': 10.0,
                'z': 0.5
            },
            'orientation_rpy': {
                'r': 181,
                'p':181,
                'y': 181
            },
            'orientation': {
                'x': 100.0,
                'y': 100.0,
                'z': 100.0,
                'w': 100.0,
            }
        },
        'type': STABILIZE,
        'duration': 5.0
    } 
]


class WaypointMarkerTester(Node):

    def __init__(self):
        super().__init__('waypoint_marker_tester')

        self.publisher = self.create_publisher(
            Waypoint,
            '/triton/controls/waypoint_marker/set',
            10
        )

        # Start a particular target (by index)
        self.subscription = self.create_subscription(
            String,
            '/triton/controls/waypoint_marker_tester/start',
            self.start_callback,
            1
        )

        self.subscription = self.create_subscription(
            Waypoint,
            '/triton/controls/waypoint_marker/current_goal',
            self.waypoint_callback,
            10
        )

        # calculate orientations in quaternion
        for tp in target_poses:
            pose_q = quaternion_from_euler(tp['pose']['orientation_rpy']['r'], tp['pose']['orientation_rpy']['p'], tp['pose']['orientation_rpy']['y'])
            dist_q = quaternion_from_euler(tp['distance']['orientation_rpy']['r'], tp['distance']['orientation_rpy']['p'], tp['distance']['orientation_rpy']['y'])
            tp['pose']['orientation']['x'] = pose_q[1]
            tp['pose']['orientation']['y'] = pose_q[2]
            tp['pose']['orientation']['z'] = pose_q[3]
            tp['pose']['orientation']['w'] = pose_q[0]
            tp['distance']['orientation']['x'] = abs(dist_q[1])
            tp['distance']['orientation']['y'] = abs(dist_q[2])
            tp['distance']['orientation']['z'] = abs(dist_q[3])
            tp['distance']['orientation']['w'] = abs(dist_q[0])
            self.get_logger().info(str(pose_q))
            self.get_logger().info(str(dist_q))

        self.get_logger().info("WaypointMarkerTester successfully started!")


    def start_callback(self, data):

        target_index = int(data.data)
        if target_index > len(target_poses):
            self.get_logger().info("Target index too large! ")
            return


        wp = Waypoint()
        wp.pose.position.x = target_poses[target_index]['pose']['position']['x']
        wp.pose.position.y = target_poses[target_index]['pose']['position']['y']
        wp.pose.position.z = target_poses[target_index]['pose']['position']['z']
        wp.pose.orientation.x = target_poses[target_index]['pose']['orientation']['x']
        wp.pose.orientation.y = target_poses[target_index]['pose']['orientation']['y']
        wp.pose.orientation.z = target_poses[target_index]['pose']['orientation']['z']
        wp.pose.orientation.w = target_poses[target_index]['pose']['orientation']['w']
        wp.distance.position.x = target_poses[target_index]['distance']['position']['x']
        wp.distance.position.y = target_poses[target_index]['distance']['position']['y']
        wp.distance.position.z = target_poses[target_index]['distance']['position']['z']
        wp.distance.orientation.x = target_poses[target_index]['distance']['orientation']['x']
        wp.distance.orientation.y = target_poses[target_index]['distance']['orientation']['y']
        wp.distance.orientation.z = target_poses[target_index]['distance']['orientation']['z']
        wp.distance.orientation.w = target_poses[target_index]['distance']['orientation']['w']

        wp.success = False
        wp.type = target_poses[target_index]['type']
        wp.duration = target_poses[target_index]['duration']

        self.publisher.publish(wp)

        self.get_logger().info("WaypointMarkerTester published a waypoint!")


    def waypoint_callback(self, data):
        return



def main(args=None):
    rclpy.init(args=args)
    node = WaypointMarkerTester()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


if __name__ == '__main__':
    main()
