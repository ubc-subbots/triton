#! /usr/bin/env python3
import rclpy
from rclpy.node import Node
from triton_interfaces.msg import Waypoint
from std_msgs.msg import String

STABILIZE = 0
PASSTHROUGH = 1

target_poses = [
    # Move forward by 1 unit in x
    {
        'pose': {
            'position': {
                'x': 1.0,
                'y': 0.0,
                'z': 0.0
            },
            'orientation': {
                'x': 0.0,
                'y': 0.0,
                'z': 5.0,
                'w': 1.0,
            }
        },
        'distance': {
            'position': {
                'x': 0.5,
                'y': 10.0,
                'z': 10.0
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

        self.step = 0

        self.targets_achieved = [0]

        self.publisher = self.create_publisher(
            Waypoint,
            '/triton/controls/waypoint_marker/set',
            10
        )

        # Anything value starts a new waypoint
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

        self.get_logger().info("WaypointMarkerTester successfully started!")

    def start_callback(self, data):

        if self.step >= len(target_poses):
            self.get_logger().info("All target waypoints have already been published!")
            return

        wp = Waypoint()
        wp.pose.position.x = target_poses[self.step]['pose']['position']['x']
        wp.pose.position.y = target_poses[self.step]['pose']['position']['y']
        wp.pose.position.z = target_poses[self.step]['pose']['position']['z']
        wp.pose.orientation.x = target_poses[self.step]['pose']['orientation']['x']
        wp.pose.orientation.y = target_poses[self.step]['pose']['orientation']['y']
        wp.pose.orientation.z = target_poses[self.step]['pose']['orientation']['z']
        wp.pose.orientation.w = target_poses[self.step]['pose']['orientation']['w']
        wp.distance.position.x = target_poses[self.step]['distance']['position']['x']
        wp.distance.position.y = target_poses[self.step]['distance']['position']['y']
        wp.distance.position.z = target_poses[self.step]['distance']['position']['z']
        wp.distance.orientation.x = target_poses[self.step]['distance']['orientation']['x']
        wp.distance.orientation.y = target_poses[self.step]['distance']['orientation']['y']
        wp.distance.orientation.z = target_poses[self.step]['distance']['orientation']['z']
        wp.distance.orientation.w = target_poses[self.step]['distance']['orientation']['w']

        wp.success = False
        wp.type = target_poses[self.step]['type']
        wp.duration = target_poses[self.step]['duration']

        self.publisher.publish(wp)

        self.get_logger().info("WaypointMarkerTester published a waypoint!")

        if self.step < len(target_poses):
            self.step = self.step + 1

    def waypoint_callback(self, data):
        if data.success == True:
            if self.targets_achieved[self.step - 1] == 0: # To prevent the same success message from repeating
                self.get_logger().info("WAYPOINT ACHIEVED!")
                self.targets_achieved[self.step - 1] = 1



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
