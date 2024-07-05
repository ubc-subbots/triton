#! /usr/bin/env python3
from math import sin, cos, pi, atan, asin, atan2
import threading
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Quaternion, PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Path
from tf2_ros import TransformBroadcaster, TransformStamped


class AUVTransformPublisher(Node):

    def __init__(self):
        super().__init__('auv_transform_publisher')

        qos_profile = QoSProfile(depth=10)
        self.state_subscriber = self.create_subscription(
            #PoseStamped,
            PoseWithCovarianceStamped,
            '/triton/state',
            self.state_callback,
            10

        )
        self.path_publisher = self.create_publisher(
            Path,
            '/triton/path',
            qos_profile
        )
        self.pose_array = []
        self.broadcaster = TransformBroadcaster(self, qos=qos_profile)
        self.get_logger().info("AUVTransformPublisher successfully started!")

    def state_callback(self, msg):
        msg_wo_cov = PoseStamped()
        msg_wo_cov.header = msg.header
        msg_wo_cov.pose = msg.pose.pose
        self.pose_array.append(msg_wo_cov)
        now = self.get_clock().now()
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        odom_trans = TransformStamped()
        odom_trans.header.frame_id = msg.header.frame_id
        odom_trans.child_frame_id = 'base_link'
        odom_trans.header.stamp = now.to_msg()
        odom_trans.transform.translation.x = p.x
        odom_trans.transform.translation.y = p.y
        odom_trans.transform.translation.z = p.z
        odom_trans.transform.rotation = q
        self.broadcaster.sendTransform(odom_trans)
        path_msg = Path()
        path_msg.header.frame_id = msg.header.frame_id
        path_msg.header.stamp = now.to_msg()
        path_msg.poses = self.pose_array
        self.path_publisher.publish(path_msg)

def main(args=None):
    rclpy.init(args=args)
    node = AUVTransformPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


if __name__ == '__main__':
    main()
