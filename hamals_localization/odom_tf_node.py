#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


class OdomTFNode(Node):
    """
    odom_tf_node
    ------------
    Responsibility:
      - Subscribe to odom_topic (nav_msgs/Odometry)
      - Publish TF: parent_frame -> child_frame

    Does NOT:
      - Compute odometry
      - Filter data
      - Modify pose
      - Publish /odom
    """

    def __init__(self):
        super().__init__('odom_tf_node')

        # -----------------------------
        # Parameters (configurable)
        # -----------------------------
        self.declare_parameter('odom_topic', '/odom_raw')
        self.declare_parameter('parent_frame', 'odom')
        self.declare_parameter('child_frame', 'base_link')

        self.odom_topic = (
            self.get_parameter('odom_topic')
            .get_parameter_value()
            .string_value
        )
        self.parent_frame = (
            self.get_parameter('parent_frame')
            .get_parameter_value()
            .string_value
        )
        self.child_frame = (
            self.get_parameter('child_frame')
            .get_parameter_value()
            .string_value
        )

        # -----------------------------
        # TF broadcaster
        # -----------------------------
        self.tf_broadcaster = TransformBroadcaster(self)

        # -----------------------------
        # Subscription
        # -----------------------------
        self.sub = self.create_subscription(
            Odometry,
            self.odom_topic,
            self.odom_callback,
            10
        )

        self.get_logger().info(
            f'odom_tf_node started | '
            f'subscribing: {self.odom_topic} | '
            f'TF: {self.parent_frame} -> {self.child_frame}'
        )

    def odom_callback(self, msg: Odometry):
        t = TransformStamped()

        t.header.stamp = msg.header.stamp

        t.header.frame_id = self.parent_frame
        t.child_frame_id = self.child_frame

        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = 0.0

        t.transform.rotation = msg.pose.pose.orientation

        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)

    node = OdomTFNode()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()