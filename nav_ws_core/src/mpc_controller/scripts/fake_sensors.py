#!/usr/bin/env python3
"""D3 smoke 用假传感器：静态 odom->base TF、恒零 /odom、全 inf /scan。"""

import math

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from tf2_ros import StaticTransformBroadcaster


class FakeSensors(Node):
    def __init__(self):
        super().__init__("fake_sensors")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_footprint")
        self.declare_parameter("base_yaw_deg", 0.0)
        self.declare_parameter("odom_rate_hz", 50.0)
        self.declare_parameter("scan_rate_hz", 10.0)

        self.odom_frame = self.get_parameter("odom_frame").value
        self.base_frame = self.get_parameter("base_frame").value
        self.yaw = math.radians(self.get_parameter("base_yaw_deg").value)
        self.qz = math.sin(self.yaw / 2.0)
        self.qw = math.cos(self.yaw / 2.0)

        self.static_broadcaster = StaticTransformBroadcaster(self)
        self.publish_static_tf()

        self.odom_pub = self.create_publisher(Odometry, "odom", 10)
        self.scan_pub = self.create_publisher(LaserScan, "scan", 10)
        self.create_timer(1.0 / float(self.get_parameter("odom_rate_hz").value),
                          self.publish_odom)
        self.create_timer(1.0 / float(self.get_parameter("scan_rate_hz").value),
                          self.publish_scan)
        self.get_logger().info(
            f"fake sensors: {self.odom_frame}->{self.base_frame} yaw={self.yaw:.3f} rad")

    def publish_static_tf(self):
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = self.odom_frame
        transform.child_frame_id = self.base_frame
        transform.transform.rotation.z = self.qz
        transform.transform.rotation.w = self.qw
        self.static_broadcaster.sendTransform(transform)

    def publish_odom(self):
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.odom_frame
        msg.child_frame_id = self.base_frame
        msg.pose.pose.orientation.z = self.qz
        msg.pose.pose.orientation.w = self.qw
        self.odom_pub.publish(msg)

    def publish_scan(self):
        msg = LaserScan()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.base_frame
        msg.angle_min = -math.pi
        msg.angle_max = math.pi
        msg.angle_increment = math.pi / 180.0
        msg.range_min = 0.05
        msg.range_max = 10.0
        msg.ranges = [float("inf")] * 360
        self.scan_pub.publish(msg)


def main():
    rclpy.init()
    node = FakeSensors()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
