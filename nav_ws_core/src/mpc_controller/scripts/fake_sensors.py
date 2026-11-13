#!/usr/bin/env python3
"""D3 smoke 用假传感器：静态 odom->base TF、恒零 /odom、全 inf /scan。"""

import math
import time

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster


class FakeSensors(Node):
    def __init__(self):
        super().__init__("fake_sensors")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_footprint")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("base_yaw_deg", 0.0)
        self.declare_parameter("map_x", 0.0)
        self.declare_parameter("map_y", 0.0)
        self.declare_parameter("map_yaw_deg", 0.0)
        self.declare_parameter("publish_map_odom", True)
        self.declare_parameter("map_odom_dynamic_seconds", 0.0)
        self.declare_parameter("odom_rate_hz", 50.0)
        self.declare_parameter("scan_rate_hz", 10.0)

        self.odom_frame = self.get_parameter("odom_frame").value
        self.base_frame = self.get_parameter("base_frame").value
        self.map_frame = self.get_parameter("map_frame").value
        self.yaw = math.radians(self.get_parameter("base_yaw_deg").value)
        self.map_x = float(self.get_parameter("map_x").value)
        self.map_y = float(self.get_parameter("map_y").value)
        self.map_yaw = math.radians(self.get_parameter("map_yaw_deg").value)
        self.publish_map_odom = bool(self.get_parameter("publish_map_odom").value)
        self.map_odom_dynamic_seconds = float(
            self.get_parameter("map_odom_dynamic_seconds").value)
        self.map_odom_start = time.monotonic()
        self.map_odom_dynamic = (
            self.publish_map_odom and self.map_odom_dynamic_seconds > 0.0)
        self.qz = math.sin(self.yaw / 2.0)
        self.qw = math.cos(self.yaw / 2.0)

        self.static_broadcaster = StaticTransformBroadcaster(self)
        self.dynamic_broadcaster = TransformBroadcaster(self) if self.map_odom_dynamic else None
        self.publish_static_tf()
        if self.map_odom_dynamic:
            self.create_timer(0.05, self.publish_dynamic_map_odom)

        self.odom_pub = self.create_publisher(Odometry, "odom", 10)
        self.scan_pub = self.create_publisher(LaserScan, "scan", 10)
        self.create_timer(1.0 / float(self.get_parameter("odom_rate_hz").value),
                          self.publish_odom)
        self.create_timer(1.0 / float(self.get_parameter("scan_rate_hz").value),
                          self.publish_scan)
        self.get_logger().info(
            f"fake sensors: {self.odom_frame}->{self.base_frame} yaw={self.yaw:.3f} rad; "
            f"map_odom={self.publish_map_odom} dynamic={self.map_odom_dynamic} "
            f"({self.map_x:.2f}, {self.map_y:.2f}, {self.map_yaw:.3f} rad)")

    def publish_static_tf(self):
        odom_base = TransformStamped()
        odom_base.header.stamp = self.get_clock().now().to_msg()
        odom_base.header.frame_id = self.odom_frame
        odom_base.child_frame_id = self.base_frame
        odom_base.transform.rotation.z = self.qz
        odom_base.transform.rotation.w = self.qw

        transforms = [odom_base]
        if self.publish_map_odom and not self.map_odom_dynamic:
            transforms.append(self.make_map_odom(odom_base.header.stamp))
        # StaticTransformBroadcaster latches depth=1: send both in one message,
        # otherwise the second call overwrites the first on /tf_static.
        self.static_broadcaster.sendTransform(transforms)

    def make_map_odom(self, stamp):
        map_odom = TransformStamped()
        map_odom.header.stamp = stamp
        map_odom.header.frame_id = self.map_frame
        map_odom.child_frame_id = self.odom_frame
        map_odom.transform.translation.x = self.map_x
        map_odom.transform.translation.y = self.map_y
        map_odom.transform.rotation.z = math.sin(self.map_yaw / 2.0)
        map_odom.transform.rotation.w = math.cos(self.map_yaw / 2.0)
        return map_odom

    def publish_dynamic_map_odom(self):
        if time.monotonic() - self.map_odom_start > self.map_odom_dynamic_seconds:
            return
        stamp = self.get_clock().now().to_msg()
        self.dynamic_broadcaster.sendTransform(self.make_map_odom(stamp))

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
