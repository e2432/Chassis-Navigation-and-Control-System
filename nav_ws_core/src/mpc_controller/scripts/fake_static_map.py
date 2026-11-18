#!/usr/bin/env python3
"""TEST ONLY: publish a small static OccupancyGrid on /map.

Used by the smoke launch to exercise the plugin's static distance field
builder. The default wall is at y=2.0 m, off the straight 2 m test path, so
the field is built and used without changing the expected motion.
"""

from __future__ import annotations

import rclpy
from geometry_msgs.msg import Pose
from nav_msgs.msg import MapMetaData, OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


class FakeStaticMap(Node):
    def __init__(self):
        super().__init__("fake_static_map")
        self.declare_parameter("width", 40)
        self.declare_parameter("height", 40)
        self.declare_parameter("resolution", 0.1)
        self.declare_parameter("origin_x", 0.0)
        self.declare_parameter("origin_y", 0.0)
        self.declare_parameter("wall_y", 2.0)
        self.declare_parameter("rate_hz", 1.0)

        width = int(self.get_parameter("width").value)
        height = int(self.get_parameter("height").value)
        resolution = float(self.get_parameter("resolution").value)
        origin_x = float(self.get_parameter("origin_x").value)
        origin_y = float(self.get_parameter("origin_y").value)
        wall_y = float(self.get_parameter("wall_y").value)
        rate_hz = float(self.get_parameter("rate_hz").value)

        message = OccupancyGrid()
        message.header.frame_id = "map"
        message.info = MapMetaData()
        message.info.resolution = resolution
        message.info.width = width
        message.info.height = height
        message.info.origin = Pose()
        message.info.origin.position.x = origin_x
        message.info.origin.position.y = origin_y
        message.info.origin.orientation.w = 1.0
        message.data = [0] * (width * height)
        for y in range(height):
            world_y = origin_y + (y + 0.5) * resolution
            if abs(world_y - wall_y) <= resolution * 0.5:
                for x in range(width):
                    message.data[y * width + x] = 100
        self.message = message

        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.publisher = self.create_publisher(OccupancyGrid, "/map", qos)
        self.timer = self.create_timer(1.0 / max(rate_hz, 0.2), self.publish)
        self.get_logger().warn(
            "TEST ONLY: static map %dx%d res=%.2f wall_y=%.2f" %
            (width, height, resolution, wall_y))

    def publish(self):
        self.message.header.stamp = self.get_clock().now().to_msg()
        self.publisher.publish(self.message)


def main():
    rclpy.init()
    node = FakeStaticMap()
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
