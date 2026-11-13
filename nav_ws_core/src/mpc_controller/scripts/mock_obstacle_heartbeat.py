#!/usr/bin/env python3
"""D3 smoke 用独立 mock：10 Hz 发布有效空 ObstacleStateArray，可由服务暂停/恢复。"""

import rclpy
from mpc_controller.msg import ObstacleStateArray
from rclpy.node import Node
from std_srvs.srv import SetBool


class MockObstacleHeartbeat(Node):
    def __init__(self):
        super().__init__("mock_obstacle_heartbeat")
        self.declare_parameter("topic", "/mpc/obstacles")
        self.declare_parameter("rate_hz", 10.0)
        self.declare_parameter("enabled", False)

        self.topic = self.get_parameter("topic").value
        self.enabled = bool(self.get_parameter("enabled").value)
        self.publisher = self.create_publisher(ObstacleStateArray, self.topic, 10)
        self.create_service(
            SetBool, "/mock_obstacle_heartbeat/set_enabled", self.set_enabled)
        self.create_timer(1.0 / float(self.get_parameter("rate_hz").value), self.publish)
        self.get_logger().info(
            f"mock heartbeat on {self.topic} at {self.get_parameter('rate_hz').value} Hz; "
            f"enabled={self.enabled}")

    def set_enabled(self, request, response):
        self.enabled = bool(request.data)
        response.success = True
        response.message = f"heartbeat enabled={self.enabled}"
        self.get_logger().info(response.message)
        return response

    def publish(self):
        if not self.enabled:
            return
        msg = ObstacleStateArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "odom"
        msg.obstacles = []
        self.publisher.publish(msg)


def main():
    rclpy.init()
    node = MockObstacleHeartbeat()
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
