#!/usr/bin/env python3
"""D3 运行时验收：Active、非零 /cmd_vel、失联零速与恢复、限速、diff 转向。"""

import math
import sys
import threading
import time

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import FollowPath
from nav2_msgs.msg import SpeedLimit
from nav_msgs.msg import Path
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_srvs.srv import SetBool


class SmokeCheck(Node):
    def __init__(self):
        super().__init__("smoke_check")
        self.declare_parameter("chassis_type", "omni")
        self.chassis_type = self.get_parameter("chassis_type").value

        self.lock = threading.Lock()
        self.last_planar = None
        self.last_vx = 0.0
        self.last_vy = 0.0
        self.last_wz = 0.0
        self.cmd_count = 0

        self.create_subscription(Twist, "/cmd_vel", self.cmd_cb, 10)
        self.speed_limit_pub = self.create_publisher(SpeedLimit, "/speed_limit", 10)
        self.action_client = ActionClient(self, FollowPath, "follow_path")
        self.mock_client = self.create_client(
            SetBool, "/mock_obstacle_heartbeat/set_enabled")
        self.goal_handle = None

    def cmd_cb(self, msg):
        with self.lock:
            self.last_vx = msg.linear.x
            self.last_vy = msg.linear.y
            self.last_wz = msg.angular.z
            self.last_planar = math.hypot(msg.linear.x, msg.linear.y)
            self.cmd_count += 1

    def snapshot(self):
        with self.lock:
            return (self.last_planar, self.last_vx, self.last_vy, self.last_wz,
                    self.cmd_count)

    def wait_future(self, future, timeout):
        deadline = time.monotonic() + timeout
        while not future.done():
            if time.monotonic() > deadline:
                return None
            time.sleep(0.05)
        return future.result()

    def set_mock(self, enabled):
        request = SetBool.Request()
        request.data = enabled
        result = self.wait_future(self.mock_client.call_async(request), 5.0)
        if result is None or not result.success:
            raise RuntimeError(f"mock set_enabled({enabled}) failed")
        self.get_logger().info(f"mock heartbeat enabled={enabled}")

    def wait_nonzero(self, timeout=3.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            planar, _, _, wz, _ = self.snapshot()
            if (planar is not None and planar > 0.05) or abs(wz) > 0.1:
                return True
            time.sleep(0.05)
        return False

    def zero_window(self, duration=0.6):
        start = time.monotonic()
        seen = 0
        _, _, _, _, last_count = self.snapshot()
        while time.monotonic() - start < duration:
            planar, _, _, wz, count = self.snapshot()
            if (planar is not None and planar > 1e-6) or abs(wz) > 1e-6:
                return False, seen
            if count != last_count:
                seen += count - last_count
                last_count = count
            time.sleep(0.02)
        return seen >= 3, seen

    def sample_planar(self, duration=0.5):
        start = time.monotonic()
        maximum = 0.0
        seen = 0
        _, _, _, _, last_count = self.snapshot()
        while time.monotonic() - start < duration:
            planar, _, _, _, count = self.snapshot()
            if planar is not None:
                maximum = max(maximum, planar)
            if count != last_count:
                seen += count - last_count
                last_count = count
            time.sleep(0.02)
        return maximum, seen

    def send_goal(self):
        if not self.action_client.wait_for_server(timeout_sec=60.0):
            raise RuntimeError("action server /follow_path not available")

        path = Path()
        path.header.frame_id = "odom"
        path.header.stamp = self.get_clock().now().to_msg()
        for index in range(21):
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = 0.1 * index
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)

        goal = FollowPath.Goal()
        goal.path = path
        goal.controller_id = "FollowPath"
        goal.goal_checker_id = "general_goal_checker"

        deadline = time.monotonic() + 30.0
        while time.monotonic() < deadline:
            goal_handle = self.wait_future(self.action_client.send_goal_async(goal), 5.0)
            if goal_handle is not None and goal_handle.accepted:
                self.goal_handle = goal_handle
                return
            time.sleep(0.5)
        raise RuntimeError("FollowPath goal rejected")

    def speed_limit_msg(self, value, percentage=False):
        msg = SpeedLimit()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.percentage = percentage
        msg.speed_limit = value
        return msg

    def check_mode_output(self):
        planar, vx, vy, wz, _ = self.snapshot()
        if planar is None:
            raise RuntimeError("no /cmd_vel messages")
        if self.chassis_type == "diff":
            if abs(vy) > 1e-6:
                raise RuntimeError(f"diff chassis published vy={vy:.3f} (expected 0)")
            if abs(wz) < 0.1:
                raise RuntimeError(f"diff chassis did not turn toward path (wz={wz:.3f})")
            if abs(vx) > 0.2:
                raise RuntimeError(f"diff chassis kept large vx={vx:.3f} with 90 deg error")
            self.get_logger().info(
                f"PASS diff kinematics: vx={vx:.3f} vy={vy:.3f} wz={wz:.3f}")
        else:
            if vx <= 0.1:
                raise RuntimeError(f"omni forward command too small (vx={vx:.3f})")
            if abs(vy) > 0.05:
                raise RuntimeError(f"unexpected omni lateral command (vy={vy:.3f})")
            if abs(wz) > 0.05:
                raise RuntimeError(f"unexpected yaw command on straight path (wz={wz:.3f})")
            self.get_logger().info(
                f"PASS omni kinematics: vx={vx:.3f} vy={vy:.3f} wz={wz:.3f}")

    def run(self):
        if not self.action_client.wait_for_server(timeout_sec=60.0):
            raise RuntimeError("action server /follow_path not available")
        if not self.mock_client.wait_for_service(timeout_sec=30.0):
            raise RuntimeError("mock heartbeat service not available")
        self.send_goal()
        self.get_logger().info("FollowPath goal accepted")

        time.sleep(0.6)
        ok, seen = self.zero_window(0.6)
        if not ok:
            raise RuntimeError(
                f"expected zero velocity before first heartbeat (samples={seen})")
        self.get_logger().info("PASS zero velocity before first heartbeat")

        self.set_mock(True)
        if not self.wait_nonzero(3.0):
            raise RuntimeError("expected non-zero command with valid heartbeat")
        self.get_logger().info("PASS non-zero command with valid heartbeat")
        self.check_mode_output()

        self.set_mock(False)
        time.sleep(0.8)
        ok, seen = self.zero_window(0.6)
        if not ok:
            raise RuntimeError(
                f"expected zero velocity after heartbeat timeout (samples={seen})")
        self.get_logger().info("PASS zero velocity after heartbeat timeout")

        self.set_mock(True)
        if not self.wait_nonzero(3.0):
            raise RuntimeError("expected command recovery after heartbeat resumes")
        self.get_logger().info("PASS recovery after heartbeat resumes")

        if self.chassis_type != "diff":
            self.speed_limit_pub.publish(self.speed_limit_msg(0.2))
            time.sleep(0.6)
            maximum, seen = self.sample_planar(0.5)
            if seen < 3 or maximum > 0.22:
                raise RuntimeError(
                    f"speed limit 0.2 m/s violated (max={maximum:.3f}, samples={seen})")
            self.get_logger().info(
                f"PASS absolute speed limit: max planar speed={maximum:.3f} m/s")

            self.speed_limit_pub.publish(self.speed_limit_msg(0.0))
            time.sleep(0.6)
            if not self.wait_nonzero(2.0):
                raise RuntimeError("NO_SPEED_LIMIT did not restore chassis speed limit")
            self.get_logger().info("PASS NO_SPEED_LIMIT restores chassis limit")

        if self.goal_handle is not None:
            self.goal_handle.cancel_goal_async()
        return True


def main():
    rclpy.init()
    node = SmokeCheck()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    success = False
    try:
        success = node.run()
        print("RESULT: PASS", flush=True)
    except Exception as exc:  # noqa: BLE001 - the smoke harness reports any failure
        node.get_logger().error(f"smoke check failed: {exc}")
        print(f"RESULT: FAIL: {exc}", flush=True)
    finally:
        executor.shutdown()
        if spin_thread.is_alive():
            spin_thread.join(timeout=2.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    sys.exit(0 if success else 1)


if __name__ == "__main__":
    main()
