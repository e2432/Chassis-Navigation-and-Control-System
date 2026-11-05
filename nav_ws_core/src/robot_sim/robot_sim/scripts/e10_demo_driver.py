#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""E10 演示指令源：按 M1 验收要求发 /cmd_vel（并打印自己在做什么）。

序列（与路线书 M1 对应）。**跑完就退出**——否则它会以 20 Hz 持续发零速，
把你之后手动发的 `/cmd_vel` 全部盖掉（本机实测踩到过：手动 pub 0.2 m/s 车不动）。
  1) 0.1 m/s 前进 5 s（M1：发 0.1 m/s 指令能移动）
  2) 0.1 m/s 左移 5 s（全向验证）
  3) 原地逆时针 0.5 rad/s 2 s
  4) 停 3 s（看静止稳定性）

用法：ros2 run robot_sim e10_demo_driver.py --ros-args -p use_sim_time:=true
"""

import threading

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node

SEGMENTS = [
    ("前进 0.1 m/s", 0.1, 0.0, 0.0, 5.0),
    ("左移 0.1 m/s", 0.0, 0.1, 0.0, 5.0),
    ("原地逆时针 0.5 rad/s", 0.0, 0.0, 0.5, 2.0),
    ("停车", 0.0, 0.0, 0.0, 3.0),
]


class DemoDriver(Node):
    def __init__(self):
        super().__init__("e10_demo_driver")
        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.i = 0
        self.t0 = None
        self._timer = self.create_timer(0.05, self.tick)      # 20 Hz
        self.get_logger().info("E10 演示开始：%s" % [s[0] for s in SEGMENTS])

    def tick(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.t0 is None:
            self.t0 = now
        elapsed = now - self.t0
        acc = 0.0
        for idx, (name, vx, vy, wz, hold) in enumerate(SEGMENTS):
            if elapsed < acc + hold:
                if idx != self.i:
                    self.i = idx
                    self.get_logger().info("指令段 %d/%d：%s" % (idx + 1, len(SEGMENTS), name))
                msg = Twist()
                msg.linear.x = vx
                msg.linear.y = vy
                msg.angular.z = wz
                self.pub.publish(msg)
                return
            acc += hold
        if getattr(self, "_done", False):
            return
        msg = Twist()                     # 结束时发一次零速
        self.pub.publish(msg)
        self._done = True
        self.get_logger().info("E10 演示结束：已发一次零速并退出本节点"
                               "（此后 /cmd_vel 完全交给你自己的终端）")
        if self._timer is not None:
            self._timer.cancel()
        # 稍等一拍让消息发出，然后关节点（launch 会看到该进程正常退出）
        threading.Timer(0.5, self._shutdown).start()

    def _shutdown(self):
        rclpy.shutdown()


def main(argv=None):
    rclpy.init(args=argv)
    node = DemoDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
