#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""E10 运动探针：发一段指令，用 /odom 量化"到底动了多少"，不用靠眼睛看。

用途（排查"发了指令但看起来不动"）：
  * 报告起点位置（如果 x≈2.6~3.0，说明车已经顶在世界 x=3 m 的墙上，平移当然不动，旋转还正常）
  * 分四段测量：平移 +x、平移 +y、旋转 +wz、平移+旋转，各段给出 Δ、平均速度、与指令的比值

用法:
  ros2 run robot_sim e10_motion_probe.py            # 默认测 4 段
  ros2 run robot_sim e10_motion_probe.py --vx 0.2 --secs 5
"""
import argparse
import math
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class Probe(Node):
    def __init__(self):
        super().__init__("e10_motion_probe")
        self.cur = None
        qos = QoSProfile(depth=50, reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(Odometry, "/odom", self.cb, qos)
        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)

    def cb(self, m):
        p = m.pose.pose.position
        self.cur = (p.x, p.y, yaw_of(m.pose.pose.orientation))

    def spin(self, secs, vx=0.0, vy=0.0, wz=0.0):
        t = time.time()
        while time.time() - t < secs:
            m = Twist()
            m.linear.x, m.linear.y, m.angular.z = vx, vy, wz
            self.pub.publish(m)
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(0.05)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--vx", type=float, default=0.2)
    ap.add_argument("--vy", type=float, default=0.2)
    ap.add_argument("--wz", type=float, default=0.5)
    ap.add_argument("--secs", type=float, default=5.0)
    a = ap.parse_args()

    rclpy.init()
    n = Probe()
    t0 = time.time()
    while time.time() - t0 < 8.0 and n.cur is None:
        rclpy.spin_once(n, timeout_sec=0.05)
    if n.cur is None:
        print("8 秒内没收到 /odom —— 检查 ROS_DOMAIN_ID 是否与 launch 一致（应为 42）")
        return 1

    x, y, yaw = n.cur
    print("起点：x=%.3f y=%.3f yaw=%.1f°" % (x, y, math.degrees(yaw)))
    if math.hypot(x, y) > 2.5:
        print("⚠ 注意：离原点已 %.2f m（世界 x=3 m 处有墙、1.5 m 处有箱体）——"
              "若已顶住障碍物，平移会被挡住而**旋转仍然正常**，这正是\"平移不动\"的常见原因"
              % math.hypot(x, y))
    print("")

    def seg(label, vx, vy, wz, secs):
        s = n.cur
        n.spin(secs, vx, vy, wz)
        n.spin(1.0)                      # 停车 1 s
        e = n.cur
        dx, dy = e[0] - s[0], e[1] - s[1]
        c, sn = math.cos(s[2]), math.sin(s[2])
        bx, by = dx * c + dy * sn, -dx * sn + dy * c
        dyaw = math.atan2(math.sin(e[2] - s[2]), math.cos(e[2] - s[2]))
        v_meas = math.hypot(dx, dy) / secs
        v_cmd = math.hypot(vx, vy)
        ratio = (v_meas / v_cmd) if v_cmd > 1e-9 else float("nan")
        print("%-16s cmd=(%.2f,%.2f,%.2f) %.0fs → 世界 Δ=(%+.3f,%+.3f)  机体 Δ=(%+.3f,%+.3f)  "
              "Δyaw=%+.3f  实测速度 %.3f m/s（指令的 %.0f%%）"
              % (label, vx, vy, wz, secs, dx, dy, bx, by, dyaw, v_meas, 100.0 * ratio))
        return bx, by, dyaw

    print("== 分段测量（physics 模式：平移约 30~45% 且会偏 45°；kinematic 模式接近 100%）==")
    bx1, by1, _ = seg("平移 +x", a.vx, 0.0, 0.0, a.secs)
    seg("平移 +y", 0.0, a.vy, 0.0, a.secs)
    _, _, dyaw = seg("旋转 +wz", 0.0, 0.0, a.wz, 2.0)
    seg("平移+旋转", a.vx, 0.0, 0.3, 4.0)
    print("")
    verdict = []
    if math.hypot(bx1, by1) < 0.02:
        verdict.append("平移几乎为 0 → 检查是否顶住障碍物、或 mode=physics 下轮子打滑")
    else:
        verdict.append("平移有位移（%.3f m）" % math.hypot(bx1, by1))
    verdict.append("旋转 %s" % ("正常" if abs(dyaw) > 0.3 else "异常"))
    print("结论：" + "；".join(verdict))
    n.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
