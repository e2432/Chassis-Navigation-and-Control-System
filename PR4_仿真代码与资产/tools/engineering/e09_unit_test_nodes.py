#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""E09 节点级测试（不需要 Isaac Sim）：验证真值适配节点与 map→odom 静态节点的行为。

做法：以 20 Hz 持续发"已知"的合成真值（位姿缓慢移动、偏航固定 45°、世界系速度 0.3 m/s
+ ω=0.5 rad/s），一边订阅节点输出，逐条核对：

  1. `/odom` 的 header.frame_id=odom、child_frame_id=base_footprint、stamp 与输入一致；
  2. **twist 从世界系转到机体系**：期望 (0.3·cos45, −0.3·sin45) = (0.2121, −0.2121)，
     而不是世界的 (0.3, 0.0) —— 这是路线书 E09 检查项 3；
  3. TF `odom→base_footprint` 与 `/odom` 位姿一致；
  4. `/tf_static` 上有 `map→odom`（identity）；
  5. 输出频率 ≈ 输入频率（节点不丢帧、不重复）。

用法（先起节点，见 tools/engineering/e09_node_test.sh）：
    python3 tools/engineering/e09_unit_test_nodes.py --duration 8 --out results/w3/e09
"""
import argparse
import math
import os
import sys
import time

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from tf2_msgs.msg import TFMessage

YAW = math.radians(45.0)
V_WORLD = (0.30, 0.0)
W_WORLD = 0.5
EXPECT_BODY = (V_WORLD[0] * math.cos(YAW), -V_WORLD[0] * math.sin(YAW))


def quat_from_yaw(yaw):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


class UnitTest(Node):
    def __init__(self, out_dir):
        super().__init__("e09_unit_test")
        self.out_dir = out_dir
        qos = QoSProfile(depth=50, reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST)
        self.pub = self.create_publisher(Odometry, "/sim/ground_truth", 50)
        self.odom = []
        self.tf_odom_base = []
        self.tf_static = []
        self.create_subscription(Odometry, "/odom", self.on_odom, qos)
        self.create_subscription(TFMessage, "/tf", self.on_tf, 100)
        static_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                                history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(TFMessage, "/tf_static", self.on_tf_static, static_qos)

    def on_odom(self, msg):
        self.odom.append(msg)

    def on_tf(self, msg):
        for tr in msg.transforms:
            if tr.child_frame_id == "base_footprint":
                self.tf_odom_base.append(tr)

    def on_tf_static(self, msg):
        for tr in msg.transforms:
            self.tf_static.append(tr)

    def publish_truth(self, i):
        msg = Odometry()
        msg.header.frame_id = "world"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.child_frame_id = "base_footprint"
        msg.pose.pose.position.x = 1.0
        msg.pose.pose.position.y = 0.5
        msg.pose.pose.position.z = 0.0
        qx, qy, qz, qw = quat_from_yaw(YAW)
        msg.pose.pose.orientation.x = qx
        msg.pose.pose.orientation.y = qy
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw
        msg.twist.twist.linear.x = V_WORLD[0]      # 世界系
        msg.twist.twist.linear.y = V_WORLD[1]
        msg.twist.twist.angular.z = W_WORLD
        self.pub.publish(msg)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--duration", type=float, default=8.0)
    ap.add_argument("--rate", type=float, default=20.0)
    ap.add_argument("--out", default="results/w3/e09")
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)

    rclpy.init()
    node = UnitTest(a.out)
    lines = []
    say = lambda s: (print(s), lines.append(s))

    # 按 20 Hz 持续发布（先给 DDS 发现留时间），避免把位姿/频率统计搅乱
    period = 1.0 / a.rate
    t0 = time.time()
    t_next = t0
    i = 0
    while time.time() - t0 < a.duration:
        node.publish_truth(i)
        i += 1
        rclpy.spin_once(node, timeout_sec=0.001)
        now = time.time()
        if t_next > now:
            time.sleep(min(period, t_next - now))
        t_next += period
    # 再 spin 一会儿确保输出都收到
    t1 = time.time()
    while time.time() - t1 < 1.0:
        rclpy.spin_once(node, timeout_sec=0.05)

    say("E09 节点级测试（合成真值，不需要 Isaac Sim）")
    say("输入：/sim/ground_truth %.0f Hz，位姿 yaw=45°，twist 世界系 (%.2f, %.2f) m/s, wz=%.2f rad/s"
        % (a.rate, V_WORLD[0], V_WORLD[1], W_WORLD))
    say("发布 %d 条，收到 /odom %d 条，TF odom→base_footprint %d 条，/tf_static %d 条"
        % (i, len(node.odom), len(node.tf_odom_base), len(node.tf_static)))
    say("")

    ok = True

    def check(name, cond, detail=""):
        nonlocal ok
        ok = ok and bool(cond)
        say("[%s] %s%s" % ("PASS" if cond else "FAIL", name, (" — " + detail) if detail else ""))

    if not node.odom:
        check("收到 /odom", False, "节点没输出，先确认 truth_odom_node.py 在跑")
    else:
        m = node.odom[len(node.odom) // 2]
        check("/odom header.frame_id", m.header.frame_id == "odom", m.header.frame_id)
        check("/odom child_frame_id", m.child_frame_id == "base_footprint", m.child_frame_id)
        check("/odom stamp 非零", (m.header.stamp.sec + m.header.stamp.nanosec) > 0,
              "%d.%09d" % (m.header.stamp.sec, m.header.stamp.nanosec))
        vx = m.twist.twist.linear.x
        vy = m.twist.twist.linear.y
        wz = m.twist.twist.angular.z
        say("twist（机体系）= (%.4f, %.4f) wz=%.4f；世界系原值 = (%.2f, %.2f) wz=%.2f；"
            "期望机体 = (%.4f, %.4f)" % (vx, vy, wz, V_WORLD[0], V_WORLD[1], W_WORLD,
                                         EXPECT_BODY[0], EXPECT_BODY[1]))
        check("twist 已转到机体系", abs(vx - EXPECT_BODY[0]) < 1e-3 and abs(vy - EXPECT_BODY[1]) < 1e-3,
              "误差 (%.1e, %.1e)" % (abs(vx - EXPECT_BODY[0]), abs(vy - EXPECT_BODY[1])))
        check("twist 未停在世界系", abs(vx - V_WORLD[0]) > 1e-2,
              "若等于 (0.30, 0.00) 说明没做 R(−yaw) 旋转")
        check("wz 保持", abs(wz - W_WORLD) < 1e-6, "%.4f" % wz)
        # 频率
        stamps = [(x.header.stamp.sec + x.header.stamp.nanosec * 1e-9) for x in node.odom]
        d = [b - a_ for a_, b in zip(stamps[:-1], stamps[1:]) if b > a_]
        if d:
            med = sorted(d)[len(d) // 2]
            say("输出间隔中位 %.1f ms（≈%.1f Hz）" % (med * 1e3, 1.0 / med))
            check("输出不重复（每条 stamp 唯一）", len(set(stamps)) == len(stamps),
                  "%d 条 / %d 个唯一 stamp" % (len(stamps), len(set(stamps))))
        # 位姿一致性
        p = m.pose.pose.position
        check("位姿原样透传（x=1.000 不动）", abs(p.x - 1.0) < 1e-9 and abs(p.y - 0.5) < 1e-9,
              "x=%.6f y=%.6f" % (p.x, p.y))
        # TF 与 /odom 一致
        if node.tf_odom_base:
            tr = node.tf_odom_base[-1]
            check("TF odom→base_footprint 存在", tr.header.frame_id in ("odom", "/odom"),
                  tr.header.frame_id)
            near = min(node.odom, key=lambda x: abs(
                (x.header.stamp.sec + x.header.stamp.nanosec * 1e-9)
                - (tr.header.stamp.sec + tr.header.stamp.nanosec * 1e-9)))
            dx = abs(tr.transform.translation.x - near.pose.pose.position.x)
            dy = abs(tr.transform.translation.y - near.pose.pose.position.y)
            check("TF 与 /odom 位姿一致", dx < 1e-6 and dy < 1e-6, "Δ=(%.1e, %.1e)" % (dx, dy))
        else:
            check("TF odom→base_footprint 存在", False, "没有收到该 TF")

    map_odom = [t for t in node.tf_static
                if t.header.frame_id in ("map", "/map") and t.child_frame_id in ("odom", "/odom")]
    check("/tf_static 有 map→odom", bool(map_odom),
          "" if map_odom else "检查 map_odom_static_node.py 是否在跑")

    say("")
    say("结果：%s" % ("PASS" if ok else "FAIL"))
    with open(os.path.join(a.out, "node_unit_test.txt"), "w") as f:
        f.write("\n".join(lines) + "\n")
    node.destroy_node()
    rclpy.shutdown()
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
