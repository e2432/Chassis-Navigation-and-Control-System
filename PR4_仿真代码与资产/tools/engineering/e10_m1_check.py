#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""E10 / M1 验收检查：一条命令启动后，用订阅端独立核对四件事。

M1 要求（路线书）：
  1) 一条已记录命令启动（由 e10_run_all.sh 记录启动日志）；
  2) 发 0.1 m/s 指令能移动；
  3) /odom 连续输出；
  4) RViz 点云可见（截图由 e10_run_all.sh 抓）。

这里实测 2)、3) 和点云本身，并检查 TF 是否可解析（无 extrapolation 错误）。
"""
import argparse, csv, math, os, statistics as st, sys, time
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import PointCloud2
from tf2_ros import Buffer, TransformListener


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class M1(Node):
    def __init__(self):
        super().__init__("e10_m1_check")
        q = QoSProfile(depth=100, reliability=ReliabilityPolicy.BEST_EFFORT,
                       history=HistoryPolicy.KEEP_LAST)
        self.odom, self.clouds, self.clock = [], [], []
        self.create_subscription(Odometry, "/odom", lambda m: self.odom.append(
            (m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, m)), q)
        self.create_subscription(PointCloud2, "/lidar/points", lambda m: self.clouds.append(
            (m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, m)), q)
        self.create_subscription(Clock, "/clock", lambda m: self.clock.append(
            m.clock.sec + m.clock.nanosec * 1e-9), q)
        self.buf = Buffer()
        self.listener = TransformListener(self.buf, self)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--duration", type=float, default=25.0)
    ap.add_argument("--out", default="results/w3/e10")
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    rclpy.init()
    node = M1()
    lines = []
    say = lambda s: (print(s), lines.append(s))
    ok = [True]

    def check(name, cond, detail=""):
        ok[0] = ok[0] and bool(cond)
        say("[%s] %s%s" % ("PASS" if cond else "FAIL", name, (" — " + detail) if detail else ""))

    # 热身：等 DDS 发现完成（收不到 /clock 就继续等，最多 8 s）
    tw = time.time()
    while time.time() - tw < 8.0 and not node.clock:
        rclpy.spin_once(node, timeout_sec=0.05)
    node.get_logger().info("热身结束：/clock 已收到 %d 条" % len(node.clock))
    # 统计窗口从"开始有数据"算起
    t0 = time.time()
    while time.time() - t0 < a.duration:
        rclpy.spin_once(node, timeout_sec=0.05)

    say("E10 / M1 验收（订阅端实测 %.1f s）" % (time.time() - t0))
    say("/odom %d 条、/lidar/points %d 帧、/clock %d 条" % (len(node.odom), len(node.clouds),
                                                    len(node.clock)))
    say("")

    # ---- 2) /odom 连续 ----
    if len(node.odom) < 10:
        check("/odom 连续输出", False, "只有 %d 条" % len(node.odom))
    else:
        ts = [s for s, _ in node.odom]
        gaps = [b - a_ for a_, b in zip(ts[:-1], ts[1:])]
        span = ts[-1] - ts[0]
        hz = (len(ts) - 1) / span
        say("/odom：%d 条 / %.2f s → %.2f Hz，最大间隔 %.1f ms（断流阈值 500 ms）"
            % (len(ts), span, hz, max(gaps) * 1e3))
        check("/odom 连续输出（无 >500 ms 断流）", max(gaps) < 0.5, "max %.1f ms" % (max(gaps) * 1e3))
        check("/odom 频率 >20 Hz", hz > 20.0, "%.2f Hz" % hz)

    # ---- 3) 0.1 m/s 指令能移动 ----
    if len(node.odom) > 10:
        xs = [m.pose.pose.position.x for _, m in node.odom]
        ys = [m.pose.pose.position.y for _, m in node.odom]
        path = sum(math.hypot(xs[i] - xs[i - 1], ys[i] - ys[i - 1]) for i in range(1, len(xs)))
        seg = []
        for i in range(1, len(node.odom)):
            _, m0 = node.odom[i - 1]
            _, m1 = node.odom[i]
            seg.append(math.hypot(m1.pose.pose.position.x - m0.pose.pose.position.x,
                                  m1.pose.pose.position.y - m0.pose.pose.position.y))
        say("轨迹总长度 %.3f m（净位移 %.3f m）；最大单步 %.4f m"
            % (path, math.hypot(xs[-1] - xs[0], ys[-1] - ys[0]), max(seg)))
        check("收到 0.1 m/s 指令后机器人确实移动（>2 cm）", path > 0.02, "%.3f m" % path)
        vmax = max(math.hypot(m.twist.twist.linear.x, m.twist.twist.linear.y) for _, m in node.odom)
        say("实测线速度峰值 %.3f m/s（指令 0.1：physics 模式约 30~40%%（见 docs/engineering/motion_check.md），"
            "kinematic 模式≈0.1）" % vmax)
        check("速度量级合理（<2 m/s 上限）", vmax < 2.0, "%.3f m/s" % vmax)

    # ---- 4) 点云可见 ----
    if node.clouds:
        counts = []
        for _, m in node.clouds[::5]:
            counts.append(m.width * m.height)
        first = node.clouds[0][1]
        say("/lidar/points：%d 帧，每帧点数中位 %d，frame_id=%s，字段=%s"
            % (len(node.clouds), int(st.median(counts)), first.header.frame_id,
               ",".join(f.name for f in first.fields)))
        check("点云非空（每帧 >100 点）", st.median(counts) > 100, "%d 点" % int(st.median(counts)))
        check("点云 frame_id=lidar_front_link", first.header.frame_id == "lidar_front_link",
              first.header.frame_id)
    else:
        check("收到 /lidar/points", False, "0 帧")

    # ---- TF 可解析（无 extrapolation） ----
    try:
        tr = node.buf.lookup_transform("map", "base_link", Time())
        check("TF map→base_link 可解析", True,
              "(%.3f, %.3f, %.3f)" % (tr.transform.translation.x, tr.transform.translation.y,
                                      tr.transform.translation.z))
    except Exception as exc:
        check("TF map→base_link 可解析", False, type(exc).__name__)

    with open(os.path.join(a.out, "m1_check.csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["sim_t", "x", "y", "yaw", "vx", "vy", "wz"])
        for s, m in node.odom:
            w.writerow(["%.4f" % s, "%.4f" % m.pose.pose.position.x,
                        "%.4f" % m.pose.pose.position.y, "%.4f" % yaw_of(m.pose.pose.orientation),
                        "%.4f" % m.twist.twist.linear.x, "%.4f" % m.twist.twist.linear.y,
                        "%.4f" % m.twist.twist.angular.z])
    say("")
    say("结果：%s" % ("PASS" if ok[0] else "FAIL"))
    with open(os.path.join(a.out, "m1_check.txt"), "w") as f:
        f.write("\n".join(lines) + "\n")
    node.destroy_node()
    rclpy.shutdown()
    return 0 if ok[0] else 1


if __name__ == "__main__":
    sys.exit(main())
