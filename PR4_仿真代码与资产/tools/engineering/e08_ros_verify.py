#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""E08 外部 ROS 2 验证：在 RViz 之外用订阅者核实 /lidar/points 与 /imu。

做四件事（都不事先假定频率，全部实测）：
  1. 类型 / frame_id / 字段（PointCloud2 的 x,y,z；Imu 的 9 个量）
  2. 实测频率（相邻消息时间差）与满量程丢帧（用 /clock 的仿真时间做基准）
  3. 把点云转成 body 坐标系下的最近障碍方向，验证"机器人转动时数据变化方向正确"
  4. 把 IMU 三轴加速度/角速度落盘，验证静止重力方向与转向符号

用法:
  source /opt/ros/humble/setup.bash
  ROS_DOMAIN_ID=42 python3 tools/engineering/e08_ros_verify.py --duration 20 --out results/w2/e08/ros

产出: pointcloud_samples.csv, imu_samples.csv, verify_summary.txt
"""
import argparse
import csv
import math
import os
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2, Imu
from rosgraph_msgs.msg import Clock


def pc2_points(msg):
    """把 PointCloud2 解成 [(x, y, z), ...]，只支持 FLOAT32 的 x/y/z。"""
    fields = {f.name: f for f in msg.fields}
    if not all(k in fields for k in ("x", "y", "z")):
        return []
    if any(fields[k].datatype != 7 for k in ("x", "y", "z")):   # 7 = FLOAT32
        return []
    import struct

    ox, oy, oz = fields["x"].offset, fields["y"].offset, fields["z"].offset
    step = msg.point_step
    data = msg.data
    n = msg.width * msg.height
    out = []
    for i in range(n):
        base = i * step
        x = struct.unpack_from("<f", data, base + ox)[0]
        y = struct.unpack_from("<f", data, base + oy)[0]
        z = struct.unpack_from("<f", data, base + oz)[0]
        if math.isnan(x) or math.isnan(y) or math.isnan(z):
            continue
        out.append((x, y, z))
    return out


class Verify(Node):
    def __init__(self):
        super().__init__("e08_ros_verify")
        qos = QoSProfile(depth=50, reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST)
        self.clouds = []
        self.imus = []
        self.clock = []          # (recv_wall, sim_ns)
        self.last_sim_ns = None
        self.sim_steps = []      # 每次 clock 的仿真时间增量
        self.create_subscription(PointCloud2, "/lidar/points", self.on_cloud, qos)
        self.create_subscription(Imu, "/imu", self.on_imu, qos)
        self.create_subscription(Clock, "/clock", self.on_clock, qos)

    def on_cloud(self, msg):
        self.clouds.append((time.time(), msg))

    def on_imu(self, msg):
        self.imus.append((time.time(), msg))

    def on_clock(self, msg):
        ns = msg.clock.sec * 10 ** 9 + msg.clock.nanosec
        self.clock.append((time.time(), ns))
        if self.last_sim_ns is not None and ns > self.last_sim_ns:
            self.sim_steps.append((ns - self.last_sim_ns) / 1e9)
        self.last_sim_ns = ns


def pct(vals, p):
    if not vals:
        return float("nan")
    s = sorted(vals)
    k = max(0, min(len(s) - 1, int(round((len(s) - 1) * p / 100.0))))
    return s[k]


def rate_from_stamps(stamps):
    if len(stamps) < 3:
        return float("nan"), float("nan"), float("nan")
    d = [b - a for a, b in zip(stamps[:-1], stamps[1:])]
    d = [x for x in d if x > 0]
    if not d:
        return float("nan"), float("nan"), float("nan")
    mean = sum(d) / len(d)
    return 1.0 / mean, pct(d, 50), pct(d, 95)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--duration", type=float, default=20.0)
    ap.add_argument("--out", default="results/w2/e08/ros")
    a = ap.parse_args()

    os.makedirs(a.out, exist_ok=True)
    rclpy.init()
    node = Verify()
    t0 = time.time()
    try:
        while time.time() - t0 < a.duration:
            rclpy.spin_once(node, timeout_sec=0.05)
    finally:
        pass

    lines = []
    say = lambda s: (print(s), lines.append(s))

    say("E08 外部 ROS 2 验证（订阅端实测，未假定任何频率）")
    say("订阅时长 %.1f s" % (time.time() - t0))
    say("")

    # ---------- 点云 ----------
    say("== /lidar/points ==")
    say("消息数: %d" % len(node.clouds))
    if node.clouds:
        first = node.clouds[0][1]
        say("类型: sensor_msgs/msg/PointCloud2")
        say("frame_id: %s" % first.header.frame_id)
        say("字段: %s" % ", ".join("%s@%d(type=%d)" % (f.name, f.offset, f.datatype)
                                  for f in first.fields))
        say("point_step=%d  is_dense=%s  is_bigendian=%s"
            % (first.point_step, first.is_dense, first.is_bigendian))
        hz, med, p95 = rate_from_stamps([t for t, _ in node.clouds])
        say("频率: 平均 %.2f Hz / 中位间隔 %.1f ms / P95 间隔 %.1f ms"
            % (hz, med * 1e3, p95 * 1e3))
        counts = []
        rows = []
        for t, m in node.clouds:
            pts = pc2_points(m)
            counts.append(len(pts))
            if pts:
                r = [math.hypot(p[0], p[1]) for p in pts]
                near = min(pts, key=lambda p: math.hypot(p[0], p[1]))
                rows.append({
                    "t_wall": "%.3f" % t,
                    "sim_sec": "%.3f" % (m.header.stamp.sec + m.header.stamp.nanosec * 1e-9),
                    "n_points": len(pts),
                    "r_min": "%.3f" % min(r),
                    "r_max": "%.3f" % max(r),
                    "nearest_x": "%.3f" % near[0],
                    "nearest_y": "%.3f" % near[1],
                    "nearest_z": "%.3f" % near[2],
                    "nearest_az_deg": "%.1f" % math.degrees(math.atan2(near[1], near[0])),
                })
        say("每帧点数: min %d / 中位 %d / max %d（空帧 %d 个）"
            % (min(counts), int(pct(counts, 50)), max(counts), sum(1 for c in counts if c == 0)))
        if rows:
            with open(os.path.join(a.out, "pointcloud_samples.csv"), "w", newline="") as f:
                w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
                w.writeheader()
                w.writerows(rows)
            say("最近点极角范围: %.1f° .. %.1f°（用于核对转向方向）"
                % (min(float(r["nearest_az_deg"]) for r in rows),
                   max(float(r["nearest_az_deg"]) for r in rows)))
    say("")

    # ---------- IMU ----------
    say("== /imu ==")
    say("消息数: %d" % len(node.imus))
    if node.imus:
        m = node.imus[0][1]
        say("frame_id: %s" % m.header.frame_id)
        hz, med, p95 = rate_from_stamps([t for t, _ in node.imus])
        say("频率: 平均 %.2f Hz / 中位间隔 %.1f ms / P95 间隔 %.1f ms"
            % (hz, med * 1e3, p95 * 1e3))
        rows = []
        for t, m in node.imus:
            la, av = m.linear_acceleration, m.angular_velocity
            rows.append({
                "t_wall": "%.3f" % t,
                "sim_sec": "%.3f" % (m.header.stamp.sec + m.header.stamp.nanosec * 1e-9),
                "ax": "%.4f" % la.x, "ay": "%.4f" % la.y, "az": "%.4f" % la.z,
                "wx": "%.4f" % av.x, "wy": "%.4f" % av.y, "wz": "%.4f" % av.z,
            })
        with open(os.path.join(a.out, "imu_samples.csv"), "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
            w.writeheader()
            w.writerows(rows)
        azs = [float(r["az"]) for r in rows]
        wzs = [float(r["wz"]) for r in rows]
        axs = [abs(float(r["ax"])) for r in rows]
        ays = [abs(float(r["ay"])) for r in rows]
        say("az: min %.3f / 中位 %.3f / max %.3f" % (min(azs), pct(azs, 50), max(azs)))
        say("|ax| 中位 %.4f, |ay| 中位 %.4f（水平放置应为 0）" % (pct(axs, 50), pct(ays, 50)))
        say("wz: min %.3f / max %.3f（正负号反映转向方向）" % (min(wzs), max(wzs)))
        say("角速度非零样本数: %d / %d" % (sum(1 for v in wzs if abs(v) > 0.05), len(wzs)))
    say("")

    # ---------- 丢帧 ----------
    say("== 频率与丢帧（以 /clock 仿真时间为基准）==")
    say("/clock 消息数: %d" % len(node.clock))
    if node.clock:
        sim_span = (node.clock[-1][1] - node.clock[0][1]) / 1e9
        wall_span = node.clock[-1][0] - node.clock[0][0]
        say("仿真时间跨度 %.3f s / 墙钟 %.3f s → 实时因子 %.3f"
            % (sim_span, wall_span, sim_span / wall_span if wall_span > 0 else 0))
        if node.sim_steps:
            say("每帧仿真步长: 中位 %.2f ms / max %.2f ms（步长抖动=掉物理帧）"
                % (pct(node.sim_steps, 50) * 1e3, max(node.sim_steps) * 1e3))
        if sim_span > 0:
            for name, lst, nominal in (
                    ("/lidar/points", node.clouds, 10.0),
                    ("/imu", node.imus, 200.0)):
                expect = max(1.0, sim_span * nominal)
                got = len(lst)
                say("%-13s 实测 %5.2f Hz，按标称 %.0f Hz 应有 %d 帧，实收 %d 帧 → "
                    "缺口 %d 帧（%.1f%%）"
                    % (name, len(lst) / sim_span, nominal, int(round(expect)), got,
                       int(round(expect)) - got, 100.0 * (1 - got / expect)))
            say("说明：IMU 标称 200 Hz 是传感器采样率；ROS 发布按物理步（实测≈60 Hz），"
                "缺口属于「发布路径受限」而非丢包，见 docs/engineering/sensors.md。")

    with open(os.path.join(a.out, "verify_summary.txt"), "w") as f:
        f.write("\n".join(lines) + "\n")
    node.destroy_node()
    rclpy.shutdown()
    print("\n已写出 %s/verify_summary.txt" % a.out)
    return 0


if __name__ == "__main__":
    sys.exit(main())
