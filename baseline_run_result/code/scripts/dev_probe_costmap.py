#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""局部代价地图 ASCII 探针：每 2 s 打印机器人位置 + 窗口内 lethal/inscribed 形状。

用法（栈已启动）：
    python3 scripts/dev_probe_costmap.py [时长秒]
"""
import sys
import time

import rclpy
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

DUR = float(sys.argv[1]) if len(sys.argv) > 1 else 40.0

st = {"odom": None, "loc": None, "n_upd": 0}
rclpy.init()
node = rclpy.create_node("dev_probe_costmap")
node.create_subscription(
    Odometry, "/odom",
    lambda m: st.__setitem__("odom", (m.pose.pose.position.x, m.pose.pose.position.y)), 10)


def loc_cb(m):
    st["loc"] = m
    st["n_upd"] += 1


node.create_subscription(
    OccupancyGrid, "/local_costmap/costmap", loc_cb,
    QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
               durability=DurabilityPolicy.TRANSIENT_LOCAL,
               history=HistoryPolicy.KEEP_LAST))

t0 = time.monotonic()
nxt = 0.0
while time.monotonic() - t0 < DUR:
    rclpy.spin_once(node, timeout_sec=0.05)
    if time.monotonic() < nxt:
        continue
    nxt = time.monotonic() + 2.0
    od, m = st["odom"], st["loc"]
    if od is None or m is None:
        continue
    step = max(1, m.info.width // 24)
    lethal = sum(1 for v in m.data if v == 100)
    insc = sum(1 for v in m.data if v == 99)
    print("robot=(%.2f,%.2f) 更新次数=%d 窗口x=%.1f..%.1f leth=%d insc=%d" % (
        od[0], od[1], st["n_upd"], m.info.origin.position.x,
        m.info.origin.position.x + m.info.width * m.info.resolution, lethal, insc), flush=True)
    for jy in range(m.info.height - 1, -1, -step):
        row = ""
        for jx in range(0, m.info.width, step):
            v = m.data[jy * m.info.width + jx]
            cx = m.info.origin.position.x + (jx + 0.5) * m.info.resolution
            cy = m.info.origin.position.y + (jy + 0.5) * m.info.resolution
            if abs(cx - od[0]) < 0.35 and abs(cy - od[1]) < 0.35:
                row += "R"
            elif v == 100:
                row += "#"
            elif v >= 99:
                row += "x"
            elif v > 0:
                row += "-"
            else:
                row += "."
        print("   " + row, flush=True)

node.destroy_node()
rclpy.shutdown()
