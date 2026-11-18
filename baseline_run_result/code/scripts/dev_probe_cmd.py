#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""速度链探针：每 2 s 打印 /cmd_vel_raw → /cmd_vel_limited → /cmd_vel 与 /odom。

用法（栈已启动）：python3 scripts/dev_probe_cmd.py [时长秒]
"""
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

DUR = float(sys.argv[1]) if len(sys.argv) > 1 else 40.0
st = {"raw": 0.0, "lim": 0.0, "cmd": 0.0, "n_raw": 0, "n_lim": 0, "n_cmd": 0,
      "odom": None, "n_odom": 0}

rclpy.init()
node = rclpy.create_node("dev_probe_cmd")
node.create_subscription(
    Twist, "/cmd_vel_raw", lambda m: (st.__setitem__("raw", m.linear.x), st.__setitem__("n_raw", st["n_raw"] + 1)), 50)
node.create_subscription(
    Twist, "/cmd_vel_limited", lambda m: (st.__setitem__("lim", m.linear.x), st.__setitem__("n_lim", st["n_lim"] + 1)), 50)
node.create_subscription(
    Twist, "/cmd_vel", lambda m: (st.__setitem__("cmd", m.linear.x), st.__setitem__("n_cmd", st["n_cmd"] + 1)), 50)
node.create_subscription(
    Odometry, "/odom", lambda m: (st.__setitem__("odom", (m.pose.pose.position.x, m.pose.pose.position.y)),
                                  st.__setitem__("n_odom", st["n_odom"] + 1)), 50)

t0 = time.monotonic()
nxt = 0.0
while time.monotonic() - t0 < DUR:
    rclpy.spin_once(node, timeout_sec=0.05)
    if time.monotonic() < nxt:
        continue
    nxt = time.monotonic() + 2.0
    od = st["odom"]
    print("t=%4.0f robot=(%s) raw=%.2f[%d] lim=%.2f[%d] cmd=%.2f[%d] odom=%d" % (
        time.monotonic() - t0,
        "-" if od is None else "%.2f,%.2f" % od,
        st["raw"], st["n_raw"], st["lim"], st["n_lim"], st["cmd"], st["n_cmd"], st["n_odom"]), flush=True)
