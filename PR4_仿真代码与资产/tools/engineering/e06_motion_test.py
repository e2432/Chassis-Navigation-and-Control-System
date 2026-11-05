#!/usr/bin/env python3
"""E06：外部 ROS 2 侧速度指令发布器（系统 python3，rclpy）。

按仿真时间推进测试阶段，向 /cmd_vel 发机体速度，同时向 /e06/heartbeat 发心跳
（心跳用 angular.x 承载递增序号，仅用于让仿真侧判断“发布方是否还活着”；
 约定里 angular.x 必须为 0，因此心跳走独立话题，不污染 /cmd_vel）。

阶段结束时间写进 phases.csv，供 e06_analyze.py 对齐。
"""

import argparse
import csv
import os
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rosgraph_msgs.msg import Clock

ap = argparse.ArgumentParser()
ap.add_argument("--out", required=True)
ap.add_argument("--rate", type=float, default=20.0, help="指令发布频率 Hz")
args, _ = ap.parse_known_args()
OUT = os.path.abspath(args.out)
os.makedirs(OUT, exist_ok=True)

SETTLE = 2.0      # 开头静置
HOLD = 3.0        # 每个动作保持
GAP = 1.0         # 阶段间隔（发零速）

# (名称, vx, vy, wz, 保持时间)
PHASES = [
    ("forward_+0.1", 0.10, 0.0, 0.0, HOLD),
    ("backward_-0.1", -0.10, 0.0, 0.0, HOLD),
    ("left_+0.1", 0.0, 0.10, 0.0, HOLD),
    ("right_-0.1", 0.0, -0.10, 0.0, HOLD),
    ("ccw_+0.3", 0.0, 0.0, 0.30, HOLD),
    ("cw_-0.3", 0.0, 0.0, -0.30, HOLD),
    ("combined", 0.10, 0.05, 0.20, HOLD),
    ("zero_cmd", 0.0, 0.0, 0.0, 1.5),
    ("cmd_drop", 0.10, 0.0, 0.0, 1.5),   # 之后停发 -> 期待超时停车
]
TIMEOUT_WAIT = 2.5

rclpy.init()
node = Node("e06_motion_test")
node.set_parameters([rclpy.parameter.Parameter("use_sim_time", rclpy.parameter.Parameter.Type.BOOL, True)])
pub_cmd = node.create_publisher(Twist, "/cmd_vel", 10)
pub_beat = node.create_publisher(Twist, "/e06/heartbeat", 10)
clk = {"t": 0.0}
node.create_subscription(Clock, "/clock", lambda m: clk.__setitem__("t", m.clock.sec + m.clock.nanosec * 1e-9), 10)


def spin(seconds=None, until=None):
    end = None if until is not None else time.time() + (seconds or 0.0)
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.01)
        if until is not None and clk["t"] >= until:
            return
        if end is not None and time.time() >= end:
            return


def publish(vx, vy, wz, seq):
    t = Twist()
    t.linear.x, t.linear.y, t.angular.z = float(vx), float(vy), float(wz)
    pub_cmd.publish(t)
    b = Twist()
    b.angular.x = float(seq)          # 心跳序号
    pub_beat.publish(b)


print("[E06-EXT] 等待 /clock ...", flush=True)
deadline = time.time() + 180
while clk["t"] <= 0.0 and time.time() < deadline:
    rclpy.spin_once(node, timeout_sec=0.05)
print("[E06-EXT] /clock 已就绪 t=%.3f" % clk["t"], flush=True)

f = open(os.path.join(OUT, "phases.csv"), "w", newline="")
w = csv.writer(f)
w.writerow(["phase", "name", "t_start", "t_end", "cmd_vx", "cmd_vy", "cmd_wz", "note"])

seq = 0
t_start = clk["t"]


phase_no = {"n": 0}


def run_phase(name, vx, vy, wz, hold, note=""):
    """在仿真时间里保持一段时间：按 20 Hz 发指令 + 心跳。"""
    global seq
    t0 = clk["t"]
    next_pub = 0.0
    while clk["t"] - t0 < hold:
        rclpy.spin_once(node, timeout_sec=0.01)
        if clk["t"] >= next_pub:
            seq += 1
            publish(vx, vy, wz, seq)
            next_pub = clk["t"] + 1.0 / args.rate
    phase_no["n"] += 1
    w.writerow([phase_no["n"], name, round(t0, 3), round(clk["t"], 3), vx, vy, wz, note])
    f.flush()
    print("[E06-EXT] phase %-14s t=%.2f..%.2f" % (name, t0, clk["t"]), flush=True)


phase_no["n"] += 1
w.writerow([phase_no["n"], "settle", round(t_start, 3), round(t_start + SETTLE, 3), 0, 0, 0, "开机静置"])
run_phase("settle", 0.0, 0.0, 0.0, SETTLE)

for name, vx, vy, wz, hold in PHASES:
    if name == "cmd_drop":
        run_phase(name, vx, vy, wz, hold, note="之后停止发布 cmd 与心跳，期待超时停车")
        t_stop = clk["t"]
        # 停止发布：只 spin，不回任何指令
        while clk["t"] - t_stop < TIMEOUT_WAIT:
            rclpy.spin_once(node, timeout_sec=0.02)
        phase_no["n"] += 1
        w.writerow([phase_no["n"], "cmd_drop_wait", round(t_stop, 3), round(clk["t"], 3),
                    0, 0, 0, "已停发，观察是否在 0.5 s 内停车"])
        f.flush()
        print("[E06-EXT] 停发后静默 %.2f s" % (clk["t"] - t_stop), flush=True)
        break
    run_phase(name, vx, vy, wz, hold)
    run_phase(name + "_gap", 0.0, 0.0, 0.0, GAP, note="阶段间隔零速")

f.close()
node.destroy_node()
rclpy.shutdown()
print("[E06-EXT] 完成，阶段写入 %s/phases.csv" % OUT, flush=True)
