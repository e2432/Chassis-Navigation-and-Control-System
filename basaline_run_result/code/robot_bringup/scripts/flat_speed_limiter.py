#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""共享执行层平面限速（9-1 §2「平面合速度执行口径」）。

为什么需要它：9-1 要求三基线 + MPC 的**平面合速度**都满足 √(vx²+vy²) ≤ 0.6 m/s。
DWB 有原生的 `max_speed_xy`；**MPPI 的 `vx_max`/`vy_max` 是分别约束**（同时接近 0.6 时合速度
可达 ~0.85）；TEB 取决于 fork。因此限速必须落在**共享执行层**，对所有控制器一视同仁：

    Nav2 controller / MPC  ──/cmd_vel_raw──▶  本节点  ──/cmd_vel──▶  底盘桥接
                                                 └── /cmd_vel_limited（镜像，供 Round 0 记录）

限速内容（全部可参数化）：
  1. 平面合速度：‖(vx, vy)‖ ≤ max_planar_speed（等比缩放，保持方向）
  2. 角速度：|wz| ≤ max_wz
  3. 机体系加速度限幅：(vx, vy) 相对上一拍的变化 ≤ max_ax/max_ay · dt；wz ≤ max_alpha · dt
  4. 镜像发布限速后的实际值，便于把"控制器原始输出 / 实际执行"对照写进 Round 0 记录

用法：
    ros2 run robot_bringup flat_speed_limiter.py \
        --ros-args -p input_topic:=/cmd_vel_raw -p output_topic:=/cmd_vel

自测（无 ROS 环境时也可验证限速逻辑）：
    python3 -m robot_bringup.flat_speed_limiter --selftest   # 或直接跑本文件 --selftest
"""

from __future__ import annotations

import math
import sys
from typing import Tuple

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy


def limit_planar(vx: float, vy: float, wz: float,
                 max_planar_speed: float = 0.6, max_wz: float = 1.5) -> Tuple[float, float, float]:
    """按平面合速度等比缩放 + 角速度限幅（纯函数，便于单测）。"""
    speed = math.hypot(vx, vy)
    if speed > max_planar_speed > 0.0:
        k = max_planar_speed / speed
        vx, vy = vx * k, vy * k
    if abs(wz) > max_wz:
        wz = math.copysign(max_wz, wz)
    return vx, vy, wz


def limit_accel(vx: float, vy: float, wz: float, prev: Tuple[float, float, float],
                dt: float, max_ax: float, max_ay: float, max_alpha: float
                ) -> Tuple[float, float, float]:
    """机体系加速度限幅（相对上一拍输出）。"""
    if dt <= 0.0:
        return vx, vy, wz
    px, py, pw = prev

    def _clip(cur: float, old: float, lim: float) -> float:
        dmax = lim * dt
        d = cur - old
        if abs(d) > dmax:
            d = math.copysign(dmax, d)
        return old + d

    return _clip(vx, px, max_ax), _clip(vy, py, max_ay), _clip(wz, pw, max_alpha)


class FlatSpeedLimiter(Node):
    def __init__(self):
        super().__init__("flat_speed_limiter")
        p = self.declare_parameter
        p("input_topic", "/cmd_vel_raw")
        p("output_topic", "/cmd_vel")
        p("record_topic", "/cmd_vel_limited")
        p("max_planar_speed", 0.6)
        p("max_wz", 1.5)
        p("max_ax", 1.0)
        p("max_ay", 1.0)
        p("max_alpha", 2.0)
        p("publish_rate", 20.0)

        g = self.get_parameter
        self.in_topic = str(g("input_topic").value)
        self.out_topic = str(g("output_topic").value)
        self.rec_topic = str(g("record_topic").value)
        self.v_max = float(g("max_planar_speed").value)
        self.w_max = float(g("max_wz").value)
        self.ax = float(g("max_ax").value)
        self.ay = float(g("max_ay").value)
        self.alpha = float(g("max_alpha").value)
        rate = float(g("publish_rate").value)

        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST)
        self.pub = self.create_publisher(Twist, self.out_topic, qos)
        # record_topic 为空表示不再额外发镜像（M2 里 /cmd_vel_limited 本身就是本节点输出，
        # 再镜像一份会在同一话题上出现两个发布者，订阅端会收到重复消息）。
        self.pub_rec = (self.create_publisher(Twist, self.rec_topic, qos)
                        if self.rec_topic else None)
        self.create_subscription(Twist, self.in_topic, self.on_cmd, qos)

        self.last_cmd = Twist()
        self.last_t = None
        self.n_in = 0
        self.n_clipped_speed = 0
        self.n_clipped_accel = 0
        self.peak_raw = 0.0
        self.peak_out = 0.0
        self.create_timer(1.0 / max(1.0, rate), self.tick)
        self.get_logger().info(
            "平面限速：%s → %s（‖(vx,vy)‖≤%.2f m/s，|wz|≤%.2f rad/s；镜像 %s）"
            % (self.in_topic, self.out_topic, self.v_max, self.w_max, self.rec_topic))

    def on_cmd(self, msg: Twist):
        self.n_in += 1
        vx, vy = msg.linear.x, msg.linear.y
        raw_speed = math.hypot(vx, vy)
        self.peak_raw = max(self.peak_raw, raw_speed)
        vx_l, vy_l, wz_l = limit_planar(vx, vy, msg.angular.z, self.v_max, self.w_max)
        if raw_speed > self.v_max + 1e-9:
            self.n_clipped_speed += 1
        now = self.get_clock().now().nanoseconds * 1e-9
        prev = (self.last_cmd.linear.x, self.last_cmd.linear.y, self.last_cmd.angular.z)
        dt = 0.0 if self.last_t is None else max(0.0, now - self.last_t)
        self.last_t = now
        vx_a, vy_a, wz_a = limit_accel(vx_l, vy_l, wz_l, prev, dt, self.ax, self.ay, self.alpha)
        if (abs(vx_a - vx_l) + abs(vy_a - vy_l) + abs(wz_a - wz_l)) > 1e-9:
            self.n_clipped_accel += 1
        self.last_cmd.linear.x, self.last_cmd.linear.y, self.last_cmd.angular.z = vx_a, vy_a, wz_a
        self.peak_out = max(self.peak_out, math.hypot(vx_a, vy_a))

    def tick(self):
        self.pub.publish(self.last_cmd)
        if self.pub_rec is not None:
            self.pub_rec.publish(self.last_cmd)

    def stats(self) -> dict:
        return {"received": self.n_in, "clipped_speed": self.n_clipped_speed,
                "clipped_accel": self.n_clipped_accel,
                "peak_raw_planar_mps": round(self.peak_raw, 4),
                "peak_limited_planar_mps": round(self.peak_out, 4)}


def _selftest() -> int:
    """不依赖 ROS 的自测：验证限速与加速度限幅。"""
    ok = True

    def check(name, cond, detail=""):
        nonlocal ok
        ok = ok and cond
        print("[%s] %s%s" % ("PASS" if cond else "FAIL", name, (" — " + detail) if detail else ""))

    vx, vy, wz = limit_planar(0.6, 0.6, 2.0)
    check("合速度 0.6/0.6 → 缩放回 0.6", abs(math.hypot(vx, vy) - 0.6) < 1e-9,
          "‖v‖=%.4f" % math.hypot(vx, vy))
    check("方向保持（vx≈vy）", abs(vx - vy) < 1e-9 and vx > 0, "(%.4f, %.4f)" % (vx, vy))
    check("角速度限到 1.5", abs(wz - 1.5) < 1e-9, "wz=%.3f" % wz)
    vx2, vy2, _ = limit_planar(0.2, 0.0, 0.0)
    check("未超限不改", (vx2, vy2) == (0.2, 0.0), "(%.2f, %.2f)" % (vx2, vy2))
    ax, ay, aw = limit_accel(1.0, 1.0, 1.0, (0.0, 0.0, 0.0), 0.1, 1.0, 1.0, 2.0)
    check("加速度限幅 0.1 s × 1.0", abs(ax - 0.1) < 1e-9 and abs(ay - 0.1) < 1e-9,
          "(%.3f, %.3f)" % (ax, ay))
    check("角加速度限幅 0.1 s × 2.0", abs(aw - 0.2) < 1e-9, "aw=%.3f" % aw)
    print("结果：%s" % ("PASS" if ok else "FAIL"))
    return 0 if ok else 1


def main(argv=None) -> int:
    if argv is None:
        argv = sys.argv[1:]
    if "--selftest" in argv:
        return _selftest()
    rclpy.init()
    node = FlatSpeedLimiter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info("限速统计：%s" % node.stats())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
