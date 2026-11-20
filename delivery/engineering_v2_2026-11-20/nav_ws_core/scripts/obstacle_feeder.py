#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""obstacle_feeder —— 正式口径的 `/mpc/obstacles` 数据源（A 栅格口径）。

背景（2026-11-20 与算法侧对齐）：
  * **A 口径（正式成绩）**：`/mpc/obstacles` **只发"道口栏杆矩形"**，速度恒 0；
    **静态圆障碍与墙体永不进消息**（静态几何由算法侧从局部 costmap 建 EDT，见
    `/debug/static_local_grid`）。
  * **B 口径（A/B 章节）**：改由 `mpc_controller/tracking` 的跟踪器从同一份点云产出行人，
    与本脚本无关（`--mode tracked` 只是占位，不实现）。
  * **QoS 定案**：与插件现状 `rclcpp::QoS(10)`（reliable）对齐 —— **reliable, depth 10**。
    动态数据 20 Hz、单帧几十字节，reliable 在本机 fastrtps 下无压力，且**不改插件代码**。
    需要 best_effort 时用 `--qos best_effort`（此时插件侧订阅也必须改，否则收不到）。

几何来源：订阅采集器发布的 `/ped/corridor`（`nav_msgs/OccupancyGrid`，BEST_EFFORT；
值 100 = 栏杆带内部）。从栅格里把连通块抽出来，每个连通块是一个"栏杆带"矩形；
再用**圆链**近似这个矩形（`ObstacleState` 只有 位置+速度+半径，没有矩形类型）：
沿矩形的长边铺 N 个半径 = 短边一半的圆，间距 0.9 × 直径，保证缝隙 < 半径。
这样算法侧的圆-障碍约束能等价覆盖整条带子，且不会因为"一个巨大的圆"把整片路面堵死。

id 稳定性（契约要求帧间关联）：按连通块的 (x_min, y_min) 排序后编号
`id = 100 * band_index + k`（band_index 从 0 起、k 是带内圆序号）。
只要栏杆带几何不变，id 就不变；抬杆时这些 id 整体消失。

失效语义：
  * 收到空补丁 / 没有新的走廊消息 → 继续按 20 Hz 发**空数组**（心跳语义：插件失联 0.5 s
    就会输出零速，所以必须持续发）。
  * 走廊消息超过 `stale_s`（默认 0.5 s）没更新 → 视为"当前无栏杆"，发空数组（不留 ghost）。
  * 绝不"保留最后位置"。

用法（正式跑由采集器拉起，也可手工起）：
    python3 scripts/obstacle_feeder.py --qos reliable --rate 20
    python3 scripts/obstacle_feeder.py --mode off        # 只在调试时，停发
"""

from __future__ import annotations

import argparse
import math
import sys
import time
from collections import deque
from typing import Deque, List, Tuple

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

try:  # 来自 ASCII 镜像构建的 mpc_controller（source ~/mpc_build_ws/install/setup.bash）
    from mpc_controller.msg import ObstacleState, ObstacleStateArray
except Exception as exc:  # pragma: no cover
    print("[feeder] 无法导入 mpc_controller.msg：%s\n"
          "        请先 bash nav_ws_core/scripts/build_mpc_ascii.sh "
          "&& source ~/mpc_build_ws/install/setup.bash" % exc, file=sys.stderr)
    raise


def components(cells: List[Tuple[int, int]], w: int, h: int) -> List[List[Tuple[int, int]]]:
    """栅格上的 4-连通分量（cell 是 (ix, iy)）。"""
    work = set(cells)
    out: List[List[Tuple[int, int]]] = []
    while work:
        seed = work.pop()
        comp = [seed]
        q: Deque[Tuple[int, int]] = deque([seed])
        while q:
            ix, iy = q.popleft()
            for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                nb = (ix + dx, iy + dy)
                if nb in work:
                    work.discard(nb)
                    comp.append(nb)
                    q.append(nb)
        out.append(comp)
    return out


class ObstacleFeeder(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("obstacle_feeder")
        self.mode = args.mode
        self.frame = args.frame
        self.stale_s = float(args.stale)
        self.band_half = float(args.band_half)     # 栏杆厚度一半（与 PED_HALF_W 一致）
        self.max_obstacles = int(args.max_obstacles)
        self.in_topic = args.in_topic
        self.out_topic = args.out_topic

        self.grid = None            # 最近一帧走廊栅格
        self.grid_t = 0.0
        self.n_recv = 0
        self.n_pub = 0
        self.n_obs = 0

        qos_in = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(OccupancyGrid, self.in_topic, self._on_corridor, qos_in)

        if args.qos == "best_effort":
            qos_out = QoSProfile(depth=int(args.depth), reliability=ReliabilityPolicy.BEST_EFFORT)
        else:
            qos_out = QoSProfile(depth=int(args.depth), reliability=ReliabilityPolicy.RELIABLE)
        self.pub = self.create_publisher(ObstacleStateArray, self.out_topic, qos_out)

        rate = max(1.0, float(args.rate))
        self.create_timer(1.0 / rate, self._tick)
        self.get_logger().info(
            "feeder: mode=%s in=%s out=%s qos=%s depth=%d rate=%.0fHz frame=%s "
            "band_half=%.2f stale=%.2fs（A 口径：只发栏杆矩形，不发静态圆/行人本体）"
            % (self.mode, self.in_topic, self.out_topic, args.qos, int(args.depth), rate,
               self.frame, self.band_half, self.stale_s))

    # ---- 走廊栅格 → 栏杆矩形 → 圆链 ----
    def _on_corridor(self, msg: OccupancyGrid) -> None:
        self.grid = msg
        self.grid_t = time.monotonic()
        self.n_recv += 1

    def _bands(self) -> List[Tuple[float, float, float, float]]:
        """返回 [(cx, cy, half_long, half_short)]，单位 m，世界（map）坐标。"""
        g = self.grid
        if g is None or time.monotonic() - self.grid_t > self.stale_s:
            return []
        gi = g.info
        if gi.width <= 1 or gi.height <= 1 or gi.resolution <= 0:
            return []                      # 采集器的"空补丁"是 1x1
        cells = [(i % gi.width, i // gi.width) for i, v in enumerate(g.data) if v >= 50]
        if not cells:
            return []
        out = []
        for comp in components(cells, gi.width, gi.height):
            xs = [c[0] for c in comp]
            ys = [c[1] for c in comp]
            x0, x1 = min(xs), max(xs)
            y0, y1 = min(ys), max(ys)
            # 连通块很小时（噪声）丢掉
            if (x1 - x0 + 1) * (y1 - y0 + 1) < 4:
                continue
            cx = gi.origin.position.x + (0.5 * (x0 + x1) + 0.5) * gi.resolution
            cy = gi.origin.position.y + (0.5 * (y0 + y1) + 0.5) * gi.resolution
            ex = (x1 - x0 + 1) * gi.resolution
            ey = (y1 - y0 + 1) * gi.resolution
            if ex >= ey:
                out.append((cx, cy, 0.5 * ex, 0.5 * ey))
            else:
                out.append((cx, cy, 0.5 * ey, 0.5 * ex))
        out.sort(key=lambda b: (round(b[0] - b[2], 3), round(b[1] - b[3], 3)))
        return out

    def _tick(self) -> None:
        msg = ObstacleStateArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame

        if self.mode == "gate":
            r = max(0.05, self.band_half)
            step = max(1e-6, 1.8 * r)                  # 圆间距：留 ≤0.8r 的缝
            for bi, (cx, cy, half_long, half_short) in enumerate(self._bands()):
                # 沿长边铺圆；半径用短边一半（= 栏杆厚度一半），夹到 [0.2, 1.0]
                rad = min(max(min(half_short, r), 0.2), 1.0)
                n = max(1, int(math.floor((2.0 * half_long) / step)) + 1)
                for k in range(n):
                    if len(msg.obstacles) >= self.max_obstacles:
                        break
                    t = -half_long + (k + 0.5) * (2.0 * half_long / n)
                    ob = ObstacleState()
                    ob.id = 100 * bi + k
                    ob.position.x = float(cx + t)
                    ob.position.y = float(cy)
                    ob.velocity.x = 0.0                 # A 口径：栏杆静止
                    ob.velocity.y = 0.0
                    ob.radius = float(rad)
                    ob.stamp = msg.header.stamp
                    msg.obstacles.append(ob)

        self.pub.publish(msg)
        self.n_pub += 1
        self.n_obs = len(msg.obstacles)
        if self.n_pub % 200 == 0:                       # 约每 10 s 一次
            self.get_logger().info(
                "feeder 心跳：已发 %d 帧，当前障碍 %d 个（走廊消息 %d 条%s）"
                % (self.n_pub, self.n_obs, self.n_recv,
                   "" if self.grid is not None else "，尚未收到走廊"))

    def report(self) -> dict:
        return {"published": self.n_pub, "obstacles": self.n_obs, "corridor_msgs": self.n_recv}


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="/mpc/obstacles 正式数据源（A 口径：只发栏杆矩形）")
    ap.add_argument("--mode", choices=["gate", "off"], default="gate",
                    help="gate=发栏杆圆链；off=只发空数组（调试用）")
    ap.add_argument("--qos", choices=["reliable", "best_effort"], default="reliable",
                    help="默认 reliable（与插件 rclcpp::QoS(10) 对齐）")
    ap.add_argument("--depth", type=int, default=10)
    ap.add_argument("--rate", type=float, default=20.0, help="发布频率 Hz（与控频同拍）")
    ap.add_argument("--frame", default="odom", help="header.frame_id（契约 v1 = odom）")
    ap.add_argument("--in-topic", default="/ped/corridor")
    ap.add_argument("--out-topic", default="/mpc/obstacles")
    ap.add_argument("--band-half", type=float, default=0.74, help="栏杆厚度一半（PED_HALF_W）")
    ap.add_argument("--stale", type=float, default=0.5, help="走廊消息过期阈值 s")
    ap.add_argument("--max-obstacles", type=int, default=64)
    args, _ = ap.parse_known_args(argv)

    rclpy.init()
    node = ObstacleFeeder(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        r = node.report()
        node.get_logger().info("feeder 退出：%s" % r)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
