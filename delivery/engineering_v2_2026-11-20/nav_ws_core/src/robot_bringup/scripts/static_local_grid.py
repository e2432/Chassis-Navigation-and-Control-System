#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""/debug/static_local_grid —— 局部窗口的**纯静态**栅格（给算法侧建 EDT）。

为什么需要它（算法侧 G6 / A-P0-3a）：
  * 局部代价地图的**主栅格**里既有 inflation 代价、又有 `PedCorridorLayer` 写进去的
    "道口栏杆 band"；算法侧用 `/mpc/obstacles` 拿精确栏杆矩形时，若再用主栅格建 EDT，
    同一根栏杆会被算两遍（重复计数）。
  * `inflation` 输出的是"代价场"，不是几何距离，不能直接当 EDT 用。
  * 所以这里另发一路**只有静态几何**的栅格：从 `/map`（map_server 的 transient_local 地图）
    按机器人当前位置裁一个 `width_m × height_m` 的窗口，只做裁剪，不做膨胀、不加 band。

话题 / 口径（正式 A 口径用这一路）：
  * 输入：`/map`（`nav_msgs/OccupancyGrid`，transient_local + reliable，一次性拿到）、
          `/odom`（`nav_msgs/Odometry`）只用来取机器人 (x, y)。
  * 输出：`/debug/static_local_grid`（`nav_msgs/OccupancyGrid`），
          `frame_id=map`、`resolution=0.05`、`width=height=120`（6 m × 6 m），
          **origin = 机器人 − 半窗**，与局部 costmap 的滚动窗口同几何（可直接对齐索引）。
  * 频率：10 Hz（与局部 costmap 的 update_frequency 同拍）。
  * 取值：0 = 空闲、100 = 静态占用（墙/圆障碍）、-1 = 地图外（未知，由算法侧自行决定如何处理）。

用法：
    ros2 run robot_bringup static_local_grid.py
    # 或由 launch 打开：ros2 launch robot_bringup nav2_baseline.launch.py \
    #     baseline:=mpc static_local_grid:=true
"""

from __future__ import annotations

import math
import sys

import rclpy
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy


class StaticLocalGrid(Node):
    def __init__(self) -> None:
        super().__init__("static_local_grid")
        self.declare_parameter("map_topic", "/map")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("out_topic", "/debug/static_local_grid")
        self.declare_parameter("width_m", 6.0)
        self.declare_parameter("height_m", 6.0)
        self.declare_parameter("resolution", 0.05)
        self.declare_parameter("publish_rate", 10.0)
        self.declare_parameter("frame_id", "map")

        p = self.get_parameter
        self.res = float(p("resolution").value)
        self.w = int(round(float(p("width_m").value) / self.res))
        self.h = int(round(float(p("height_m").value) / self.res))
        self.frame_id = str(p("frame_id").value)
        self.robot = None                     # (x, y)
        self.map_msg = None

        map_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(OccupancyGrid, str(p("map_topic").value), self._on_map, map_qos)
        self.create_subscription(Odometry, str(p("odom_topic").value), self._on_odom, 10)
        self.pub = self.create_publisher(OccupancyGrid, str(p("out_topic").value), 10)

        rate = max(1.0, float(p("publish_rate").value))
        self.create_timer(1.0 / rate, self._tick)
        self._n = 0
        self.get_logger().info(
            "静态局部栅格：%dx%d @%.3f m（窗口 %.1f x %.1f m）→ %s，%.1f Hz；"
            "只裁剪 /map，不做 inflation、不加道口栏杆"
            % (self.w, self.h, self.res, self.w * self.res, self.h * self.res,
               str(p("out_topic").value), rate))

    def _on_map(self, msg: OccupancyGrid) -> None:
        self.map_msg = msg
        self.get_logger().info("收到 /map：%dx%d res=%.3f 原点(%.2f,%.2f)"
                               % (msg.info.width, msg.info.height, msg.info.resolution,
                                  msg.info.origin.position.x, msg.info.origin.position.y))

    def _on_odom(self, msg: Odometry) -> None:
        self.robot = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    def _tick(self) -> None:
        if self.map_msg is None or self.robot is None:
            return
        gm = self.map_msg
        gi = gm.info
        if gi.width <= 0 or gi.height <= 0 or gi.resolution <= 0:
            return
        # 窗口原点与局部 costmap 一致：机器人居中（向下取整到栅格）
        ox = self.robot[0] - 0.5 * self.w * self.res
        oy = self.robot[1] - 0.5 * self.h * self.res

        out = OccupancyGrid()
        out.header.frame_id = self.frame_id
        out.header.stamp = self.get_clock().now().to_msg()
        out.info.resolution = self.res
        out.info.width = self.w
        out.info.height = self.h
        out.info.origin.position.x = ox
        out.info.origin.position.y = oy
        out.info.origin.orientation.w = 1.0

        data = [-1] * (self.w * self.h)
        # 世界坐标 → 局部索引；只做"最近邻"采样，几何边界误差 ≤ 半个栅格
        for jy in range(self.h):
            wy = oy + (jy + 0.5) * self.res
            gy = int(math.floor((wy - gi.origin.position.y) / gi.resolution))
            if gy < 0 or gy >= gi.height:
                continue
            for jx in range(self.w):
                wx = ox + (jx + 0.5) * self.res
                gx = int(math.floor((wx - gi.origin.position.x) / gi.resolution))
                if gx < 0 or gx >= gi.width:
                    continue
                data[jy * self.w + jx] = int(gm.data[gy * gi.width + gx])
        out.data = data
        self.pub.publish(out)

        self._n += 1
        if self._n % 100 == 0:
            occ = sum(1 for v in data if v >= 50)
            unk = sum(1 for v in data if v < 0)
            self.get_logger().info("已发 %d 帧；占用 %d 格、未知 %d 格、机器人(%.2f,%.2f)"
                                   % (self._n, occ, unk, self.robot[0], self.robot[1]))


def main(argv=None) -> int:
    rclpy.init()
    node = StaticLocalGrid()
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
    sys.exit(main())
