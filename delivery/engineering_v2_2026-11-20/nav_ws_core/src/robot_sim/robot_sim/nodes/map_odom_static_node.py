#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""map→odom 固定变换发布者（E09）。

真值模式（避障基准默认）里 map 与 odom 是同一个坐标系（仿真世界原点 == 地图原点），
所以这条边是 identity（可用参数设成任意固定偏移，例如地图原点不在世界原点时）。

发在 `/tf_static`（transient_local），一次锁定、不占带宽。

切 SLAM/定位模式时：**停用本节点**，由定位/建图节点发动态 map→odom；
E09 验收要求任何时刻只有一套 map→odom（见 readme.md 的互斥表）。
"""

import math
import sys

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from tf2_ros import StaticTransformBroadcaster


class MapOdomStatic(Node):
    def __init__(self):
        super().__init__("map_odom_static")
        d = self.declare_parameter
        d("map_frame", "map")
        d("odom_frame", "odom")
        d("x", 0.0)
        d("y", 0.0)
        d("z", 0.0)
        d("yaw_deg", 0.0)
        p = self.get_parameter
        self.map_frame = str(p("map_frame").value)
        self.odom_frame = str(p("odom_frame").value)
        yaw = math.radians(float(p("yaw_deg").value))

        tf = TransformStamped()
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id = self.map_frame
        tf.child_frame_id = self.odom_frame
        tf.transform.translation.x = float(p("x").value)
        tf.transform.translation.y = float(p("y").value)
        tf.transform.translation.z = float(p("z").value)
        tf.transform.rotation.z = math.sin(yaw / 2.0)
        tf.transform.rotation.w = math.cos(yaw / 2.0)

        self.br = StaticTransformBroadcaster(self)
        self.br.sendTransform(tf)
        self.get_logger().info(
            "已发布静态 TF %s→%s：xyz=(%.3f, %.3f, %.3f) yaw=%.1f°（真值模式，切 SLAM 时停用本节点）"
            % (self.map_frame, self.odom_frame, tf.transform.translation.x,
               tf.transform.translation.y, tf.transform.translation.z,
               float(p("yaw_deg").value)))


def main(argv=None):
    rclpy.init(args=argv)
    node = MapOdomStatic()
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
