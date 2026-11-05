#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""真值适配节点（E09）：把仿真的原始真值转换成合规的 /odom 和唯一一条 odom→base_footprint TF。

输入 `/sim/ground_truth`（`nav_msgs/Odometry`，Isaac 侧发布）：
  * header.frame_id = `world`（仿真世界系）
  * child_frame_id  = `base_footprint`
  * **twist 是世界系速度**（`publishRawVelocities=True`，仿真直接给世界系线速度/角速度）

输出：
  * `/odom`：header.frame_id=`odom`、child_frame_id=`base_footprint`，
    twist 已转换到**机体系**（基向量旋转 R(−yaw)），位姿直接取真值（真值模式不积分）
  * TF `odom→base_footprint`（唯一发布者；启动时自检冲突）

为什么需要这个节点（而不是直接用 bridge 的 odometry 节点）：
  1. 路线书 E09 要求"若仿真返回世界系速度，应转换到约定的机体系"——转换逻辑集中在这里，
     可以被外部脚本独立验证（可与仿真真值差分对比）；
  2. TF 必须只有一套发布者，桥接节点只发话题、不发 TF；
  3. 重置/瞬移时真值会跳变，这里显式记录跳变与仿真时间回退，便于排查"里程计缓存不一致"。

参数（都可在 launch/命令行覆盖）：
  input_topic / output_topic / odom_frame / base_frame / publish_tf
  input_twist_frame: world|body  （默认 world，按仿真侧约定）
  check_conflict:   启动时用 check_conflict_seconds 扫一遍 /tf 与 /tf_static，
                    若发现已有 odom→base_* 的发布者则拒绝发布并退出（防止两套 TF）
"""

import math
import sys

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from tf2_msgs.msg import TFMessage
from tf2_ros import TransformBroadcaster


def quat_to_yaw(x, y, z, w):
    """xyzw → yaw（只取平面偏航；真值模式下 roll/pitch 恒 0）。"""
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


class TruthOdom(Node):
    def __init__(self):
        super().__init__("truth_odom")
        d = self.declare_parameter
        d("input_topic", "/sim/ground_truth")
        d("output_topic", "/odom")
        d("odom_frame", "odom")
        d("base_frame", "base_footprint")
        d("publish_tf", True)
        d("input_twist_frame", "world")
        d("check_conflict", True)
        d("check_conflict_seconds", 1.0)
        d("warn_jump_m", 0.20)
        d("max_plausible_twist_mps", 5.0)

        p = self.get_parameter
        self.in_topic = str(p("input_topic").value)
        self.out_topic = str(p("output_topic").value)
        self.odom_frame = str(p("odom_frame").value)
        self.base_frame = str(p("base_frame").value)
        self.publish_tf = bool(p("publish_tf").value)
        self.twist_frame = str(p("input_twist_frame").value)
        self.warn_jump = float(p("warn_jump_m").value)
        self.max_twist = float(p("max_plausible_twist_mps").value)

        self.qos = QoSProfile(depth=50, reliability=ReliabilityPolicy.RELIABLE,
                              history=HistoryPolicy.KEEP_LAST)
        self.pub = self.create_publisher(Odometry, self.out_topic, 50)
        self.br = TransformBroadcaster(self) if self.publish_tf else None

        self.n_in = 0
        self.n_out = 0
        self.n_resets = 0
        self.n_jumps = 0
        self.last_stamp = None
        self.last_xy = None
        self.conflict_from = None
        self.blocked = False

        if bool(p("check_conflict").value):
            self._check_conflict(float(p("check_conflict_seconds").value))

        if not self.blocked:
            self.create_subscription(Odometry, self.in_topic, self.on_truth, self.qos)
            self.get_logger().info(
                "真值适配已启动：%s（twist=%s 系）→ %s + TF %s→%s；publish_tf=%s"
                % (self.in_topic, self.twist_frame, self.out_topic,
                   self.odom_frame, self.base_frame, self.publish_tf))

    # ---------- 启动前冲突自检 ----------
    def _check_conflict(self, seconds):
        """扫 /tf 与 /tf_static：若已有 odom→base_* 的发布者，就不要再发（避免两套 TF）。"""
        seen = []

        def on_tf(msg):
            for tr in msg.transforms:
                parent = tr.header.frame_id.strip("/")
                child = tr.child_frame_id.strip("/")
                if parent == self.odom_frame and child.startswith(self.base_frame.rstrip("0123456789")):
                    seen.append((parent, child))

        subs = [
            self.create_subscription(TFMessage, "/tf", on_tf, 10),
            self.create_subscription(TFMessage, "/tf_static", on_tf, 10),
        ]
        import time
        t0 = time.time()
        while time.time() - t0 < seconds and not seen:
            rclpy.spin_once(self, timeout_sec=0.05)
        for s in subs:
            self.destroy_subscription(s)
        if seen:
            self.blocked = True
            self.conflict_from = seen[0]
            self.get_logger().error(
                "检测到已有 %s→%s 的发布者（%d 条）。为避免两套 TF，本节点不再发布。"
                "请先停止其他发布相同 TF 的节点。"
                % (seen[0][0], seen[0][1], len(seen)))

    # ---------- 主流程 ----------
    def on_truth(self, msg):
        self.n_in += 1
        stamp = msg.header.stamp
        t = stamp.sec + stamp.nanosec * 1e-9

        # 重置/瞬移检测：仿真时间回退 或 相邻两帧位置大跳
        if self.last_stamp is not None and t + 1e-6 < self.last_stamp:
            self.n_resets += 1
            self.get_logger().warn("仿真时间回退（%.3f → %.3f s）：判定为重置，里程计直接跟新真值"
                                   % (self.last_stamp, t))
        self.last_stamp = t
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        z = msg.pose.pose.position.z
        if self.last_xy is not None:
            jump = math.hypot(x - self.last_xy[0], y - self.last_xy[1])
            if jump > self.warn_jump:
                self.n_jumps += 1
                self.get_logger().warn("真值位姿跳变 %.3f m（重置/瞬移）：真值模式不积分，"
                                       "odom 立刻跟随，无需重新初始化" % jump)
        self.last_xy = (x, y)

        q = msg.pose.pose.orientation
        yaw = quat_to_yaw(q.x, q.y, q.z, q.w)

        # ---- 世界系速度 → 机体系（E09 检查项 3）----
        vx_w = msg.twist.twist.linear.x
        vy_w = msg.twist.twist.linear.y
        wz_w = msg.twist.twist.angular.z
        if self.twist_frame == "world":
            c, s = math.cos(yaw), math.sin(yaw)
            vx_b = vx_w * c + vy_w * s          # R(−yaw)·v_world
            vy_b = -vx_w * s + vy_w * c
            wz_b = wz_w                          # 绕 z 在本节点约定里与偏航一致
        else:
            vx_b, vy_b, wz_b = vx_w, vy_w, wz_w

        out = Odometry()
        out.header.stamp = stamp                 # 与真值同一时间戳（仿真时间）
        out.header.frame_id = self.odom_frame
        out.child_frame_id = self.base_frame
        out.pose.pose.position.x = x
        out.pose.pose.position.y = y
        out.pose.pose.position.z = z
        out.pose.pose.orientation = msg.pose.pose.orientation
        # 重置/瞬移会让"位姿差分速度"出现几十 m/s 的尖峰（真值本身是跳变，不是运动）。
        # 这种样本的 twist 置 0，位姿照常透传——否则下游安全层/控制器会被假速度触发。
        speed = math.hypot(vx_b, vy_b)
        if speed > self.max_twist:
            self.get_logger().warn("twist 超过可信上限（%.1f m/s）：判定为重置/瞬移，本帧 twist 置 0"
                                   % speed)
            vx_b = vy_b = wz_b = 0.0
        out.twist.twist.linear.x = vx_b
        out.twist.twist.linear.y = vy_b
        out.twist.twist.linear.z = 0.0
        out.twist.twist.angular.x = 0.0
        out.twist.twist.angular.y = 0.0
        out.twist.twist.angular.z = wz_b
        # 协方差：真值模式给很小的值（固定、非零，避免下游把 0 当成"未知"）
        for i in range(6):
            out.pose.covariance[i * 6 + i] = 1e-6
            out.twist.covariance[i * 6 + i] = 1e-6
        self.pub.publish(out)
        self.n_out += 1

        if self.br is not None:
            tf = TransformStamped()
            tf.header.stamp = stamp
            tf.header.frame_id = self.odom_frame
            tf.child_frame_id = self.base_frame
            tf.transform.translation.x = x
            tf.transform.translation.y = y
            tf.transform.translation.z = z
            tf.transform.rotation = msg.pose.pose.orientation
            self.br.sendTransform(tf)


def main(argv=None):
    rclpy.init(args=argv)
    node = TruthOdom()
    try:
        if node.blocked:
            node.get_logger().error("已阻塞（存在冲突的 TF 发布者），本节点不发布任何消息")
            return 2
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info("统计：收到真值 %d，发布 /odom %d，重置 %d，跳变 %d"
                               % (node.n_in, node.n_out, node.n_resets, node.n_jumps))
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
