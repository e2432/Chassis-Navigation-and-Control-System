#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""安全层（M2 套件的最后一道速度闸门）。

速度通路的完整链路（改动后，`/cmd_vel` 仍是底盘唯一订阅）：

    controller_server ──/cmd_vel_raw──▶ flat_speed_limiter ──/cmd_vel_limited──▶
        safety_supervisor ──/cmd_vel──▶ Isaac 桥接（底盘）

两级职责分开，谁负责什么写清楚：
  * `flat_speed_limiter` 负责**契约口径的形状**：平面合速度 ≤0.6、|wz| ≤1.5、加速度限幅。
    四个控制器同等生效，纯几何运算。
  * 本节点负责**安全**：任何一条安全条件成立就把速度清零，并在 `/safety/stop` 上给出
    上升沿（9-1 的 `estop_count` 就是数这个上升沿）。安全层不改控制器的正常输出。

四条安全条件（任一成立即制动）：

  1. `operator`   —— 外部 `/safety/stop_request`（Bool）为真。人工接管。
  2. `odom_stale` —— `/odom` 超过 `odom_timeout_s`（默认 0.5 s）没更新。控制器还在按旧
     位姿算速度时最容易出事故，必须停。
  3. `speed_invariant` —— **只限幅、不制动**。上游指令越过平面合速度/角速度上限时按契约
     口径缩回，并记 `clipped` 计数。为什么不判急停：越限通常来自限速层的数值抖动
     （实测 0.6 m/s 边界上每分钟 20~30 次），把它算成安全事件会让 `estop_count` 全是噪声，
     真正该被记录的"操作员接管/碰撞临近"就被淹没了。
  4. `contact_imminent` —— 点云里有点落在**底盘外接圆 + 余量**之内（默认 0.36 m，
     底盘外接半径 0.343 m）。这是"马上要撞"的判据，正常控制器不会触发；一旦触发就说明
     该 run 是安全层救回来的，必须记进 `estop_count` 而不是当成正常成功。

    注意：雷达近场有 2.63 m 盲区（−7° 下视射线打地），但障碍高于底盘的部分由更大的
    仰角射线覆盖，所以 0.36 m 内的障碍是能被看见的（见 docs/9-1工程执行台账.md §G）。

发布：
  * `/cmd_vel`（Twist）—— 最终下发（底盘只订阅这个）
  * `/safety/stop`（Bool）—— 制动状态（True=正在制动），采集器数上升沿
  * `/safety/status`（std_msgs/String，JSON）—— 诊断：当前触发了哪几条、累计次数

自检（不起 ROS）：python3 safety_supervisor.py --selftest
"""

import argparse
import json
import math
import sys


def clamp_planar(vx: float, vy: float, wz: float, v_max: float, w_max: float):
    """与 flat_speed_limiter 同口径的等比缩放；返回 (vx, vy, wz, 是否被限)。"""
    speed = math.hypot(vx, vy)
    clipped = False
    if speed > v_max > 0.0:
        k = v_max / speed
        vx, vy, clipped = vx * k, vy * k, True
    if w_max > 0.0 and abs(wz) > w_max:
        wz, clipped = math.copysign(w_max, wz), True
    return vx, vy, wz, clipped


def speed_violation(vx: float, vy: float, wz: float, v_max: float, w_max: float,
                    tol: float = 1e-3) -> bool:
    """上游指令是否**违反**上限（不是"超一点点"，而是明确违规）。"""
    return math.hypot(vx, vy) > v_max + tol or abs(wz) > w_max + tol


def min_range_ok(points_xy, guard_radius: float):
    """points_xy 是机体系下的 (x, y)；返回 (是否安全, 最近距离)。"""
    best = float("inf")
    for (x, y) in points_xy:
        d = math.hypot(x, y)
        if d < best:
            best = d
    if best == float("inf"):
        return True, None
    return best >= guard_radius, best


def _selftest() -> int:
    ok = True

    def check(name, cond, detail=""):
        nonlocal ok
        print("[%s] %s %s" % ("PASS" if cond else "FAIL", name, detail))
        ok = ok and cond

    vx, vy, wz, clipped = clamp_planar(0.6, 0.6, 0.0, 0.6, 1.5)
    check("合速度 0.6/0.6 → 缩回 0.6", abs(math.hypot(vx, vy) - 0.6) < 1e-9 and clipped,
          "‖v‖=%.4f" % math.hypot(vx, vy))
    check("未超限不改", clamp_planar(0.2, 0.0, 1.0, 0.6, 1.5)[:3] == (0.2, 0.0, 1.0))
    check("违规判定：0.62 m/s 算违规", speed_violation(0.62, 0.0, 0.0, 0.6, 1.5))
    check("违规判定：0.6 m/s 不算违规", not speed_violation(0.6, 0.0, 0.0, 0.6, 1.5))
    check("违规判定：wz 1.6 算违规", speed_violation(0.0, 0.0, 1.6, 0.6, 1.5))
    safe, d = min_range_ok([(0.5, 0.0), (0.9, 0.2)], 0.36)
    check("碰撞临近：最近 0.51 m 安全", safe and abs(d - math.hypot(0.5, 0.0)) < 1e-9,
          "最近=%.3f" % d)
    safe, d = min_range_ok([(0.30, 0.10)], 0.36)
    check("碰撞临近：0.32 m 触发制动", (not safe) and abs(d - math.hypot(0.3, 0.1)) < 1e-9,
          "最近=%.3f" % d)
    safe, _ = min_range_ok([], 0.36)
    check("碰撞临近：没点时不误报", safe)
    print("结果：%s" % ("PASS" if ok else "FAIL"))
    return 0 if ok else 1


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="M2 安全层：速度闸门 + /safety/stop")
    ap.add_argument("--selftest", action="store_true", help="只跑纯逻辑自检，不起 ROS")
    # 必须 parse_known_args：ros2/launch 会注入 `--ros-args ... -r __node:=... --params-file ...`
    args, _unknown = ap.parse_known_args(argv)
    if args.selftest:
        return _selftest()

    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
    from rclpy.time import Time as RTime
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import PointCloud2
    from std_msgs.msg import Bool, String
    from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException
    from tf2_ros import ExtrapolationException

    class SafetySupervisor(Node):
        def __init__(self):
            super().__init__("safety_supervisor")
            p = self.declare_parameter
            p("input_topic", "/cmd_vel_limited")
            p("output_topic", "/cmd_vel")
            p("stop_topic", "/safety/stop")
            p("request_topic", "/safety/stop_request")
            p("status_topic", "/safety/status")
            p("odom_topic", "/odom")
            p("cloud_topic", "/lidar/points")
            p("base_frame", "base_link")
            p("max_planar_speed", 0.6)
            p("max_wz", 1.5)
            p("odom_timeout_s", 0.5)
            p("guard_enabled", True)
            p("guard_radius_m", 0.36)
            p("release_hold_s", 0.3)
            p("publish_rate", 20.0)

            g = self.get_parameter
            self.in_topic = str(g("input_topic").value)
            self.out_topic = str(g("output_topic").value)
            self.stop_topic = str(g("stop_topic").value)
            self.base_frame = str(g("base_frame").value)
            self.v_max = float(g("max_planar_speed").value)
            self.w_max = float(g("max_wz").value)
            self.odom_timeout = float(g("odom_timeout_s").value)
            self.guard_enabled = bool(g("guard_enabled").value)
            self.guard_radius = float(g("guard_radius_m").value)
            self.release_hold = float(g("release_hold_s").value)
            rate = float(g("publish_rate").value)

            qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             history=HistoryPolicy.KEEP_LAST)
            self.pub = self.create_publisher(Twist, self.out_topic, qos)
            self.pub_stop = self.create_publisher(Bool, self.stop_topic, qos)
            self.pub_status = self.create_publisher(
                String, str(g("status_topic").value),
                QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                           history=HistoryPolicy.KEEP_LAST))

            self.create_subscription(Twist, self.in_topic, self.on_cmd, qos)
            self.create_subscription(Bool, str(g("request_topic").value), self.on_request, qos)
            self.create_subscription(Odometry, str(g("odom_topic").value), self.on_odom, 50)
            self.create_subscription(
                PointCloud2, str(g("cloud_topic").value), self.on_cloud,
                QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT,
                           history=HistoryPolicy.KEEP_LAST))

            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)

            self.cmd = Twist()
            self.request_stop = False
            self.last_odom_t = None
            self.cloud_xy = None            # 机体系点云 xy
            self.cloud_t = None
            self.stopping = False
            self.stop_release_t = None
            self.n_stops = 0
            self.reasons = {}
            self.n_frames = 0
            self.n_clipped = 0
            self.n_tf_fail = 0
            self.stopping_prev = False
            self.reason_counts = {}
            self.speed_violation = False
            self.create_timer(1.0 / max(1.0, rate), self.tick)
            self.get_logger().info(
                "安全层：%s → %s（合速度≤%.2f、|wz|≤%.2f；碰撞临近半径 %.2f m 守卫=%s）"
                % (self.in_topic, self.out_topic, self.v_max, self.w_max,
                   self.guard_radius, self.guard_enabled))

        # ---- 订阅 ----
        def on_cmd(self, msg: Twist):
            self.cmd = msg

        def on_request(self, msg: Bool):
            self.request_stop = bool(msg.data)

        def on_odom(self, msg: Odometry):
            self.last_odom_t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        def on_cloud(self, msg: PointCloud2):
            if not self.guard_enabled:
                return
            stamp = msg.header.stamp
            try:
                tr = self.tf_buffer.lookup_transform(
                    self.base_frame, msg.header.frame_id,
                    RTime(seconds=stamp.sec, nanoseconds=stamp.nanosec))
            except (LookupException, ConnectivityException, ExtrapolationException) as e:
                self.n_tf_fail += 1
                if self.n_tf_fail % 50 == 1:
                    self.get_logger().warn("点云转换失败，碰撞守卫本帧不可用：%s" % e)
                self.cloud_xy = None
                return
            t = tr.transform.translation
            q = tr.transform.rotation
            yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                             1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            c, s = math.cos(yaw), math.sin(yaw)
            step = msg.point_step // 4
            import struct
            raw = msg.data
            pts = []
            for i in range(msg.width * msg.height):
                off = i * msg.point_step
                (x, y, z) = struct.unpack_from("<fff", raw, off)
                # 机体系 z 高于地面才算障碍（雷达会打地，地面回波 z≈-0.32）
                bz = z + t.z
                if bz < -0.05:
                    continue
                pts.append((c * x - s * y + t.x, s * x + c * y + t.y))
            self.cloud_xy = pts
            self.cloud_t = stamp.sec + stamp.nanosec * 1e-9

        # ---- 主循环 ----
        def tick(self):
            self.n_frames += 1
            now = self.get_clock().now().nanoseconds * 1e-9
            reasons = []

            if self.request_stop:
                reasons.append("operator")
            if self.last_odom_t is None or (now - self.last_odom_t) > self.odom_timeout:
                reasons.append("odom_stale")
            vx, vy, wz = self.cmd.linear.x, self.cmd.linear.y, self.cmd.angular.z
            # 越限只限幅，不进入 reasons（见文件头第 3 条）
            self.speed_violation = speed_violation(vx, vy, wz, self.v_max, self.w_max)
            if self.guard_enabled and self.cloud_xy is not None:
                safe, dist = min_range_ok(self.cloud_xy, self.guard_radius)
                if not safe:
                    reasons.append("contact_imminent")

            was_stopping = self.stopping_prev
            for r in reasons:
                self.reason_counts[r] = self.reason_counts.get(r, 0) + 1
            if reasons:
                self.stopping = True
                self.stop_release_t = None
                self.reasons = {r: True for r in reasons}
            elif self.stopping:
                # 条件消失后保持一个很短的时间再放行，避免抖振
                if self.stop_release_t is None:
                    self.stop_release_t = now
                elif (now - self.stop_release_t) >= self.release_hold:
                    self.stopping = False
                    self.reasons = {}

            out = Twist()
            if self.stopping:
                out.linear.x = out.linear.y = out.linear.z = 0.0
                out.angular.x = out.angular.y = out.angular.z = 0.0
                if not was_stopping:
                    self.n_stops += 1
                    self.get_logger().warn("安全层制动：%s" % ",".join(sorted(self.reasons)))
            else:
                cx, cy, cw, clipped = clamp_planar(vx, vy, wz, self.v_max, self.w_max)
                if clipped:
                    self.n_clipped += 1
                out.linear.x, out.linear.y, out.angular.z = cx, cy, cw
            self.pub.publish(out)
            self.pub_stop.publish(Bool(data=self.stopping))
            if self.n_frames % 20 == 0:
                self.pub_status.publish(String(data=json.dumps({
                    "stopping": self.stopping, "reasons": sorted(self.reasons),
                    "stops": self.n_stops, "clipped": self.n_clipped,
                    "guard": self.guard_enabled,
                    "cloud_points": None if self.cloud_xy is None else len(self.cloud_xy),
                    # 各条安全条件累计触发次数（用于判断是"真接近障碍"还是"看门狗误报"）
                    "reason_counts": self.reason_counts,
                    "speed_violation": self.speed_violation,
                    "tf_fail": self.n_tf_fail,
                })))
            self.stopping_prev = self.stopping

    rclpy.init()
    node = SafetySupervisor()
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
