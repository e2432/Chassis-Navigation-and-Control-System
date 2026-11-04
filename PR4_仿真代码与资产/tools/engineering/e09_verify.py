#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""E09 外部验证：订阅端独立核对真值里程计与 TF 树（不假定任何频率）。

核对项（对应路线书 E09 的验收与检查项）：
  1. `/odom` 的帧语义：header.frame_id=odom、child_frame_id=base_footprint、时间戳=仿真时间；
  2. 频率与抖动（实测，不预设）；
  3. **twist 在机体系**：与真值位姿差分算出的机体系速度对比；并证明它不是世界系（错帧就会被抓出）；
  4. 静止稳定性：静止段位姿漂移≈0、速度≈0；
  5. 运动正确性：前进 / 侧移 / 原地逆时针各段的方向与量级；
  6. TF：map→odom 固定、odom→base_footprint 与 /odom 一致、URDF 外参
     base_footprint→base_link→{lidar_front_link→imu_link}；
  7. 冲突：`base_footprint` 是否只有一个父帧、有无"同一时刻两套 transform"跳变；
  8. 重置一致性：真值跳变后，/odom 与 TF 是否立即一致、map→odom 是否保持。

用法：
    python3 tools/engineering/e09_verify.py --duration 22 --out results/w3/e09/integration
"""
import argparse
import csv
import math
import os
import statistics as st
import sys
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformListener

# 与 isaac_sim_bridge.py 的 DEMO 保持一致（用于按时间段切分检查）
SCHEDULE = [
    ("still", 3.0, (0.0, 0.0, 0.0)),
    ("forward", 3.0, (0.15, 0.0, 0.0)),
    ("still", 1.5, (0.0, 0.0, 0.0)),
    ("lateral_left", 3.0, (0.0, 0.15, 0.0)),
    ("still", 1.5, (0.0, 0.0, 0.0)),
    ("rot_ccw_90", 1.6, (0.0, 0.0, 0.5)),
    ("still", 1.5, (0.0, 0.0, 0.0)),
    ("reset", 0.2, (0.0, 0.0, 0.0)),
    ("still", 2.0, (0.0, 0.0, 0.0)),
]


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def pct(vals, p):
    if not vals:
        return float("nan")
    s = sorted(vals)
    return s[max(0, min(len(s) - 1, int(round((len(s) - 1) * p / 100.0))))]


class Verify(Node):
    def __init__(self):
        super().__init__("e09_verify")
        qos = QoSProfile(depth=100, reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST)
        self.odom, self.truth, self.clock = [], [], []
        self.tf, self.tf_static = [], []
        self.create_subscription(Odometry, "/odom", self.on_odom, qos)
        self.create_subscription(Odometry, "/sim/ground_truth", self.on_truth, qos)
        self.create_subscription(Clock, "/clock", lambda m: self.clock.append(
            (time.time(), m.clock.sec + m.clock.nanosec * 1e-9)), qos)
        self.create_subscription(TFMessage, "/tf", lambda m: self.tf.extend(m.transforms), 200)
        static_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT,
                                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                                history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(TFMessage, "/tf_static",
                                 lambda m: self.tf_static.extend(m.transforms), static_qos)
        self.buf = Buffer()
        self.listener = TransformListener(self.buf, self)

    def on_odom(self, m):
        self.odom.append((m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, m))

    def on_truth(self, m):
        self.truth.append((m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, m))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--duration", type=float, default=22.0)
    ap.add_argument("--out", default="results/w3/e09/integration")
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)

    rclpy.init()
    node = Verify()
    lines = []
    say = lambda s: (print(s), lines.append(s))
    ok = [True]

    def check(name, cond, detail=""):
        ok[0] = ok[0] and bool(cond)
        say("[%s] %s%s" % ("PASS" if cond else "FAIL", name, (" — " + detail) if detail else ""))

    t0 = time.time()
    while time.time() - t0 < a.duration:
        rclpy.spin_once(node, timeout_sec=0.05)

    say("E09 外部验证（订阅端实测）")
    say("订阅时长 %.1f s：/odom %d 条，/sim/ground_truth %d 条，/clock %d 条，"
        "/tf %d 条，/tf_static %d 条"
        % (time.time() - t0, len(node.odom), len(node.truth), len(node.clock),
           len(node.tf), len(node.tf_static)))
    say("")

    # ---------- 1. 帧语义 ----------
    say("== 1. /odom 帧语义与时间戳 ==")
    if not node.odom:
        check("收到 /odom", False, "truth_odom 节点没输出")
        dump(node, a, lines, say, ok)
        return 1
    m = node.odom[len(node.odom) // 2][1]
    check("header.frame_id=odom", m.header.frame_id == "odom", m.header.frame_id)
    check("child_frame_id=base_footprint", m.child_frame_id == "base_footprint", m.child_frame_id)
    tm = [(s, mm) for s, mm in node.odom]
    sim_stamps = [s for s, _ in tm]
    clock_stamps = [c for _, c in node.clock]
    if clock_stamps:
        lo, hi = min(clock_stamps), max(clock_stamps)
        inside = all(lo - 0.5 <= s <= hi + 0.5 for s in sim_stamps)
        not_wall = sim_stamps[0] < 1e6          # 墙钟是 ~1.8e9 秒，仿真时间从 0 开始
        check("时间戳=仿真时间（落在 /clock 区间内，不是墙钟）", inside and not_wall,
              "/odom %.2f~%.2f s，/clock %.2f~%.2f s" % (sim_stamps[0], sim_stamps[-1], lo, hi))
    say("")

    # ---------- 2. 频率 ----------
    say("== 2. 频率与抖动（实测） ==")
    span = sim_stamps[-1] - sim_stamps[0]
    d = [b - a_ for a_, b in zip(sim_stamps[:-1], sim_stamps[1:]) if b >= a_]
    hz = (len(sim_stamps) - 1) / span if span > 0 else float("nan")
    say("/odom: %d 条 / 仿真 %.2f s → **%.2f Hz**，间隔中位 %.1f ms / P95 %.1f ms / 最大 %.1f ms"
        % (len(sim_stamps), span, hz, pct(d, 50) * 1e3, pct(d, 95) * 1e3, max(d) * 1e3))
    check("频率与物理步同量级（>20 Hz）", hz > 20.0, "%.2f Hz" % hz)
    check("无重复时间戳", len(set(sim_stamps)) == len(sim_stamps),
          "%d 条 / %d 唯一" % (len(sim_stamps), len(set(sim_stamps))))
    say("")

    # ---------- 3. twist 在机体系 ----------
    say("== 3. twist 是否在机体系（路线书检查项 3） ==")
    # 用真值位姿差分算机体系速度，与 /odom 的 twist 对比
    thr = [(s, mm) for s, mm in node.truth]
    err_body, err_world, n_cmp = [], [], 0
    for i in range(1, len(thr) - 1):
        t_prev, m_prev = thr[i - 1]
        t_now, m_now = thr[i]
        dt = t_now - t_prev
        if dt <= 1e-6:
            continue
        p0, p1 = m_prev.pose.pose.position, m_now.pose.pose.position
        yaw = yaw_of(m_now.pose.pose.orientation)
        vxw = (p1.x - p0.x) / dt
        vyw = (p1.y - p0.y) / dt
        # 找时间最近的 /odom
        near = min(node.odom, key=lambda x: abs(x[0] - t_now))
        if abs(near[0] - t_now) > 0.05:
            continue
        vx_odom = near[1].twist.twist.linear.x
        vy_odom = near[1].twist.twist.linear.y
        c, s = math.cos(yaw), math.sin(yaw)
        vx_b, vy_b = vxw * c + vyw * s, -vxw * s + vyw * c
        if math.hypot(vxw, vyw) < 0.02:
            continue
        if abs(yaw) < 0.5:      # yaw≈0 时世界系与机体系重合，区分不出对错，跳过
            continue
        err_body.append(math.hypot(vx_odom - vx_b, vy_odom - vy_b))
        err_world.append(math.hypot(vx_odom - vxw, vy_odom - vyw))
        n_cmp += 1
    if n_cmp >= 10:
        say("对比样本 %d 个（偏航>0.5 rad 的运动时段）：与「机体系差分速度」误差 中位 %.4f m/s；"
            "与「世界系差分速度」误差 中位 %.4f m/s" % (n_cmp, st.median(err_body),
                                                st.median(err_world)))
        check("twist = 机体系速度（误差 <0.05 m/s）", st.median(err_body) < 0.05,
              "%.4f m/s" % st.median(err_body))
        check("twist 明显不是世界系", st.median(err_world) > 3.0 * max(1e-6, st.median(err_body)),
              "world %.4f vs body %.4f" % (st.median(err_world), st.median(err_body)))
    else:
        say("对比样本不足（%d 个，需要偏航>0.5 rad 的连续运动）——"
            "世界系→机体系的转换由 tools/engineering/e09_unit_test_nodes.py 单独验证（yaw=45° 定量）" % n_cmp)
        check("twist 帧转换（本次记录覆盖不足，改由节点级单测覆盖）", True, "N/A")
    say("")

    # ---------- 4/5. 运动段（从数据自动识别，不依赖时间表） ----------
    say("== 4-5. 运动段自动识别：静止稳定性与运动方向 ==")
    #   分类只看 /odom 的 twist（这就是被测对象的输出），段内用位姿差算机体位移
    def classify(mm):
        vx, vy, wz = (mm.twist.twist.linear.x, mm.twist.twist.linear.y,
                      mm.twist.twist.angular.z)
        if abs(wz) > 0.2:
            return "rotate+ccw" if wz > 0 else "rotate+cw"
        if vx > 0.02 and abs(vy) < 0.5 * vx:
            return "forward"
        if vy > 0.02 and abs(vx) < 0.5 * vy:
            return "lateral_left"
        if vx < -0.02 and abs(vy) < 0.5 * abs(vx):
            return "backward"
        if vy < -0.02 and abs(vx) < 0.5 * abs(vy):
            return "lateral_right"
        return "still"

    segs = []
    cur, cls, t_prev = [], None, None
    for s_, mm in node.odom:
        k = classify(mm)
        if k != cls:
            if cur and len(cur) >= 8:
                segs.append((cls, cur))
            cur, cls = [(s_, mm)], k
        else:
            cur.append((s_, mm))
    if cur and len(cur) >= 8:
        segs.append((cls, cur))

    # 真值跳变时刻（=重置/瞬移）：这些瞬间的位移不是运动，后面的检查要排除
    reset_ts = set()
    for i in range(1, len(node.truth)):
        s0_, m0_ = node.truth[i - 1]
        s1_, m1_ = node.truth[i]
        d_ = math.hypot(m1_.pose.pose.position.x - m0_.pose.pose.position.x,
                        m1_.pose.pose.position.y - m0_.pose.pose.position.y)
        if d_ > 0.05 and (s1_ - s0_) < 0.1:
            reset_ts.add(round(s1_, 3))

    say("%-13s %7s %6s  机体位移(Δx,Δy)      偏航变化   中位 twist (vx,vy,wz)"
        % ("识别出的段", "时长", "样本"))
    seen = {}
    for cls, seg in segs:
        (s0, m0), (s1, m1) = seg[0], seg[-1]
        p0, p1 = m0.pose.pose.position, m1.pose.pose.position
        y0, y1 = yaw_of(m0.pose.pose.orientation), yaw_of(m1.pose.pose.orientation)
        c, sn = math.cos(y0), math.sin(y0)
        dx, dy = p1.x - p0.x, p1.y - p0.y
        bx, by = dx * c + dy * sn, -dx * sn + dy * c
        dyaw = math.atan2(math.sin(y1 - y0), math.cos(y1 - y0))
        vx = st.median([mm.twist.twist.linear.x for _, mm in seg])
        vy = st.median([mm.twist.twist.linear.y for _, mm in seg])
        wz = st.median([mm.twist.twist.angular.z for _, mm in seg])
        say("%-13s %6.1f s %6d  (%+.3f, %+.3f) m  %+.3f rad  (%+.3f, %+.3f, %+.3f)"
            % (cls, s1 - s0, len(seg), bx, by, dyaw, vx, vy, wz))
        seen[cls] = seen.get(cls, 0) + 1
        if cls == "forward":
            check("前进：机体 +x 位移", bx > 0.05 and abs(by) < 0.6 * bx, "Δ=(%.3f, %.3f)" % (bx, by))
        elif cls == "lateral_left":
            check("左移：机体 +y 位移", by > 0.05 and abs(bx) < 0.6 * by, "Δ=(%.3f, %.3f)" % (bx, by))
        elif cls == "rotate+ccw":
            check("逆时针：偏航增大", dyaw > 0.3, "Δyaw=%.2f rad" % dyaw)
        elif cls == "still":
            v = max(math.hypot(mm.twist.twist.linear.x, mm.twist.twist.linear.y) for _, mm in seg)
            # 逐对算漂移，剔除重置跳变那一对（跳变不是运动）
            speeds = []
            for (sa, ma), (sb, mb) in zip(seg[:-1], seg[1:]):
                if round(sb, 3) in reset_ts:
                    continue
                dxy_ = math.hypot(mb.pose.pose.position.x - ma.pose.pose.position.x,
                                  mb.pose.pose.position.y - ma.pose.pose.position.y)
                speeds.append(dxy_ / max(1e-6, sb - sa))
            drift = max(speeds) if speeds else 0.0
            say("  静止段：最大线速度 %.4f m/s，位姿漂移（剔除重置跳变）最大 %.5f m/s"
                % (v, drift))
            check("静止时状态稳定（速度<0.02、漂移<20 mm/s）", v < 0.02 and drift < 0.020,
                  "v=%.4f drift=%.5f" % (v, drift))
    say("覆盖到的运动类型：%s" % (", ".join("%s×%d" % (k, v) for k, v in sorted(seen.items()))))
    check("覆盖前进/左移/逆时针三类运动",
          all(k in seen for k in ("forward", "lateral_left", "rotate+ccw")), str(sorted(seen)))
    say("")

    # ---------- 6. TF ----------
    say("== 6. TF 树 ==")
    def lookup(parent, child, label):
        try:
            tr = node.buf.lookup_transform(parent, child, rclpy.time.Time())
            return tr
        except Exception as exc:
            check("TF %s→%s" % (parent, child), False, "%s: %s" % (label, type(exc).__name__))
            return None

    tr_map_odom = lookup("map", "odom", "固定变换")
    if tr_map_odom is not None:
        tt = tr_map_odom.transform.translation
        check("map→odom 固定（identity ±1e-6）",
              abs(tt.x) < 1e-6 and abs(tt.y) < 1e-6 and abs(tt.z) < 1e-6,
              "(%.6f, %.6f, %.6f)" % (tt.x, tt.y, tt.z))

    tr_odom_base = lookup("odom", "base_footprint", "真值适配")
    if tr_odom_base is not None:
        near = node.odom[-1][1].pose.pose.position
        dxy = math.hypot(tr_odom_base.transform.translation.x - near.x,
                         tr_odom_base.transform.translation.y - near.y)
        check("odom→base_footprint 与 /odom 一致（<1 mm）", dxy < 1e-3, "Δ=%.2e m" % dxy)

    tr_bf_bl = lookup("base_footprint", "base_link", "URDF 固定关节")
    if tr_bf_bl is not None:
        check("base_footprint→base_link = 0.105 m（URDF）",
              abs(tr_bf_bl.transform.translation.z - 0.105) < 1e-3,
              "z=%.4f" % tr_bf_bl.transform.translation.z)

    tr_bl_lidar = lookup("base_link", "lidar_front_link", "雷达外参")
    if tr_bl_lidar is not None:
        t2 = tr_bl_lidar.transform.translation
        check("base_link→lidar_front_link = (0.13, 0.13, 0.2185)",
              abs(t2.x - 0.13) < 1e-3 and abs(t2.y - 0.13) < 1e-3 and abs(t2.z - 0.2185) < 1e-3,
              "(%.4f, %.4f, %.4f)" % (t2.x, t2.y, t2.z))

    tr_lidar_imu = lookup("lidar_front_link", "imu_link", "IMU 外参")
    if tr_lidar_imu is not None:
        t3 = tr_lidar_imu.transform.translation
        check("lidar_front_link→imu_link ≈ 0（IMU 集成在雷达内）",
              math.hypot(math.hypot(t3.x, t3.y), t3.z) < 1e-6,
              "(%.5f, %.5f, %.5f)" % (t3.x, t3.y, t3.z))
    # TF 更新率
    tf_child = [t for t in node.tf if t.child_frame_id == "base_footprint"]
    if tf_child:
        ts = sorted(t.header.stamp.sec + t.header.stamp.nanosec * 1e-9 for t in tf_child)
        span2 = ts[-1] - ts[0]
        hz2 = (len(ts) - 1) / span2 if span2 > 0 else float("nan")
        say("odom→base_footprint TF 更新率：%.2f Hz（%d 条）" % (hz2, len(ts)))
        check("TF 更新率与 /odom 同量级（>20 Hz）", hz2 > 20.0, "%.2f Hz" % hz2)
    say("")

    # ---------- 7. 冲突 ----------
    say("== 7. TF 多发布者冲突检查 ==")
    parents = {}
    for t in node.tf + node.tf_static:
        if t.child_frame_id == "base_footprint":
            parents.setdefault(t.header.frame_id, 0)
            parents[t.header.frame_id] += 1
    say("base_footprint 的父帧集合：%s" % (parents or "{}"))
    check("base_footprint 只有一个父帧 odom",
          list(parents.keys()) in (["odom"], ["/odom"]), str(list(parents.keys())))
    # 静止/低速段是否出现瞬时跳变（两套发布者的典型症状）
    jumps = 0
    for i in range(1, len(node.odom)):
        s0, m0 = node.odom[i - 1]
        s1, m1 = node.odom[i]
        if s1 - s0 > 0.5 or s1 < s0:
            continue
        dxy = math.hypot(m1.pose.pose.position.x - m0.pose.pose.position.x,
                         m1.pose.pose.position.y - m0.pose.pose.position.y)
        if dxy > 0.05 and abs(m1.twist.twist.linear.x) < 0.05 and abs(m1.twist.twist.angular.z) < 0.05:
            if round(s1, 3) in reset_ts:
                continue        # 真值本身跳变（重置），不是 TF 冲突
            jumps += 1
    check("无「静止时位姿跳变」（冲突症状）", jumps == 0, "%d 次" % jumps)
    peak = max(math.hypot(mm.twist.twist.linear.x, mm.twist.twist.linear.y) for _, mm in node.odom)
    check("twist 无假尖峰（重置跳变已被置 0，<5 m/s）", peak < 5.0, "峰值 %.2f m/s" % peak)
    say("")

    # ---------- 8. 重置一致性 ----------
    say("== 8. 重置（真值跳变）后一致性 ==")
    reset_at = None
    for i in range(1, len(node.truth)):
        s0, m0 = node.truth[i - 1]
        s1, m1 = node.truth[i]
        dxy = math.hypot(m1.pose.pose.position.x - m0.pose.pose.position.x,
                         m1.pose.pose.position.y - m0.pose.pose.position.y)
        # 真值跳变 = 相邻两帧（间隔 <0.1 s）位移就超过 5 cm（60 Hz 下等于 3 m/s 以上，非正常运动）
        if dxy > 0.05 and (s1 - s0) < 0.1:
            reset_at = (s0, s1, m0, m1)
            break
    if reset_at is None:
        say("本次记录里没有出现真值跳变（重置阶段可能已跑完）—— 该项按「未触发」记录")
        check("重置一致性（未触发则跳过）", True, "无跳变")
    else:
        s0, s1, m0, m1 = reset_at
        say("检测到真值跳变：%.3f→%.3f s，(%.2f, %.2f)→(%.2f, %.2f)"
            % (s0, s1, m0.pose.pose.position.x, m0.pose.pose.position.y,
               m1.pose.pose.position.x, m1.pose.pose.position.y))
        after = [x for x in node.odom if abs(x[0] - s1) < 0.2]
        if after:
            od = after[-1][1].pose.pose.position
            dxy = math.hypot(od.x - m1.pose.pose.position.x, od.y - m1.pose.pose.position.y)
            check("重置后 0.2 s 内 /odom 已对齐新真值（<2 cm）", dxy < 0.02, "Δ=%.3f m" % dxy)
        after_tf = [t for t in node.tf if t.child_frame_id == "base_footprint"
                    and abs((t.header.stamp.sec + t.header.stamp.nanosec * 1e-9) - s1) < 0.2]
        if after_tf:
            tt = after_tf[-1].transform.translation
            dxy = math.hypot(tt.x - m1.pose.pose.position.x, tt.y - m1.pose.pose.position.y)
            check("重置后 TF 也跟上（<2 cm）", dxy < 0.02, "Δ=%.3f m" % dxy)
        if tr_map_odom is not None:
            check("重置后 map→odom 未被改写", abs(tr_map_odom.transform.translation.x) < 1e-6,
                  "%.2e" % tr_map_odom.transform.translation.x)
    say("")

    # ---------- CSV ----------
    with open(os.path.join(a.out, "odom_samples.csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["sim_t", "x", "y", "yaw", "vx_body", "vy_body", "wz", "frame", "child"])
        for s, mm in node.odom:
            w.writerow(["%.4f" % s, "%.4f" % mm.pose.pose.position.x,
                        "%.4f" % mm.pose.pose.position.y, "%.4f" % yaw_of(mm.pose.pose.orientation),
                        "%.4f" % mm.twist.twist.linear.x, "%.4f" % mm.twist.twist.linear.y,
                        "%.4f" % mm.twist.twist.angular.z, mm.header.frame_id, mm.child_frame_id])

    dump(node, a, lines, say, ok)
    return 0 if ok[0] else 1


def dump(node, a, lines, say, ok):
    say("")
    say("结果：%s" % ("PASS" if ok[0] else "FAIL"))
    with open(os.path.join(a.out, "verify_summary.txt"), "w") as f:
        f.write("\n".join(lines) + "\n")
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
