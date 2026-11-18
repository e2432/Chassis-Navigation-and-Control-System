#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""基线 run 的裁判与七项指标（契约 `docs/接口契约.md` v1.4「评测记录 schema」+ `docs/指标定义.md`）。

设计要点：
  * **纯函数、无 ROS 依赖** —— 算法侧可以拿同一份 trajectory.npz 复核我们的判定；
  * 判定口径与指标定义逐条对应（到达 0.25 m / 0.25 rad、时限取实例 time_limit_s、
    碰撞 = 底盘外缘与场景几何间距 ≤ 0、路径长度按采样累加、平滑度按 20 Hz 重采样、
    最小间距 = run 内最小值、急停 = `/safety/stop` 上升沿）；
  * 底盘外形用八边形 footprint（与 costmap 配置一致），间距按"多边形到障碍"的几何距离算，
    不用外接圆近似（外接圆会把最小间距算小、把碰撞算早）。

坐标：一律 **path frame**（与实例表 `coordinate_frame: path` 一致；world_poses 只在 Isaac 预览时应用）。
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence, Tuple

# ---- 契约常量（改这里必须同步 docs/接口契约.md 与 docs/指标定义.md）----
GOAL_POS_TOL_M = 0.25
GOAL_YAW_TOL_RAD = 0.25
TASK_MAX_PLANAR_MPS = 0.6        # 平面合速度上限（limiter 执行层保证）
TASK_MAX_WZ_RADPS = 1.5
CONTROL_HZ = 20.0                # 平滑度重采样频率（契约控制频率）
COLLISION_EPS = 1e-6
ESTOP_BURST_GAP_S = 0.5          # 相邻急停样本间隔小于此值 → 视为同一次急停过程（契约口径）


@dataclass
class Footprint:
    """八边形底盘外形（与 nav2_common.yaml 的 footprint 一致）。"""
    vertices: List[Tuple[float, float]] = field(default_factory=lambda: [
        (0.3215, 0.113), (0.113, 0.328), (-0.113, 0.328), (-0.3215, 0.113),
        (-0.3215, -0.113), (-0.113, -0.328), (0.113, -0.328), (0.3215, -0.113)])

    def world_vertices(self, x: float, y: float, yaw: float) -> List[Tuple[float, float]]:
        c, s = math.cos(yaw), math.sin(yaw)
        return [(x + vx * c - vy * s, y + vx * s + vy * c) for vx, vy in self.vertices]


# --------------------------------------------------------------------------- #
# 几何距离
# --------------------------------------------------------------------------- #
def _point_seg_dist(px: float, py: float, ax: float, ay: float, bx: float, by: float) -> float:
    vx, vy = bx - ax, by - ay
    wx, wy = px - ax, py - ay
    L2 = vx * vx + vy * vy
    t = 0.0 if L2 <= 1e-12 else max(0.0, min(1.0, (wx * vx + wy * vy) / L2))
    dx, dy = px - (ax + t * vx), py - (ay + t * vy)
    return math.hypot(dx, dy)


def _poly_to_circle(poly: Sequence[Tuple[float, float]], cx: float, cy: float, r: float) -> float:
    """多边形与圆的间距（负值 = 重叠深度）。"""
    dmin = min(_point_seg_dist(cx, cy, poly[i][0], poly[i][1],
                               poly[(i + 1) % len(poly)][0], poly[(i + 1) % len(poly)][1])
               for i in range(len(poly)))
    return dmin - r


def _poly_to_box(poly: Sequence[Tuple[float, float]],
                 x0: float, y0: float, x1: float, y1: float) -> float:
    """多边形与轴对齐矩形的间距（负值 = 重叠）。SAT 简化：任一顶点在内 → 负；否则取最近边距。"""
    # 1) 矩形顶点在多边形内 或 多边形顶点在矩形内 → 视为重叠
    def in_box(px, py):
        return x0 - COLLISION_EPS <= px <= x1 + COLLISION_EPS and y0 - COLLISION_EPS <= py <= y1 + COLLISION_EPS

    if any(in_box(px, py) for px, py in poly):
        return -COLLISION_EPS
    corners = [(x0, y0), (x1, y0), (x1, y1), (x0, y1)]
    if any(_point_in_poly(cx, cy, poly) for cx, cy in corners):
        return -COLLISION_EPS
    # 2) 否则取矩形 4 条边到多边形各边的最近距离
    best = float("inf")
    for i in range(len(poly)):
        ax, ay = poly[i]
        bx, by = poly[(i + 1) % len(poly)]
        for j in range(4):
            cx, cy = corners[j]
            dx, dy = corners[(j + 1) % 4]
            best = min(best, _seg_seg_dist(ax, ay, bx, by, cx, cy, dx, dy))
    return best


def _point_in_poly(px: float, py: float, poly: Sequence[Tuple[float, float]]) -> bool:
    inside = False
    n = len(poly)
    for i in range(n):
        x1, y1 = poly[i]
        x2, y2 = poly[(i + 1) % n]
        if (y1 > py) != (y2 > py):
            xin = (x2 - x1) * (py - y1) / (y2 - y1 + 1e-12) + x1
            if px < xin:
                inside = not inside
    return inside


def _seg_seg_dist(ax, ay, bx, by, cx, cy, dx, dy) -> float:
    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

    def intersect(p1, p2, p3, p4):
        d1 = cross(p3, p4, p1)
        d2 = cross(p3, p4, p2)
        d3 = cross(p1, p2, p3)
        d4 = cross(p1, p2, p4)
        return ((d1 > 0) != (d2 > 0)) and ((d3 > 0) != (d4 > 0))

    if intersect((ax, ay), (bx, by), (cx, cy), (dx, dy)):
        return 0.0
    return min(_point_seg_dist(ax, ay, cx, cy, dx, dy), _point_seg_dist(bx, by, cx, cy, dx, dy),
               _point_seg_dist(cx, cy, ax, ay, bx, by), _point_seg_dist(dx, dy, ax, ay, bx, by))


def pedestrian_xy(ped: Dict, t: float) -> Tuple[float, float]:
    """契约 v1.4 行人轨迹：x=x_cross，y = dir·speed·(t − t_meet)（匀速直线、无停住无钳位）。"""
    x = float(ped["x_cross"])
    y = float(ped["dir"]) * float(ped["speed"]) * (t - float(ped["t_meet"]))
    return x, y


def clearance_at(pose: Tuple[float, float, float], inst: Dict, fp: Optional[Footprint] = None,
                 t: Optional[float] = None) -> float:
    """该位姿下底盘外缘到场景几何的最小间距（m，负 = 穿透）。

    几何来源：静态圆障碍 + 墙体；给了 t 时**并把行人按契约轨迹当作圆障碍**一起算
    （动态场景必须能判定"撞人"，否则成功/碰撞判定会失真）。
    """
    fp = fp or Footprint()
    poly = fp.world_vertices(*pose)
    best = float("inf")
    for ob in inst.get("obstacles", []):
        best = min(best, _poly_to_circle(poly, ob["x"], ob["y"], ob["r"]))
    for w in inst.get("walls", []):
        # 实例表墙体：轴对齐 {x0,y0,x1,y1}
        best = min(best, _poly_to_box(poly, float(w["x0"]), float(w["y0"]),
                                      float(w["x1"]), float(w["y1"])))
    if t is not None:
        for ped in inst.get("pedestrians", []) or []:
            px, py = pedestrian_xy(ped, t)
            best = min(best, _poly_to_circle(poly, px, py, float(ped.get("r", 0.3))))
    return best


# --------------------------------------------------------------------------- #
# 七项指标 + 公共判定
# --------------------------------------------------------------------------- #
def path_length(states: Sequence[Sequence[float]]) -> float:
    return float(sum(math.hypot(states[i][0] - states[i - 1][0], states[i][1] - states[i - 1][1])
                     for i in range(1, len(states))))


def smoothness(t: Sequence[float], control: Sequence[Sequence[float]]) -> Optional[float]:
    """TV = Σ(‖Δv‖/v_max + |Δω|/ω_max)，控制量重采样到 20 Hz（契约口径）。"""
    if len(t) < 3 or len(control) < 3:
        return None
    ts = list(t)
    vs = [(c[0], c[1]) for c in control]
    ws = [c[2] for c in control]
    dt = 1.0 / CONTROL_HZ
    tv = 0.0
    k = 0
    cur_v = (vs[0][0], vs[0][1])
    cur_w = ws[0]
    step = 0.0
    while True:
        step += dt
        while k + 1 < len(ts) and ts[k + 1] <= ts[0] + step:
            k += 1
        if k + 1 >= len(ts):
            break
        nv = (vs[k][0], vs[k][1])
        tv += math.hypot(nv[0] - cur_v[0], nv[1] - cur_v[1]) / TASK_MAX_PLANAR_MPS
        tv += abs(ws[k] - cur_w) / TASK_MAX_WZ_RADPS
        cur_v, cur_w = nv, ws[k]
        if step > (ts[-1] - ts[0]) + 1.0:
            break
    return round(tv, 6)


def judge(inst: Dict, trajectory: Dict, estop_times: Optional[Sequence[float]] = None,
          t_start: Optional[float] = None) -> Dict:
    """对一个 run 做公共判定 + 指标计算。

    trajectory 需含：t (T,)、state (T,3) [x,y,yaw]、control (T,3) [vx,vy,wz]，均为 path frame。
    estop_times：`/safety/stop` 上升沿的仿真时刻列表（无安全层时传空列表）。
    """
    t = [float(x) for x in trajectory["t"]]
    states = [[float(v) for v in s] for s in trajectory["state"]]
    control = [[float(v) for v in c] for c in trajectory.get("control", [])]
    if not t or len(states) != len(t):
        raise ValueError("trajectory 的 t/state 长度不一致或为空")

    # 时间必须相对本 run 起点：/odom 的时间戳是**仿真时钟绝对值**（采集时可能已经跑到
    # 100 s+），直接用会把每个 run 都判成超时（2026-11-14 实测：实际 18.5 s 到达被算成
    # 125.9 s）。契约里的 t 一律指"本 run 开始后的秒数"。
    # t_start：本 run 的 t=0 在仿真时钟上的时刻。动态场景必须由采集器传入
    # （行人相位锚点），否则判分与仿真侧的行人位置不同步 → 假碰撞。
    t0 = t[0] if t_start is None else float(t_start)
    t_rel = [x - t0 for x in t]

    goal = inst["goal"]
    t_goal, yaw_goal = float(goal["x"]), float(goal.get("yaw", 0.0))
    limit = float(inst["time_limit_s"])

    # ---- 到达时刻（进入公差即算到达，之后位姿不考核）----
    t_arrive = None
    for k, (x, y, yaw) in enumerate(states):
        dyaw = abs(math.atan2(math.sin(yaw - yaw_goal), math.cos(yaw - yaw_goal)))
        if math.hypot(x - t_goal, y - float(goal.get("y", 0.0))) <= GOAL_POS_TOL_M and dyaw <= GOAL_YAW_TOL_RAD:
            t_arrive = t_rel[k]
            break

    # ---- 碰撞与最小间距 ----
    clr = [clearance_at((s[0], s[1], s[2]), inst, t=t_rel[k]) for k, s in enumerate(states)]
    min_clear = min(clr)
    collision = min_clear <= 0.0

    # ---- 急停（上升沿）----
    estops = sorted(float(e) for e in (estop_times or []))
    estop_count = 0
    prev = None
    for e in estops:
        if prev is None or (e - prev) > ESTOP_BURST_GAP_S:
            estop_count += 1     # 新的一次急停（上升沿）；连续段只计 1 次
        prev = e

    # ---- 时限 ----
    t_end = t_rel[-1]
    timeout = t_end > limit + 1e-9

    success = (not collision) and (t_arrive is not None) and (not timeout)
    if collision:
        failure = "collision"
    elif t_arrive is None:
        failure = "timeout" if timeout else "other"
    elif timeout:
        failure = "timeout"
    else:
        failure = None

    return {
        "success": bool(success),
        "failure_class": failure,
        "time_to_goal_s": None if t_arrive is None else round(t_arrive, 4),
        "path_length_m": round(path_length(states), 4),
        "smoothness": smoothness(t, control) if control else None,
        "min_clearance_m": round(min_clear, 4),
        "estop_count": int(estop_count),
        # 便于排查的附加信息（不进 metrics 契约键，落 run.json 的 judge_detail）
        "judge_detail": {
            "t_end_s": round(t_end, 4),
            "time_limit_s": limit,
            "arrived": t_arrive is not None,
            "collision": bool(collision),
            "timeout": bool(timeout),
            "samples": len(t),
            "control_samples": len(control),
            "peak_planar_speed_mps": round(max((math.hypot(c[0], c[1]) for c in control), default=0.0), 4),
        },
    }
