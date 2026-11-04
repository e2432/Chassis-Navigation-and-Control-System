#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""五类基准场景的实例生成器（路线书 E14）。

设计要点（每条都对应 E14 的一条硬要求）：

1. **确定性**：实例完全由 `seed` 决定 —— `seed = 场景基准 × 100 + run 序`（与算法侧
   `5场景需求清单.md` 的种子表一致），同一 seed 重跑结果逐字节相同。
2. **实例保存全部取值**，不只保存种子：位置、半径、速度、启动时刻都写进 YAML，
   这样即使生成器以后改了，旧实例仍可复现。
3. **物理有效性检查**（不合格就重抽样，最多 N 次）：
   - 机器人起点 / 目标点不在任何障碍内部；
   - 用膨胀后的占据栅格 + A* 确认**存在可通行路径**（避免"必然碰撞却标为正常避障"）；
   - 窄通道净宽 ≥ 1.3 m（按整机 footprint 0.662 m 折算单侧余量 ≥ 0.30 m）。
4. **可解性标注**：`dead_end` 标注 `solvable`/`recovery_required`，无解实例单列，
   不计入成功率分母（与算法侧指标定义口径一致）。
5. **同一批实例供所有控制器复用**：`test_list.yaml` 是冻结清单。

用法（生成 + 校验）：

    python3 -m robot_benchmark.gen_scenarios --out scenarios --runs 20
"""

from __future__ import annotations

import heapq
import math
import random
from typing import Any, Dict, List, Optional, Tuple

# ---- 场景键（与算法侧 docs/接口契约.md、5场景需求清单.md 一致，不增不换）----
SCENARIO_KEYS: Tuple[str, ...] = (
    "static_avoid",
    "dynamic_pedestrian",
    "narrow_passage",
    "dead_end",
    "mixed",
)
SEED_BASE: Dict[str, int] = {k: (i + 1) * 100 for i, k in enumerate(SCENARIO_KEYS)}

# ---- 底盘几何（来源 src/robot_description/config/robot_geometry.yaml）----
FOOTPRINT_ENVELOPE_M = 0.662            # 八边形外接方盒
ROBOT_RADIUS_M = 0.35                   # 保守外接圆半径（用于连通性/间距判定）
CLEARANCE_TARGET_M = 0.30               # 指标 5 阈值（单侧净空）
ESTP_THRESHOLD_M = 0.15                 # 安全层急停阈值（仅用于说明净宽下限）

GRID_M = 0.10                           # 连通性检查栅格
MAX_RESAMPLE = 200                      # 单个实例最多重抽样次数


# --------------------------------------------------------------------------- #
# 公共几何工具
# --------------------------------------------------------------------------- #
def _dist(ax: float, ay: float, bx: float, by: float) -> float:
    return math.hypot(ax - bx, ay - by)


def _point_in_obstacle(x: float, y: float, obs: Dict[str, Any], margin: float) -> bool:
    """点是否落在障碍（圆/矩形）内（含 margin 外扩）。"""
    if obs["type"] == "circle":
        return _dist(x, y, obs["x"], obs["y"]) < obs["r"] + margin
    # 矩形：{type: box, x, y, w, h}
    return (abs(x - obs["x"]) < obs["w"] / 2.0 + margin
            and abs(y - obs["y"]) < obs["h"] / 2.0 + margin)


def _segments_free(p0, p1, obstacles, r_robot) -> bool:
    """直线段是否与（外扩 r_robot 的）障碍相交。用于快速判否与最终复核。"""
    steps = max(2, int(_dist(*p0, *p1) / (GRID_M * 0.5)))
    for i in range(steps + 1):
        t = i / steps
        x = p0[0] + (p1[0] - p0[0]) * t
        y = p0[1] + (p1[1] - p0[1]) * t
        for obs in obstacles:
            if _point_in_obstacle(x, y, obs, r_robot):
                return False
    return True


def _connectivity_ok(start, goal, obstacles, r_robot=ROBOT_RADIUS_M,
                     bounds=(-2.0, -4.0, 22.0, 6.0)) -> bool:
    """膨胀栅格 + A*：起点到目标是否存在可通行路径（8 邻域）。"""
    x0, y0, x1, y1 = bounds
    nx = int((x1 - x0) / GRID_M) + 1
    ny = int((y1 - y0) / GRID_M) + 1

    def cell(x, y):
        return int(round((x - x0) / GRID_M)), int(round((y - y0) / GRID_M))

    def free(ix, iy):
        if not (0 <= ix < nx and 0 <= iy < ny):
            return False
        x, y = x0 + ix * GRID_M, y0 + iy * GRID_M
        return not any(_point_in_obstacle(x, y, o, r_robot) for o in obstacles)

    s, g = cell(*start), cell(*goal)
    if not free(*s) or not free(*g):
        return False
    open_heap = [(0.0, s)]
    best = {s: 0.0}
    neigh = [(1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
             (1, 1, 1.414), (1, -1, 1.414), (-1, 1, 1.414), (-1, -1, 1.414)]
    while open_heap:
        _, cur = heapq.heappop(open_heap)
        if cur == g:
            return True
        for dx, dy, cost in neigh:
            nxt = (cur[0] + dx, cur[1] + dy)
            if not free(*nxt):
                continue
            ng = best[cur] + cost
            if ng < best.get(nxt, 1e9):
                best[nxt] = ng
                h = math.hypot(nxt[0] - g[0], nxt[1] - g[1])
                heapq.heappush(open_heap, (ng + h, nxt))
    return False


# --------------------------------------------------------------------------- #
# 五类场景的实例生成
# --------------------------------------------------------------------------- #
def _gen_static(rng: random.Random, tpl: Dict[str, Any]) -> Optional[Dict[str, Any]]:
    L = rng.uniform(*tpl["corridor_length_m"])
    n = rng.randint(*tpl["obstacle_count"])
    off = tpl["lateral_offset_m"]
    half_c = tpl["corridor_half_width_m"]
    obstacles = []
    for _ in range(n):
        r = rng.uniform(*tpl["obstacle_radius_m"])
        x = rng.uniform(tpl["obstacle_x_range_m"][0], L - tpl["obstacle_x_range_m"][1])
        y = rng.uniform(-off, off)
        # 障碍位置抖动 ±0.3 m（需求清单）
        x += rng.uniform(-0.3, 0.3)
        y += rng.uniform(-0.3, 0.3)
        if abs(y) > half_c - r:
            continue
        obstacles.append({"type": "circle", "x": round(x, 3), "y": round(y, 3), "r": round(r, 3)})
    return {
        "start": (0.0, 0.0),
        "goal": (round(L, 3), 0.0),
        "obstacles": obstacles,
        "dynamic": [],
        "timeout_s": tpl["timeout_s"],
        "solvable": True,
    }


def _gen_pedestrian(rng: random.Random, tpl: Dict[str, Any]) -> Optional[Dict[str, Any]]:
    L = rng.uniform(*tpl["corridor_length_m"])
    n = rng.randint(*tpl["pedestrian_count"])
    dyn = []
    for _ in range(n):
        v = rng.uniform(*tpl["pedestrian_speed_mps"])
        cross_x = L / 2.0 + rng.uniform(-tpl["cross_x_jitter_m"], tpl["cross_x_jitter_m"])
        side = 1.0 if rng.random() < 0.5 else -1.0
        y_start = -side * tpl["cross_half_span_m"]
        half_span = tpl["cross_half_span_m"]
        t_to_center = half_span / v
        encounter = tpl["encounter_time_s"] + rng.uniform(-tpl["encounter_jitter_s"],
                                                          tpl["encounter_jitter_s"])
        dyn.append({
            "type": "pedestrian",
            "radius": tpl["pedestrian_radius_m"],
            "start": {"x": round(cross_x, 3), "y": round(y_start, 3)},
            "velocity": {"vx": 0.0, "vy": round(-side * v, 3)},
            "start_time_s": round(max(0.0, encounter - t_to_center), 3),
            "stop_at_y": round(-y_start, 3),
        })
    return {
        "start": (0.0, 0.0),
        "goal": (round(L, 3), 0.0),
        "obstacles": [],
        "dynamic": dyn,
        "timeout_s": tpl["timeout_s"],
        "solvable": True,
    }


def _gen_narrow(rng: random.Random, tpl: Dict[str, Any]) -> Optional[Dict[str, Any]]:
    L_total = tpl["approach_length_m"]
    plen = rng.uniform(*tpl["passage_length_m"])
    width = rng.uniform(*tpl["net_width_m"])
    # 净宽下限硬约束（需求清单：不得窄于 1.3 m）
    width = max(width, tpl["net_width_min_m"])
    half = width / 2.0
    x_in = L_total
    x_out = L_total + plen
    wall_t = 0.2
    obstacles = [
        # 通道两侧墙（上/下），各用一块矩形
        {"type": "box", "x": round((x_in + x_out) / 2, 3), "y": round(half + wall_t / 2, 3),
         "w": round(plen, 3), "h": wall_t},
        {"type": "box", "x": round((x_in + x_out) / 2, 3), "y": round(-half - wall_t / 2, 3),
         "w": round(plen, 3), "h": wall_t},
    ]
    goal = (round(x_out + 1.0, 3), 0.0)
    margin_per_side = (width - FOOTPRINT_ENVELOPE_M) / 2.0
    return {
        "start": (0.0, 0.0),
        "goal": goal,
        "obstacles": obstacles,
        "dynamic": [],
        "timeout_s": tpl["timeout_s"],
        "solvable": bool(margin_per_side >= CLEARANCE_TARGET_M - 1e-9),
        "passage": {"net_width_m": round(width, 3), "length_m": round(plen, 3),
                    "margin_per_side_m": round(margin_per_side, 3),
                    "min_required_margin_m": CLEARANCE_TARGET_M},
    }


def _gen_dead_end(rng: random.Random, tpl: Dict[str, Any]) -> Optional[Dict[str, Any]]:
    mouth = max(rng.uniform(*tpl["mouth_width_m"]), tpl["mouth_width_min_m"])
    depth = tpl["depth_m"]
    half = mouth / 2.0
    wall_t = 0.2
    x_front = tpl["entry_length_m"]
    obstacles = [
        # U 形：两条侧壁 + 一条底壁（袋口朝 -x）
        {"type": "box", "x": round(x_front + depth / 2, 3), "y": round(half + wall_t / 2, 3),
         "w": round(depth, 3), "h": wall_t},
        {"type": "box", "x": round(x_front + depth / 2, 3), "y": round(-half - wall_t / 2, 3),
         "w": round(depth, 3), "h": wall_t},
        {"type": "box", "x": round(x_front + depth + wall_t / 2, 3), "y": 0.0,
         "w": wall_t, "h": round(mouth + 2 * wall_t, 3)},
    ]
    return {
        "start": (0.0, 0.0),
        "goal": (round(x_front + depth - 0.6, 3), 0.0),     # 袋底
        "obstacles": obstacles,
        "dynamic": [],
        "timeout_s": tpl["timeout_s"],
        "solvable": True,                                   # 袋口敞开 → 存在进出路径
        "recovery_required": True,
        "dead_end": {"mouth_width_m": round(mouth, 3), "depth_m": depth},
    }


def _gen_mixed(rng: random.Random, tpl: Dict[str, Any]) -> Optional[Dict[str, Any]]:
    L = rng.uniform(*tpl["corridor_length_m"])
    base = _gen_static(rng, {
        "corridor_length_m": (L, L),
        "obstacle_count": tpl["obstacle_count"],
        "obstacle_radius_m": tpl["obstacle_radius_m"],
        "lateral_offset_m": tpl["lateral_offset_m"],
        "corridor_half_width_m": tpl["corridor_half_width_m"],
        "obstacle_x_range_m": tpl["obstacle_x_range_m"],
        "timeout_s": tpl["timeout_s"],
    })
    if base is None:
        return None
    ped = _gen_pedestrian(rng, {
        "corridor_length_m": (L, L),
        "pedestrian_count": tpl["pedestrian_count"],
        "pedestrian_speed_mps": tpl["pedestrian_speed_mps"],
        "pedestrian_radius_m": tpl["pedestrian_radius_m"],
        "cross_half_span_m": tpl["cross_half_span_m"],
        "cross_x_jitter_m": tpl["cross_x_jitter_m"],
        "encounter_time_s": tpl["encounter_time_s"],
        "encounter_jitter_s": tpl["encounter_jitter_s"],
        "timeout_s": tpl["timeout_s"],
    })
    base["dynamic"] = ped["dynamic"] if ped else []
    return base


_GENERATORS = {
    "static_avoid": _gen_static,
    "dynamic_pedestrian": _gen_pedestrian,
    "narrow_passage": _gen_narrow,
    "dead_end": _gen_dead_end,
    "mixed": _gen_mixed,
}


# --------------------------------------------------------------------------- #
# 有效性检查（E14 硬要求）
# --------------------------------------------------------------------------- #
def validate_instance(inst: Dict[str, Any], scenario: str) -> List[str]:
    """返回问题列表；空列表 = 合格。"""
    problems: List[str] = []
    start = tuple(inst["start"])
    goal = tuple(inst["goal"])
    obstacles = inst["obstacles"]

    for name, p in (("start", start), ("goal", goal)):
        for obs in obstacles:
            if _point_in_obstacle(p[0], p[1], obs, margin=0.10):
                problems.append("%s 落在障碍内部（%s）" % (name, obs))

    if inst.get("solvable", True) and not _connectivity_ok(start, goal, obstacles):
        problems.append("起点到目标不存在可通行路径（膨胀 %.2f m 后 A* 失败）" % ROBOT_RADIUS_M)

    if scenario == "narrow_passage":
        margin = inst["passage"]["margin_per_side_m"]
        if inst["passage"]["net_width_m"] < 1.30 - 1e-9:
            problems.append("窄通道净宽 %.3f m < 硬下限 1.30 m" % inst["passage"]["net_width_m"])
        if margin < CLEARANCE_TARGET_M - 1e-9:
            problems.append("单侧余量 %.3f m < 指标 5 阈值 %.2f m" % (margin, CLEARANCE_TARGET_M))

    if scenario == "dead_end" and not inst.get("recovery_required"):
        problems.append("dead_end 实例必须标注 recovery_required")

    for d in inst.get("dynamic", []):
        if d["type"] == "pedestrian":
            if not (0.0 < abs(d["velocity"]["vy"]) <= 1.2):
                problems.append("行人速度不合理：%s" % d["velocity"])
            if d["start_time_s"] < 0:
                problems.append("行人启动时刻为负")

    return problems


def generate_instance(scenario: str, run_index: int, tpl: Dict[str, Any]) -> Dict[str, Any]:
    """生成单个实例（确定性：完全由 seed 决定）。"""
    seed = SEED_BASE[scenario] + run_index
    rng = random.Random(seed)
    gen = _GENERATORS[scenario]
    for _attempt in range(MAX_RESAMPLE):
        inst = gen(rng, tpl)
        if inst is None:
            continue
        problems = validate_instance(inst, scenario)
        if not problems:
            return {
                "scenario_id": "%s_%02d" % (scenario, run_index),
                "scenario": scenario,
                "run_index": run_index,
                "seed": seed,
                "asset_version": tpl["asset_version"],
                "map_version": tpl["map_version"],
                "robot_start": {"x": inst["start"][0], "y": inst["start"][1], "yaw": 0.0},
                "goal": {"x": inst["goal"][0], "y": inst["goal"][1], "yaw": 0.0},
                "obstacles": inst["obstacles"],
                "dynamic": inst["dynamic"],
                "timeout_s": inst["timeout_s"],
                "goal_tolerance": tpl["goal_tolerance"],
                "solvable": inst.get("solvable", True),
                "recovery_required": inst.get("recovery_required", False),
                "passage": inst.get("passage"),
                "dead_end": inst.get("dead_end"),
                "validity": {
                    "connectivity_checked": True,
                    "robot_radius_m": ROBOT_RADIUS_M,
                    "grid_m": GRID_M,
                    "resamples": _attempt,
                },
            }
    raise RuntimeError("场景 %s run %d：%d 次重抽样后仍不合格" % (scenario, run_index, MAX_RESAMPLE))


def generate_scenario(scenario: str, tpl: Dict[str, Any], runs: int) -> List[Dict[str, Any]]:
    return [generate_instance(scenario, i, tpl) for i in range(1, runs + 1)]


# --------------------------------------------------------------------------- #
# 富化（rich）：给可视化/压力查看用的更复杂版本
# --------------------------------------------------------------------------- #
# 说明：评分用的实例清单（instances/*.yaml）保持与算法侧需求清单一致（如静态障碍 4~6 个）；
# 这里额外提供 rich 变体：在**不破坏可达性**的前提下按 E14 的要素把场景做得更复杂。
# 同一实例的 rich 结果由 (seed, salt) 决定，可复现。

def core_of(inst: Dict[str, Any]) -> Dict[str, Any]:
    """把清单实例转成 validate_instance 需要的最小结构。"""
    return {
        "start": (inst["robot_start"]["x"], inst["robot_start"]["y"]),
        "goal": (inst["goal"]["x"], inst["goal"]["y"]),
        "obstacles": list(inst["obstacles"]),
        "dynamic": list(inst["dynamic"]),
        "solvable": inst.get("solvable", True),
        "recovery_required": inst.get("recovery_required", False),
        "passage": inst.get("passage"),
        "dead_end": inst.get("dead_end"),
    }


def _extra_groups(scenario: str, inst: Dict[str, Any], rng: random.Random):
    """返回按"组"组织的额外障碍（一组一起加/一起回退，便于保证可达性）。"""
    st, gl = core_of(inst)["start"], core_of(inst)["goal"]
    L = max(abs(gl[0] - st[0]), 6.0)
    groups = []

    def flank(n_per_side, y_min, y_max, r_range, x_pad=(2.0, 2.0)):
        for _ in range(n_per_side * 2):
            side = 1.0 if rng.random() < 0.5 else -1.0
            r = rng.uniform(*r_range)
            x = rng.uniform(st[0] + x_pad[0], st[0] + L - x_pad[1])
            y = side * rng.uniform(y_min, y_max)
            groups.append([{"type": "circle", "x": round(x, 3), "y": round(y, 3), "r": round(r, 3)}])

    def slalom(n, y_range, r_range, x_lo, x_hi):
        for i in range(n):
            r = rng.uniform(*r_range)
            x = x_lo + (x_hi - x_lo) * (i + 0.5) / n
            side = -1.0 if i % 2 == 0 else 1.0
            groups.append([{"type": "circle", "x": round(x, 3),
                            "y": round(side * rng.uniform(*y_range), 3), "r": round(r, 3)}])

    if scenario == "static_avoid":
        flank(3, 2.0, 2.8, (0.25, 0.45))          # 两侧各 3 个"路边障碍"
        slalom(4, (0.9, 1.6), (0.25, 0.40),
               st[0] + 2.5, st[0] + L - 2.5)      # 4 个交错障碍（绕桩）
        groups.append([{"type": "circle", "x": round(gl[0] - 1.2, 3), "y": 0.0, "r": 0.35},
                       {"type": "circle", "x": round(gl[0] - 1.2, 3), "y": 1.05, "r": 0.30},
                       {"type": "circle", "x": round(gl[0] - 1.2, 3), "y": -1.05, "r": 0.30}])

    elif scenario == "dynamic_pedestrian":
        # E14：动态行人场景也应有空间参照物（路边障碍），并把行人加到 3~4 个
        flank(2, 2.2, 3.0, (0.25, 0.40))
        slalom(2, (1.2, 1.8), (0.25, 0.35), st[0] + 2.0, st[0] + L - 2.0)

    elif scenario == "narrow_passage":
        # 入口前绕桩 4 个 + 出口后再 2 个，形成"绕桩-过通道"的组合
        slalom(4, (0.9, 1.5), (0.25, 0.35), st[0] + 1.2, st[0] + 4.6)
        slalom(2, (0.8, 1.3), (0.25, 0.35), gl[0] - 4.0, gl[0] - 1.2)

    elif scenario == "dead_end":
        # 袋内两侧各 2 个（制造"局部堵塞/需要退让"），袋口外两侧各 1 个
        de = inst.get("dead_end") or {"depth_m": 2.0}
        x_front = st[0] + 6.0
        for i in range(2):
            x = x_front + 0.5 + 0.6 * i
            for side in (1.0, -1.0):
                groups.append([{"type": "circle", "x": round(x, 3),
                                "y": round(side * 0.42, 3), "r": 0.22}])
        for side in (1.0, -1.0):
            groups.append([{"type": "circle", "x": round(x_front - 1.2, 3),
                            "y": round(side * 1.15, 3), "r": 0.35}])
        flank(2, 1.8, 2.6, (0.25, 0.40), x_pad=(1.5, 2.5))   # 接近段路边障碍

    elif scenario == "mixed":
        flank(3, 2.0, 2.9, (0.25, 0.45))
        slalom(5, (1.0, 1.7), (0.25, 0.40), st[0] + 2.0, st[0] + L - 2.0)
        groups.append([{"type": "box", "x": round(st[0] + L * 0.55, 3), "y": 2.2,
                        "w": 1.6, "h": 0.2}])     # 一段"挡墙"，逼出绕行/穿缝

    return groups


def richify(scenario: str, inst: Dict[str, Any], salt: int = 0) -> Dict[str, Any]:
    """在保持"起点→目标可达"的前提下，按 E14 要素把实例加复杂度（确定性）。"""
    rng = random.Random(inst["seed"] * 7919 + 13 + salt)
    obstacles = list(inst["obstacles"])
    dynamic = list(inst["dynamic"])
    for group in _extra_groups(scenario, inst, rng):
        trial = dict(inst, obstacles=obstacles + group)
        if not validate_instance(core_of(trial), scenario):
            obstacles = obstacles + group          # 加上这组仍然可达 → 保留
        # 否则回退（这一组会让场景不可达/压到起点或目标）

    # 动态场景：把行人补到 3~4 个（新行人交叉点错开、启动时刻错开）
    if scenario in ("dynamic_pedestrian", "mixed") and dynamic:
        target = 4 if scenario == "dynamic_pedestrian" else 3
        base = len(dynamic)
        st, gl = core_of(inst)["start"], core_of(inst)["goal"]
        L = abs(gl[0] - st[0])
        for i in range(max(0, target - base)):
            v = rng.uniform(0.55, 1.05)
            side = 1.0 if i % 2 == 0 else -1.0
            cross_x = st[0] + L * (0.25 + 0.22 * i) + rng.uniform(-0.4, 0.4)
            dynamic.append({
                "type": "pedestrian", "radius": 0.30,
                "start": {"x": round(cross_x, 3), "y": round(-side * 3.0, 3)},
                "velocity": {"vx": 0.0, "vy": round(-side * v, 3)},
                "start_time_s": round(rng.uniform(2.5, 9.0), 3),
                "stop_at_y": round(side * 3.0, 3),
            })

    out = dict(inst)
    out["obstacles"] = obstacles
    out["dynamic"] = dynamic
    out["rich"] = {
        "enabled": True,
        "obstacles_total": len(obstacles),
        "obstacles_base": len(inst["obstacles"]),
        "dynamic_total": len(dynamic),
        "dynamic_base": len(inst["dynamic"]),
        "reachability_rechecked": True,
    }
    return out
