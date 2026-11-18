#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""基线 run 的落盘与采集（9-2）。schema 严格按 `docs/接口契约.md` v1.4「评测记录 schema」（v1）。

生成物（每个 run 一个目录）：

    <out_root>/runs/<scenario>/<seed>/
        run.json          # schema_version=1、controller、scenario、seed、run、ablation、
                          # metrics{success,time_to_goal_s,path_length_m,smoothness,min_clearance_m,estop_count}
                          # qp_timing、failure_class、params
        trajectory.npz    # t(T,)、state(T,3) [x,y,yaw] path frame、control(T,3) [vx,vy,wz]、qp_ms(K,)

控制器键：`dwa` | `mppi` | `teb` | `mpc_cpp` | `mpc_proto`（契约枚举之外的一律拒绝）。

采集后端（backend）：
  * `nav2`  —— 真实采集：需 M2 套件（Isaac + Nav2 + 安全层）在跑；当前仓库尚未接线，调用会明确报错，
               **不会**伪造数据；
  * `replay` —— 用给定的 trajectory.npz（或内置的合成轨迹）驱动裁判与指标，只用于**验证评分口径
              与落盘格式**（dry-run），run.json 里会标 `backend=replay_dryrun`，不得当正式成绩。

用法：
    # dry-run（不依赖仿真）：产出 3 条样例 run + 校验格式
    python3 -m robot_benchmark.runner --backend replay --baseline dwa --scenario static_avoid \
        --runs 1 2 3 --out out/9-2_dryrun

    # 真实采集（M2 套件就绪后）
    python3 -m robot_benchmark.runner --backend nav2 --baseline mppi --scenario mixed --runs 1 2
"""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
import time
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np
import yaml

from robot_benchmark import judge as judge_mod
from robot_benchmark import scenario_gen

CONTROLLERS = ("dwa", "mppi", "teb", "mpc_cpp", "mpc_proto")
BACKENDS = ("nav2", "replay")


# --------------------------------------------------------------------------- #
# 落盘
# --------------------------------------------------------------------------- #
def save_run(out_root: str, meta: Dict, metrics: Dict, traj: Dict) -> str:
    seed = meta["seed"]
    # 每个控制器一个独立输出根：<out>/<controller>/runs/<scenario>/<seed>/
    d = os.path.join(out_root, meta["controller"], "runs", meta["scenario"], str(seed))
    os.makedirs(d, exist_ok=True)
    record = {
        "schema_version": 1,
        "controller": meta["controller"],
        "scenario": meta["scenario"],
        "seed": seed,
        "run": meta.get("run"),
        "ablation": meta.get("ablation", {}),
        "metrics": {k: metrics[k] for k in
                    ("success", "time_to_goal_s", "path_length_m", "smoothness",
                     "min_clearance_m", "estop_count")},
        "qp_timing": meta.get("qp_timing", {}),
        "failure_class": metrics.get("failure_class"),
        "params": meta.get("params", {}),
        # 以下为工程侧附加信息（契约允许的自由字典，不影响算法侧读取）
        "judge_detail": metrics.get("judge_detail", {}),
        "backend": meta.get("backend", "unknown"),
        "generator_version": meta.get("generator_version"),
        "asset_version": meta.get("asset_version"),
        "recorded_at": time.strftime("%Y-%m-%dT%H:%M:%S"),
    }
    with open(os.path.join(d, "run.json"), "w", encoding="utf-8") as f:
        json.dump(record, f, ensure_ascii=False, indent=2)
    np.savez(os.path.join(d, "trajectory.npz"),
             t=np.asarray(traj["t"], dtype=float),
             state=np.asarray(traj["state"], dtype=float).reshape(-1, 3),
             control=np.asarray(traj["control"], dtype=float).reshape(-1, 3),
             qp_ms=np.asarray(traj.get("qp_ms", []), dtype=float))
    return d


# --------------------------------------------------------------------------- #
# dry-run 用的合成轨迹（只为验证评分/格式，三种典型结局）
# --------------------------------------------------------------------------- #

def plan_path(inst: Dict, grid: float = 0.2, clear_margin: float = 0.12) -> List[Tuple[float, float]]:
    """在场景几何上做 A*（节点可用性 = 底盘外缘间距 > clear_margin），返回 path-frame 路点。"""
    import heapq

    goal = (float(inst["goal"]["x"]), float(inst.get("goal", {}).get("y", 0.0)))
    start = (0.0, 0.0)
    xs = [w["x0"] for w in inst.get("walls", [])] + [o["x"] - o["r"] for o in inst.get("obstacles", [])]
    ys = [w["y0"] for w in inst.get("walls", [])] + [o["y"] - o["r"] for o in inst.get("obstacles", [])]
    xe = [w["x1"] for w in inst.get("walls", [])] + [o["x"] + o["r"] for o in inst.get("obstacles", [])]
    ye = [w["y1"] for w in inst.get("walls", [])] + [o["y"] + o["r"] for o in inst.get("obstacles", [])]
    x_lo = min([0.0] + (xs or [0.0])) - 2.0
    x_hi = max([goal[0]] + (xe or [goal[0]])) + 2.0
    y_lo = min([-1.0] + (ys or [-1.0])) - 2.0
    y_hi = max([1.0] + (ye or [1.0])) + 2.0

    def free(px, py):
        return judge_mod.clearance_at((px, py, 0.0), inst) > clear_margin

    def cell(p):
        return (int(round((p[0] - x_lo) / grid)), int(round((p[1] - y_lo) / grid)))

    def pos(c):
        return (x_lo + c[0] * grid, y_lo + c[1] * grid)

    s_cell, g_cell = cell(start), cell(goal)
    if not free(*pos(g_cell)):
        # 目标本身贴着障碍（窄通道/死角除外）：放宽 margin 再试
        for m in (0.05, 0.02, 0.0):
            if judge_mod.clearance_at((goal[0], goal[1], 0.0), inst) > m:
                break
        clear_local = judge_mod.clearance_at((goal[0], goal[1], 0.0), inst) - 0.01
        def free(px, py, _c=clear_local):
            return judge_mod.clearance_at((px, py, 0.0), inst) > max(0.0, _c)
    open_heap = [(0.0, s_cell)]
    came, cost = {}, {s_cell: 0.0}
    neigh = [(1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (1, -1), (-1, 1), (-1, -1)]
    while open_heap:
        _, cur = heapq.heappop(open_heap)
        if cur == g_cell:
            break
        for dx, dy in neigh:
            nxt = (cur[0] + dx, cur[1] + dy)
            px, py = pos(nxt)
            if not (x_lo <= px <= x_hi and y_lo <= py <= y_hi) or not free(px, py):
                continue
            nc = cost[cur] + math.hypot(dx, dy)
            if nc < cost.get(nxt, 1e18):
                cost[nxt] = nc
                came[nxt] = cur
                heapq.heappush(open_heap, (nc + math.hypot(nxt[0] - g_cell[0], nxt[1] - g_cell[1]), nxt))
    if g_cell not in came and g_cell != s_cell:
        return [start, goal]
    path = [g_cell]
    while path[-1] != s_cell and path[-1] in came:
        path.append(came[path[-1]])
    path.reverse()
    pts = [start] + [pos(c) for c in path[1:-1]] + [goal]
    # 抽稀：保留转角（三点共线则删中点）
    out = [pts[0]]
    for i in range(1, len(pts) - 1):
        (x0, y0), (x1, y1), (x2, y2) = out[-1], pts[i], pts[i + 1]
        if abs((x1 - x0) * (y2 - y1) - (y1 - y0) * (x2 - x1)) > 1e-6:
            out.append(pts[i])
    out.append(pts[-1])
    return out

def synth_trajectory(inst: Dict, kind: str, dt: float = 0.05) -> Tuple[Dict, List[float]]:
    goal = inst["goal"]
    limit = float(inst["time_limit_s"])
    v = 0.45

    if kind == "collision":
        target = None
        for ob in inst.get("obstacles", []):
            if target is None or abs(ob["x"]) < abs(target[0]):
                target = (ob["x"], ob["y"])
        if target is None:
            target = (float(goal["x"]), float(goal.get("y", 0.0)))
        ts, xs, ys = [], [], []
        t_ = 0.0
        while t_ < limit:
            k = min(1.0, (v * t_) / max(1e-6, math.hypot(*target)))
            xs.append(target[0] * k)
            ys.append(target[1] * k)
            ts.append(t_)
            if k >= 1.0:
                break
            t_ += dt
        yaw = np.zeros(len(xs))
        return ({"t": np.array(ts), "state": np.stack([np.array(xs), np.array(ys), yaw], axis=1),
                 "control": np.tile([v, 0.0, 0.0], (len(ts), 1)),
                 "qp_ms": np.full(max(1, len(ts) // 2), 3.0)}, [0.0])

    waypoints = plan_path(inst)
    # 沿路点以 v 行驶；success 走完全程，timeout 只走 60% 就停
    v_eff = v if kind == "success" else v * 0.35
    seg_len = [math.hypot(waypoints[i + 1][0] - waypoints[i][0],
                          waypoints[i + 1][1] - waypoints[i][1]) for i in range(len(waypoints) - 1)]
    total = sum(seg_len)
    travel = total if kind == "success" else 0.6 * total
    ts, states, ctrls = [], [], []
    s_trav = 0.0
    t_ = 0.0
    while s_trav <= min(travel, total) + 1e-9:
        # 找到当前所在段
        acc = 0.0
        k = 0
        for i, L_i in enumerate(seg_len):
            if s_trav <= acc + L_i + 1e-9:
                k = i
                break
            acc += L_i
        frac = 0.0 if seg_len[k] <= 1e-9 else (s_trav - acc) / seg_len[k]
        (x0, y0), (x1, y1) = waypoints[k], waypoints[k + 1]
        x, y = x0 + (x1 - x0) * frac, y0 + (y1 - y0) * frac
        yaw = math.atan2(y1 - y0, x1 - x0)
        ts.append(t_)
        states.append([x, y, yaw])
        ctrls.append([v_eff, 0.0, 0.0])
        if s_trav >= travel:
            if kind == "success":
                break
            # timeout 场景：停住不动，一直采样到超过时限（模拟"卡住不动"）
            while t_ <= limit + 2.0:
                t_ += dt
                ts.append(t_)
                states.append([x, y, yaw])
                ctrls.append([0.0, 0.0, 0.0])
            break
        s_trav = min(travel, s_trav + v_eff * dt)
        t_ += dt
        if t_ > limit + 10.0:
            break
    return ({"t": np.array(ts), "state": np.array(states),
             "control": np.array(ctrls), "qp_ms": np.full(max(1, len(ts) // 2), 3.0)}, [])


# --------------------------------------------------------------------------- #
# 后端
# --------------------------------------------------------------------------- #
def run_nav2(inst: Dict, baseline: str, out_root: str,
             world_pose: Optional[Dict] = None,
             timeout_s: Optional[float] = None,
             warmup_s: float = 45.0,
             controller_id: Optional[str] = None,
             scene_file: Optional[str] = None) -> Tuple[Dict, List[float], Dict]:
    """真跑一个 run：驱动在线 Nav2 栈（M2），返回 (trajectory, estop_times, diag)。

    前置条件：Isaac + `nav2_baseline.launch.py`（controller_server + planner_server）
    已经在线；**仿真场景必须是本实例对应的那个**（driver 里有硬校验，不一致直接报错）。

    典型启动方式（一个实例一个场景，栈起一次可连续跑该实例的所有 run）：
        ros2 launch robot_bringup simulation.launch.py nav2:=dwa demo:=false rviz:=false \
            duration:=0 scene:=<按清单生成的 .usda>
    """
    if baseline == "teb":
        # TEB 已在本机构建（官方 humble-devel 源码），但必须先 source ASCII 构建的镜像工作区；
        # 这里做一次前置校验，避免"以为在跑 TEB、实际栈是空的"。
        try:
            from ament_index_python.packages import get_package_prefix
            get_package_prefix("teb_local_planner")
        except Exception as e:
            raise RuntimeError(
                "找不到 teb_local_planner，TEB 采集无法进行。请先：\n"
                "  export LD_LIBRARY_PATH=$HOME/teb_prebuilt/opt/ros/humble/lib/x86_64-linux-gnu:$LD_LIBRARY_PATH\n"
                "  source ~/teb_build_ws/install/setup.bash\n"
                "（构建说明见 docs/9-1工程执行台账.md §I；原始错误：%s）" % e)
    from . import nav2_driver
    from .scenario_gen import default_manifest_path
    if world_pose is None:
        _doc = yaml.safe_load(open(default_manifest_path(), encoding="utf-8"))
        world_pose = (_doc.get("world_poses") or {}).get(
            str(inst["seed"]), {"x": 0.0, "y": 0.0, "yaw": 0.0})
    limit = float(timeout_s if timeout_s is not None else inst["time_limit_s"])
    # 多控制器同栈时，FollowPath 的 controller_id 就是基线名（见 nav2_baseline.launch.py）
    out = nav2_driver.run_episode(inst, world_pose, limit, warmup_s=warmup_s,
                                  controller_id=controller_id or baseline,
                                  scene_file=scene_file)
    return out["trajectory"], out["estop_times"], out["diag"]


def collect(out_root: str, backend: str, baseline: str, scenario: str,
            runs: Sequence[int], manifest: Optional[str] = None,
            dryrun_kinds: Optional[Sequence[str]] = None,
            timeout: Optional[float] = None,
            scene_file: Optional[str] = None) -> List[str]:
    if baseline not in CONTROLLERS:
        raise SystemExit("baseline 必须是 %s" % (CONTROLLERS,))
    doc = yaml.safe_load(open(manifest or scenario_gen.default_manifest_path(), encoding="utf-8"))
    made = []
    for i, run in enumerate(runs):
        inst = scenario_gen.find_instance(doc, scenario, run)
        if backend == "nav2":
            traj, estops, diag = run_nav2(inst, baseline, out_root, timeout_s=timeout,
                                          scene_file=scene_file)
        else:
            diag = {}
            kind = (dryrun_kinds[i] if dryrun_kinds else "success")
            traj, estops = synth_trajectory(inst, kind)
        # 有行人的场景用采集器记录的相位锚点当 t=0（无行人时锚点对齐与否不影响几何判定）
        t_start = (diag or {}).get("episode_anchor_sim") if inst.get("pedestrians") else None
        metrics = judge_mod.judge(inst, traj, estops, t_start=t_start)
        meta = {
            "controller": baseline, "scenario": scenario, "seed": inst["seed"], "run": run,
            "backend": "replay_dryrun" if backend == "replay" else "nav2",
            "ablation": {"note": "baseline 无消融开关，保留空快照"},   # 契约要求必填
            # diag 放进 params（契约里 params 是自由字典；顶层多余键会被 save_run 的固定
            # schema 丢掉，2026-11-14 实测 diag 写不进去）
            "params": {"baseline_yaml": "nav_ws_core/src/robot_bringup/config/baselines/%s.yaml" % baseline,
                       "flat_speed_limit_mps": judge_mod.TASK_MAX_PLANAR_MPS,
                       "m2_diag": diag},
            "qp_timing": {},
            "generator_version": doc.get("generator_version"),
            "asset_version": doc.get("asset_version"),
        }
        d = save_run(out_root, meta, metrics, traj)
        made.append(d)
        label = "%s_%02d" % (scenario, run)
        print("%-14s run=%2d  %-22s success=%-5s failure=%-9s 到达=%-7s 间距=%6.3f 急停=%d → %s"
              % (scenario, run, label, metrics["success"], str(metrics["failure_class"]),
                 str(metrics["time_to_goal_s"]), metrics["min_clearance_m"],
                 metrics["estop_count"], d))
    return made


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="基线 run 采集（9-2）")
    ap.add_argument("--backend", choices=BACKENDS, default="replay")
    ap.add_argument("--baseline", choices=CONTROLLERS, default="dwa")
    ap.add_argument("--scenario", choices=scenario_gen.SCENARIO_KEYS, required=True)
    ap.add_argument("--runs", nargs="+", type=int, default=[1])
    ap.add_argument("--out", default="out/9-2")
    ap.add_argument("--manifest", default=None, help="实例表路径（默认按契约路径自动查找）")
    ap.add_argument("--scene", default=None,
                    help="本次所用场景文件（.usda）；给了就做 scenario/seed 身份校验")
    ap.add_argument("--timeout", type=float, default=None,
                    help="单 run 判定时限（秒）；默认用实例表的 time_limit_s")
    ap.add_argument("--dryrun-kinds", nargs="+", default=None,
                    choices=["success", "timeout", "collision"],
                    help="replay 后端的合成结局（仅 dry-run 用）")
    a = ap.parse_args(argv)
    collect(a.out, a.backend, a.baseline, a.scenario, a.runs, a.manifest, a.dryrun_kinds,
            timeout=a.timeout, scene_file=a.scene)
    return 0


if __name__ == "__main__":
    sys.exit(main())
