#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""9-2 裁判/落盘/汇总的单元测试（不依赖 ROS、不依赖仿真）。

覆盖：到达 / 超时 / 碰撞判定、七项指标、急停上升沿口径、run.json + trajectory.npz 格式、汇总聚合。
跑法：cd nav_ws_core/src/robot_benchmark && PYTHONPATH=. python3 -m pytest test/test_judge.py -q
"""

import json
import math
import os

import numpy as np
import pytest
import yaml

from robot_benchmark import judge as J
from robot_benchmark import scenario_gen
from robot_benchmark import runner as R
from robot_benchmark import summarize as S

INST = {
    "scenario": "static_avoid", "scenario_index": 1, "run": 1, "seed": 101,
    "path_length_m": 10.0, "time_limit_s": 60.0,
    "goal": {"x": 10.0, "y": 0.0, "yaw": 0.0},
    "obstacles": [{"x": 5.0, "y": 3.0, "r": 0.5}],     # 不在 y=0 直线上
    "pedestrians": [], "walls": [],
}


def straight(v=0.5, L=10.0, dt=0.05, stop_at=None):
    n = int(L / v / dt) + 1
    t = np.arange(n) * dt
    x = np.minimum(v * t, L)
    state = np.stack([x, np.zeros(n), np.zeros(n)], axis=1)
    ctrl = np.tile([v, 0.0, 0.0], (n, 1))
    if stop_at is not None:
        k = int(stop_at / dt)
        x = np.minimum(v * t, stop_at)
        state = np.stack([x, np.zeros(n), np.zeros(n)], axis=1)
        ctrl[k:] = 0.0
    return {"t": t, "state": state, "control": ctrl, "qp_ms": np.full(max(1, n // 2), 2.0)}


def test_success_and_metrics():
    traj = straight(v=0.5, L=10.0)
    m = J.judge(INST, traj, estop_times=[])
    assert m["success"] is True
    assert m["failure_class"] is None
    # 进入 0.25 m 公差即算到达 → 到达时刻 = (L - 0.25)/v 附近
    assert 19.0 <= m["time_to_goal_s"] <= 20.0
    assert m["path_length_m"] == pytest.approx(10.0, abs=0.2)
    assert m["min_clearance_m"] > 0.5
    assert m["estop_count"] == 0
    assert m["smoothness"] == pytest.approx(0.0, abs=1e-9)   # 匀速直线：总变差为 0


def test_collision_detected():
    inst = dict(INST, obstacles=[{"x": 5.0, "y": 0.0, "r": 0.5}])   # 挡在直线上
    m = J.judge(inst, straight(v=0.5, L=10.0), estop_times=[])
    assert m["success"] is False
    assert m["failure_class"] == "collision"
    assert m["min_clearance_m"] < 0.0


def test_timeout_detected():
    inst = dict(INST, time_limit_s=5.0)
    m = J.judge(inst, straight(v=0.5, L=10.0), estop_times=[])   # 需要 20 s > 5 s 时限
    assert m["failure_class"] == "timeout"
    assert m["success"] is False
    assert m["judge_detail"]["timeout"] is True


def test_arrival_tolerance():
    # 差 0.3 m（>0.25 容差）不算到达
    traj = straight(v=0.5, L=9.7)
    m = J.judge(INST, traj, estop_times=[])
    assert m["success"] is False
    # 差 0.2 m 算到达
    m2 = J.judge(INST, straight(v=0.5, L=9.8, stop_at=9.8), estop_times=[])
    assert m2["success"] is True


def test_pedestrian_counts_as_collision_geometry():
    # 行人沿 y 轴横穿，机器人停在 y=0：t_meet 时刻两者重合 → 必须判碰撞
    inst = dict(INST, obstacles=[], walls=[],
                pedestrians=[{"x_cross": 5.0, "speed": 1.0, "dir": 1.0, "t_meet": 5.0, "r": 0.3}])
    n = 300
    t = np.arange(n) * 0.05
    state = np.stack([np.full(n, 5.0), np.zeros(n), np.zeros(n)], axis=1)   # 停在 (5,0)
    traj = {"t": t, "state": state, "control": np.zeros((n, 3))}
    m = J.judge(dict(inst, goal={"x": 5.0, "y": 0.0, "yaw": 0.0}, time_limit_s=100.0), traj, [])
    assert m["failure_class"] == "collision"
    assert m["min_clearance_m"] < 0.0


def test_estop_rising_edge_count():
    inst = dict(INST, time_limit_s=100.0)
    traj = straight(v=0.5, L=10.0)
    m = J.judge(inst, traj, estop_times=[3.0, 3.05, 3.1, 20.0])   # 连续 3 帧算 1 次 + 1 次
    assert m["estop_count"] == 2


def test_smoothness_step():
    n = 100
    t = np.arange(n) * 0.05
    state = np.stack([np.zeros(n), np.zeros(n), np.zeros(n)], axis=1)
    ctrl = np.zeros((n, 3))
    ctrl[: n // 2] = (0.0, 0.0, 0.0)
    ctrl[n // 2:] = (0.5, 0.0, 0.0)      # 一次速度阶跃 0.5 m/s
    traj = {"t": t, "state": state, "control": ctrl}
    m = J.judge(dict(INST, time_limit_s=100.0, goal={"x": 0.0, "y": 0.0, "yaw": 0.0}), traj, [])
    assert m["smoothness"] == pytest.approx(0.5 / J.TASK_MAX_PLANAR_MPS, abs=0.05)


def test_footprint_clearance_geometry():
    # 无障碍 → 间距应为 inf 或很大；贴墙时按矩形几何算
    inst = {"obstacles": [], "walls": [{"x0": 1.0, "y0": -2.0, "x1": 1.2, "y1": 2.0}]}
    c = J.clearance_at((0.0, 0.0, 0.0), inst)
    assert c == pytest.approx(1.0 - 0.3215, abs=0.01)      # 外缘到墙面的距离


def test_save_run_schema(tmp_path):
    traj = straight(v=0.5, L=10.0)
    metrics = J.judge(INST, traj, [])
    meta = {"controller": "dwa", "scenario": "static_avoid", "seed": 101, "run": 1,
            "ablation": {}, "params": {}, "qp_timing": {}, "backend": "unit_test"}
    d = R.save_run(str(tmp_path), meta, metrics, traj)
    with open(os.path.join(d, "run.json"), encoding="utf-8") as f:
        rec = json.load(f)
    for key in ("schema_version", "controller", "scenario", "seed", "run", "ablation",
                "metrics", "qp_timing", "failure_class", "params"):
        assert key in rec, key
    assert set(rec["metrics"]) == {"success", "time_to_goal_s", "path_length_m",
                                   "smoothness", "min_clearance_m", "estop_count"}
    z = np.load(os.path.join(d, "trajectory.npz"))
    assert set(z.files) == {"t", "state", "control", "qp_ms"}
    assert z["state"].shape[1] == 3 and z["control"].shape[1] == 3


def test_summarize_aggregation(tmp_path):
    root = str(tmp_path)
    for scen, seeds in (("static_avoid", [101, 102]), ("mixed", [501])):
        for seed in seeds:
            inst = dict(INST, scenario=scen, seed=seed)
            traj = straight(v=0.5, L=10.0)
            metrics = J.judge(inst, traj, [])
            R.save_run(root, {"controller": "dwa", "scenario": scen, "seed": seed, "run": 1,
                              "backend": "unit_test"}, metrics, traj)
    runs = S.load_runs(root)
    summary = S.summarize(runs)
    assert summary["per_scenario"]["static_avoid"]["runs"] == 2
    assert summary["per_scenario"]["static_avoid"]["success_rate"] == 1.0
    assert summary["dynamic_stress"] == 1.0          # mixed 全成功
    assert summary["anomalies"] == []
    md = S.render_markdown(summary, "t")
    assert "static_avoid" in md and "dynamic_stress" in md


def test_runner_rejects_unknown_baseline(tmp_path):
    with pytest.raises(SystemExit):
        R.collect(str(tmp_path), "replay", "not_a_controller", "static_avoid", [1])


def test_runner_synth_three_kinds(tmp_path):
    """replay 后端：三种结局都要产出合法 run.json，且失败分类正确。"""
    out = str(tmp_path)
    R.collect(out, "replay", "dwa", "static_avoid", [1, 2, 3],
              dryrun_kinds=["success", "timeout", "collision"])
    kinds = []
    for seed in (101, 102, 103):
        with open(os.path.join(out, "dwa", "runs", "static_avoid", str(seed), "run.json"),
                  encoding="utf-8") as f:
            kinds.append(json.load(f)["failure_class"])
    assert kinds[0] is None
    assert kinds[1] == "timeout"
    assert kinds[2] == "collision"


def test_nav2_blocked_baseline_fails_loudly(tmp_path):
    """backend=nav2 已接线（2026-11-14），但被阻塞的基线仍必须显式报错、不得静默降级。

    TEB 因为 teb_local_planner 未安装/未验收，必须直接抛错；这条断言防止有人为了
    "让采集跑通"而把 TEB 换成 diff 或悄悄跳过一个基线。
    """
    doc = yaml.safe_load(open(scenario_gen.default_manifest_path(), encoding="utf-8"))
    inst = scenario_gen.find_instance(doc, "static_avoid", 1)
    with pytest.raises(RuntimeError) as exc:
        R.run_nav2(inst, "teb", str(tmp_path))
    assert "TEB" in str(exc.value)


def test_nav2_unknown_baseline_rejected(tmp_path):
    doc = yaml.safe_load(open(scenario_gen.default_manifest_path(), encoding="utf-8"))
    inst = scenario_gen.find_instance(doc, "static_avoid", 1)
    with pytest.raises(SystemExit):
        R.collect(str(tmp_path), "nav2", "not_a_controller", "static_avoid", [1])


def test_static_map_marks_walls():
    """回归：墙必须进静态地图。

    2026-11-15 踩过的坑：`build_occupancy` 用"格心是否落在墙内"判定，迭代范围又正好
    等于墙的包围盒，于是比一格的墙**一个格子都标不上**——dead_end / narrow_passage 的
    静态图占用格数为 0，机器人拿着"没有墙的地图"直接穿墙，13/15 失败。
    这条测试卡住"墙场景静态图必须有占用格"。
    """
    from robot_benchmark import scenes
    doc = yaml.safe_load(open(scenario_gen.default_manifest_path(), encoding="utf-8"))
    for sc in ("dead_end", "narrow_passage"):
        inst = scenario_gen.find_instance(doc, sc, 1)
        assert inst.get("walls"), "%s 应该带墙" % sc
        occ = scenes.build_occupancy(inst, doc["world_poses"][str(inst["seed"])])
        n_occ = sum(1 for v in occ["data"] if v)
        assert n_occ > 100, "%s 的静态图占用格只有 %d，墙没标进去" % (sc, n_occ)
        # 抽查每面墙的中心必须落在占用格上。注意实例几何是 **path frame**，
        # 静态图是 **map frame**，中间差一个 world_pose（可能带旋转）——这里必须转换，
        # 否则 yaw != 0 的种子会被误判（2026-11-15 我自己就在这上面栽过一次）。
        wp = doc["world_poses"][str(inst["seed"])]
        c, s_ = math.cos(wp["yaw"]), math.sin(wp["yaw"])
        n, res, ox, oy = occ["width"], occ["resolution"], occ["origin_x"], occ["origin_y"]
        for w in inst["walls"]:
            lx = (float(w["x0"]) + float(w["x1"])) / 2.0
            ly = (float(w["y0"]) + float(w["y1"])) / 2.0
            cx = c * lx - s_ * ly + wp["x"]
            cy = s_ * lx + c * ly + wp["y"]
            ix, iy = int((cx - ox) / res), int((cy - oy) / res)
            assert occ["data"][iy * n + ix] == 100, \
                "%s 的墙中心 map 系 (%.2f,%.2f) 未标记" % (sc, cx, cy)


def test_pedestrians_not_in_static_map():
    """行人会动，不能进静态地图（否则等于给机器人一个不存在的固定障碍）。"""
    from robot_benchmark import scenes
    doc = yaml.safe_load(open(scenario_gen.default_manifest_path(), encoding="utf-8"))
    inst = scenario_gen.find_instance(doc, "dynamic_pedestrian", 1)
    assert inst.get("pedestrians")
    occ = scenes.build_occupancy(inst, doc["world_poses"][str(inst["seed"])])
    assert sum(1 for v in occ["data"] if v) == 0
