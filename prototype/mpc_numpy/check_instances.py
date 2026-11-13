#!/usr/bin/env python3
"""W4 加载器验收（任务 #3，conda pro，无 ROS）。

覆盖：
1) 实例表顶层断言（100 条 / 5×20 / seed 公式 / 2.0.0 门禁）；
2) 5 场景各抽 run 1 逐字段对照原始 YAML（start/goal/waypoints/t_max/圆障碍/行人闭式位姿/墙）；
3) box SDF 分支与圆障碍回归；
4) 地图栅格化：墙体占据、通道中心/起终点自由、动态障碍不进图；
5) 1 s 短仿真冒烟（圆/墙/行人三类几何进 QP 与真值裁判）；
6) write_failure schema 冒烟。

用法：python check_instances.py
退出码 0 ⟺ 全部断言通过。
"""
from __future__ import annotations

import dataclasses
import json
import sys
import tempfile
from pathlib import Path

import numpy as np
import yaml

ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT))

from mpc_proto.chassis import load_chassis
from mpc_proto.distance import Obstacle, Wall, clearance, min_clearance
from mpc_proto.mapping import FREE, OCCUPIED, build_grid
from mpc_proto.mpc import MPCConfig
from mpc_proto.runlog import write_failure
from mpc_proto.scenarios import (
    GENERATOR_VERSION,
    SCENARIO_KEYS,
    default_instances_path,
    load_instance,
    load_instance_table,
    load_instances,
    parse_instance_arg,
    parse_runs_spec,
)
from mpc_proto.sim import run

TABLE = default_instances_path()
RUN_KEYS = {
    "schema_version", "controller", "scenario", "seed", "run", "ablation",
    "metrics", "qp_timing", "failure_class", "params",
}


def check(cond: bool, msg: str) -> None:
    if not cond:
        raise AssertionError(msg)


def close(a, b, msg: str, atol: float = 1e-9) -> None:
    check(np.allclose(np.asarray(a, dtype=float), np.asarray(b, dtype=float), atol=atol), f"{msg}: {a} != {b}")


def grid_occupied(sc, grid, xmin: float, ymin: float, x: float, y: float) -> bool:
    """build_grid 返回 PGM 行序（row 0 = y_max），按 cell 中心查询。"""
    h, w = grid.shape
    col = int((x - xmin) / 0.05)
    row = h - 1 - int((y - ymin) / 0.05)
    check(0 <= col < w and 0 <= row < h, f"栅格查询出界: ({x},{y})")
    return int(grid[row, col]) == OCCUPIED


def check_table(instances: list[dict]) -> None:
    raw = yaml.safe_load(TABLE.read_text(encoding="utf-8"))
    check(raw["schema_version"] == 1, "schema_version 应为 1")
    check(str(raw["generator_version"]) == GENERATOR_VERSION, "generator_version 应为 2.0.0")
    check(str(raw["asset_version"]) == "2026-11-06", "asset_version 应为 2026-11-06")
    check(raw["coordinate_frame"] == "path", "coordinate_frame 应为 path")
    check(len(instances) == 100, f"实例数应为 100，实得 {len(instances)}")

    seeds = set()
    for key in SCENARIO_KEYS:
        sub = [i for i in instances if i["scenario"] == key]
        check(len(sub) == 20, f"{key} 应有 20 条，实得 {len(sub)}")
        check(sorted(i["run"] for i in sub) == list(range(1, 21)), f"{key} run 序应为 1..20")
        for inst in sub:
            idx = SCENARIO_KEYS.index(inst["scenario"]) + 1
            check(inst["scenario_index"] == idx, f"{key} scenario_index 错误")
            check(inst["seed"] == idx * 100 + inst["run"], f"{key} run {inst['run']} seed 公式错误")
            check(inst["seed"] not in seeds, f"seed 重复: {inst['seed']}")
            seeds.add(inst["seed"])
            goal = inst["goal"]
            check(
                np.isclose(inst["path_length_m"], np.hypot(goal["x"], goal["y"]), atol=1e-6),
                f"{key} run {inst['run']} path_length_m 不等于起终点直线长度",
            )
            check(
                np.isclose(goal["yaw"], np.arctan2(goal["y"], goal["x"]), atol=1e-6),
                f"{key} run {inst['run']} goal.yaw 不是连线方向",
            )
            check(len(inst["pedestrians"]) <= 2, f"{key} run {inst['run']} 行人数超过 2")
            check(len(inst["walls"]) <= 3, f"{key} run {inst['run']} 墙体数超过 3")


def check_negative_gate() -> None:
    """错误 generator_version 必须被加载器拒绝。"""
    raw = yaml.safe_load(TABLE.read_text(encoding="utf-8"))
    raw["generator_version"] = "1.9.9"
    with tempfile.TemporaryDirectory() as td:
        p = Path(td) / "instances.yaml"
        p.write_text(yaml.safe_dump(raw, allow_unicode=True, sort_keys=False), encoding="utf-8")
        try:
            load_instance_table(p)
        except ValueError as exc:
            check("generator_version" in str(exc), "门禁错误信息应包含 generator_version")
        else:
            raise AssertionError("generator_version 1.9.9 未被拒绝")


def check_instance(key: str, raw_inst: dict) -> None:
    sc, meta = load_instance(key, 1, path=TABLE)
    label = f"{key}_01"
    check(meta.scenario == key and meta.scenario_index == raw_inst["scenario_index"], f"{label} 元信息场景/序号错误")
    check(meta.run == 1 and meta.seed == raw_inst["seed"], f"{label} 元信息 run/seed 错误")
    check(meta.scenario_id == f"{key}_01", f"{label} scenario_id 错误: {meta.scenario_id}")
    check(meta.time_limit_s == raw_inst["time_limit_s"], f"{label} 元信息时限错误")
    check(meta.path_length_m == raw_inst["path_length_m"], f"{label} 元信息路径长度错误")

    goal = raw_inst["goal"]
    close(sc.start, [0.0, 0.0, 0.0], f"{label} start 应隐含 (0,0,0)")
    close(sc.goal, [goal["x"], goal["y"], goal["yaw"]], f"{label} goal 映射错误")
    close(sc.waypoints, [[0.0, 0.0], [goal["x"], goal["y"]]], f"{label} 默认 waypoints 应为 start→goal 直线")
    check(sc.t_max == raw_inst["time_limit_s"], f"{label} t_max 应取 time_limit_s")
    check(sc.name == key, f"{label} Scenario.name 应为场景键")
    check(meta.scenario_id in sc.desc, f"{label} desc 应含实例 id")

    n_static = len(raw_inst["obstacles"])
    n_ped = len(raw_inst["pedestrians"])
    check(len(sc.obstacles) == n_static + n_ped, f"{label} 障碍总数应为静态+行人")
    for i, o in enumerate(raw_inst["obstacles"]):
        obs = sc.obstacles[i]
        close(obs.p0, [o["x"], o["y"]], f"{label} 静障#{i + 1} 位置")
        close(obs.v, [0.0, 0.0], f"{label} 静障#{i + 1} 速度应为 0")
        check(obs.radius == o["r"], f"{label} 静障#{i + 1} 半径")
        check(not obs.dynamic, f"{label} 静障#{i + 1} 不应为动态")
    for j, p in enumerate(raw_inst["pedestrians"], start=n_static):
        obs = sc.obstacles[j]
        p0 = [p["x_cross"], -p["dir"] * p["speed"] * p["t_meet"]]
        v = [0.0, p["dir"] * p["speed"]]
        close(obs.p0, p0, f"{label} 行人#{j + 1} t=0 锚点")
        close(obs.v, v, f"{label} 行人#{j + 1} 恒定速度")
        check(obs.radius == p["r"], f"{label} 行人#{j + 1} 半径")
        check(obs.dynamic, f"{label} 行人#{j + 1} 应为动态")
        close(obs.center_at(p["t_meet"]), [p["x_cross"], 0.0], f"{label} 行人#{j + 1} t_meet 过线")

    check(len(sc.walls) == len(raw_inst["walls"]), f"{label} 墙体数量")
    for i, (wall, w) in enumerate(zip(sc.walls, raw_inst["walls"])):
        check((wall.x0, wall.y0, wall.x1, wall.y1) == (w["x0"], w["y0"], w["x1"], w["y1"]), f"{label} 墙#{i + 1} 角点")

    sc10, meta10 = load_instance(key, 10, path=TABLE)
    check(meta10.scenario_id == f"{key}_10", f"{label} run10 scenario_id 补零错误: {meta10.scenario_id}")


def check_sdf() -> None:
    wall = Wall(0.0, 0.0, 2.0, 1.0)
    cases = [
        ([-1.0, 0.5], 0.7, [-1.0, 0.0]),
        ([3.0, 0.5], 0.7, [1.0, 0.0]),
        ([1.0, -1.0], 0.7, [0.0, -1.0]),
        ([1.0, 0.5], -0.8, [0.0, 1.0]),
        ([0.0, 0.5], -0.3, [-1.0, 0.0]),
    ]
    for p, d_exp, g_exp in cases:
        c = clearance(np.array(p), wall, 0.0, 0.3)
        close(c.d, d_exp, f"box SDF {p} 间隙")
        close(c.grad, g_exp, f"box SDF {p} 梯度")
    c = clearance(np.array([2.5, 1.5]), wall, 0.0, 0.3)
    close(c.d, np.hypot(0.5, 0.5) - 0.3, "box SDF 角点间隙")
    close(c.grad, [np.sqrt(0.5), np.sqrt(0.5)], "box SDF 角点梯度")

    # 圆障碍回归（W2 语义不变）
    obs = Obstacle(1, np.array([4.0, 0.0]), np.zeros(2), 0.6)
    c = clearance(np.array([5.0, 0.0]), obs, 0.0, 0.3)
    close(c.d, 0.1, "圆障碍间隙回归")
    close(c.grad, [1.0, 0.0], "圆障碍梯度回归")
    close(min_clearance(np.array([5.0, 0.0]), [obs, wall], 0.0, 0.3), min(0.1, 0.7), "混合 min_clearance")

    try:
        Wall(1.0, 0.0, 1.0, 1.0)
    except ValueError:
        pass
    else:
        raise AssertionError("非法墙体（x0==x1）未被拒绝")


def check_mapping() -> None:
    sc, _ = load_instance("narrow_passage", 1, path=TABLE)
    grid, (xmin, ymin) = build_grid(sc)
    wall = sc.walls[0]
    check(grid_occupied(sc, grid, xmin, ymin, wall.center[0], wall.center[1]), "窄道墙体中心应为占据")
    check(not grid_occupied(sc, grid, xmin, ymin, wall.center[0], 0.539), "窄道通道中心应为自由")
    check(not grid_occupied(sc, grid, xmin, ymin, 0.0, 0.0), "起点应为自由")
    check(not grid_occupied(sc, grid, xmin, ymin, sc.goal[0], sc.goal[1]), "目标应为自由")

    sc, _ = load_instance("static_avoid", 1, path=TABLE)
    grid, (xmin, ymin) = build_grid(sc)
    o = sc.obstacles[0]
    check(grid_occupied(sc, grid, xmin, ymin, o.p0[0], o.p0[1]), "静障中心应为占据")

    sc, _ = load_instance("dead_end", 1, path=TABLE)
    grid, (xmin, ymin) = build_grid(sc)
    wall = sc.walls[0]
    check(grid_occupied(sc, grid, xmin, ymin, wall.center[0], wall.center[1]), "死角墙体中心应为占据")

    sc, _ = load_instance("dynamic_pedestrian", 1, path=TABLE)
    grid, _ = build_grid(sc)
    check((grid == FREE).all(), "动态行人不应进图（纯动态场景地图应全自由）")


def check_short_sim() -> None:
    chassis = load_chassis(ROOT / "chassis/omni.yaml")
    cfg = MPCConfig.benchmark()
    for key in ("static_avoid", "narrow_passage", "dynamic_pedestrian"):
        sc, meta = load_instance(key, 1, path=TABLE)
        short = dataclasses.replace(sc, t_max=1.0)
        res = run(short, chassis, cfg, log_every=0)
        check(res.ticks > 0, f"{meta.scenario_id} 短仿真无控制拍")
        check(res.solve_fail == 0, f"{meta.scenario_id} 短仿真出现 {res.solve_fail} 次求解失败")
        check(np.isfinite(res.min_clearance) and res.min_clearance > 0.0, f"{meta.scenario_id} 短仿真间隙应有限且 >0")
        print(
            f"  短仿真 {meta.scenario_id}: ticks={res.ticks} solve_fail={res.solve_fail} "
            f"min_clearance={res.min_clearance:.3f}"
        )

    # 墙体真值：窄道中心点到两墙内缘的间隙应 ≈0.404 m
    sc, _ = load_instance("narrow_passage", 1, path=TABLE)
    c = min_clearance(np.array([4.8955, 0.539]), sc.walls, 0.0, chassis.radius)
    close(c, 0.404, "窄道中心真值间隙", atol=2e-3)


def check_failure_record() -> None:
    cfg = MPCConfig.benchmark()
    with tempfile.TemporaryDirectory() as td:
        run_dir = write_failure(
            Path(td), "static_avoid", cfg, seed=101, run=1,
            failure_class="other", reason="smoke", params={"failure_phase": "navfn"},
        )
        rec = json.loads((run_dir / "run.json").read_text(encoding="utf-8"))
        check(set(rec) == RUN_KEYS, f"failure run 顶层字段不符合契约: {sorted(rec)}")
        check(rec["scenario"] == "static_avoid" and rec["seed"] == 101 and rec["run"] == 1, "failure run 元信息错误")
        check(rec["failure_class"] == "other", "failure_class 应为 other")
        check(rec["params"]["failure_phase"] == "navfn", "failure_phase 未写入 params")
        check(rec["metrics"]["success"] is False and rec["metrics"]["min_clearance_m"] is None, "failure 指标应成功=false/间距=null")
        check((run_dir / "trajectory.npz").is_file(), "failure run 缺 trajectory.npz")


def check_parsers() -> None:
    check(parse_instance_arg("static_avoid:1") == ("static_avoid", 1), "parse_instance_arg 正常值")
    check(parse_runs_spec("all") == list(range(1, 21)), "parse_runs_spec all")
    check(parse_runs_spec("1-5") == [1, 2, 3, 4, 5], "parse_runs_spec 1-5")
    check(parse_runs_spec("1,3,5") == [1, 3, 5], "parse_runs_spec 1,3,5")
    for bad in ("S1:1", "static_avoid:0", "static_avoid:21", "static_avoid"):
        try:
            parse_instance_arg(bad)
        except ValueError:
            pass
        else:
            raise AssertionError(f"parse_instance_arg 应拒绝 {bad!r}")
    for bad in ("0", "21", "5-1", "x"):
        try:
            parse_runs_spec(bad)
        except ValueError:
            pass
        else:
            raise AssertionError(f"parse_runs_spec 应拒绝 {bad!r}")

    pairs = load_instances(runs=[1, 2], path=TABLE)
    check(len(pairs) == 10, f"load_instances(runs=[1,2]) 应返回 10 条，实得 {len(pairs)}")


def main() -> int:
    print(f"实例表: {TABLE}")
    instances = load_instance_table(TABLE)
    checks = [
        ("表顶层/种子/字段", lambda: check_table(instances)),
        ("版本门禁（负例）", check_negative_gate),
    ]
    for key in SCENARIO_KEYS:
        raw_inst = next(i for i in instances if i["scenario"] == key and i["run"] == 1)
        checks.append((f"{key}_01 字段映射", lambda k=key, r=raw_inst: check_instance(k, r)))
    checks += [
        ("box SDF / 圆回归", check_sdf),
        ("地图栅格化", check_mapping),
        ("1 s 短仿真冒烟", check_short_sim),
        ("failure run schema", check_failure_record),
        ("CLI 解析", check_parsers),
    ]

    failed = 0
    for name, fn in checks:
        try:
            fn()
        except AssertionError as exc:
            failed += 1
            print(f"FAIL  {name}: {exc}")
        else:
            print(f"PASS  {name}")

    print(f"\n{len(checks) - failed}/{len(checks)} 项通过")
    return 0 if failed == 0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
