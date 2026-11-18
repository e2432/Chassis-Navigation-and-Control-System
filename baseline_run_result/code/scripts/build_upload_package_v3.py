#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""把 9-1 / 9-2 / 9-3 的"该上传 Github 的文件"整理到一个文件夹（默认桌面）。

只收**主代码 + 配置 + 评分清单 + 结果文档与汇总**：
不收 build/install/log、不收 trajectory.npz 等原始数据、不收任何本机绝对路径。

与 v2 的差别（对应 docs/9-1参数定稿_2026-11-18_v3.md）：
  * 收录道口栏杆层（`ped_corridor_layer.cpp` + `plugins.xml` + 构建文件），
    因为它是 v3 论文口径的一部分，别人要能重建；
  * 收录雷达口径对照配置 `nav2_common_lidar.yaml`（A/B 的另一栏）；
  * 收录复现/观测脚本（`run_300.sh`、`run_mixed.sh`、`demo_gui.sh`、`dev_*`、`cleanup_sim.sh`）；
  * 文档只放**最后这版**（v3），没有历史版本对比段。

用法：
    python3 scripts/build_upload_package_v3.py \
        --out "<repo>/9-1_9-3_GitHub上传包" \
        --results out/9-2_official_v3
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import shutil
import sys
from typing import Dict, List, Tuple

WS = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))   # nav_ws_core
ROOT = os.path.dirname(WS)                                          # 仓库根
SCEN = ("static_avoid", "dynamic_pedestrian", "narrow_passage", "dead_end", "mixed")
BASE = ("dwa", "mppi", "teb")

# (源文件（相对 nav_ws_core）, 包内相对路径)
FILES: List[Tuple[str, str]] = [
    # ---- 共享执行层：Nav2 launch + 代价地图/安全层 + 四份控制器参数 ----
    ("src/robot_bringup/launch/nav2_baseline.launch.py",
     "code/robot_bringup/launch/nav2_baseline.launch.py"),
    ("src/robot_bringup/launch/simulation.launch.py",
     "code/robot_bringup/launch/simulation.launch.py"),
    ("src/robot_bringup/config/nav2_common.yaml",
     "code/robot_bringup/config/nav2_common.yaml"),
    ("src/robot_bringup/config/nav2_common_lidar.yaml",
     "code/robot_bringup/config/nav2_common_lidar.yaml"),
    ("src/robot_bringup/config/baselines/dwa.yaml",
     "code/robot_bringup/config/baselines/dwa.yaml"),
    ("src/robot_bringup/config/baselines/mppi.yaml",
     "code/robot_bringup/config/baselines/mppi.yaml"),
    ("src/robot_bringup/config/baselines/teb.yaml",
     "code/robot_bringup/config/baselines/teb.yaml"),
    ("src/robot_bringup/config/baselines/mpc.yaml",
     "code/robot_bringup/config/baselines/mpc.yaml"),
    ("src/robot_bringup/scripts/flat_speed_limiter.py",
     "code/robot_bringup/scripts/flat_speed_limiter.py"),
    ("src/robot_bringup/scripts/safety_supervisor.py",
     "code/robot_bringup/scripts/safety_supervisor.py"),
    # ---- 道口栏杆层（v3 论文口径的核心：行人 → 代价地图）----
    ("src/robot_bringup/src/ped_corridor_layer.cpp",
     "code/robot_bringup/src/ped_corridor_layer.cpp"),
    ("src/robot_bringup/plugins.xml", "code/robot_bringup/plugins.xml"),
    ("src/robot_bringup/CMakeLists.txt", "code/robot_bringup/CMakeLists.txt"),
    ("src/robot_bringup/package.xml", "code/robot_bringup/package.xml"),
    # ---- 场景生成 + 判分 + 采集 + 汇总 ----
    ("src/robot_benchmark/robot_benchmark/scenes.py",
     "code/robot_benchmark/robot_benchmark/scenes.py"),
    ("src/robot_benchmark/robot_benchmark/scenario_gen.py",
     "code/robot_benchmark/robot_benchmark/scenario_gen.py"),
    ("src/robot_benchmark/robot_benchmark/gen_scenarios.py",
     "code/robot_benchmark/robot_benchmark/gen_scenarios.py"),
    ("src/robot_benchmark/robot_benchmark/judge.py",
     "code/robot_benchmark/robot_benchmark/judge.py"),
    ("src/robot_benchmark/robot_benchmark/nav2_driver.py",
     "code/robot_benchmark/robot_benchmark/nav2_driver.py"),
    ("src/robot_benchmark/robot_benchmark/runner.py",
     "code/robot_benchmark/robot_benchmark/runner.py"),
    ("src/robot_benchmark/robot_benchmark/summarize.py",
     "code/robot_benchmark/robot_benchmark/summarize.py"),
    # ---- 仿真桥 ----
    ("src/robot_sim/robot_sim/scripts/isaac_sim_bridge.py",
     "code/robot_sim/robot_sim/scripts/isaac_sim_bridge.py"),
    ("src/robot_sim/robot_sim/nodes/truth_odom_node.py",
     "code/robot_sim/robot_sim/nodes/truth_odom_node.py"),
    ("src/robot_sim/robot_sim/nodes/map_odom_static_node.py",
     "code/robot_sim/robot_sim/nodes/map_odom_static_node.py"),
    ("src/robot_sim/robot_sim/scripts/scene_from_scenario.py",
     "code/robot_sim/robot_sim/scripts/scene_from_scenario.py"),
    # ---- 工具/复现脚本 ----
    ("scripts/env.sh", "code/scripts/env.sh"),
    ("scripts/run_workset.py", "code/scripts/run_workset.py"),
    ("scripts/run_300.sh", "code/scripts/run_300.sh"),
    ("scripts/run_mixed.sh", "code/scripts/run_mixed.sh"),
    ("scripts/freeze_time_limits.py", "code/scripts/freeze_time_limits.py"),
    ("scripts/cleanup_sim.sh", "code/scripts/cleanup_sim.sh"),
    ("scripts/demo_gui.sh", "code/scripts/demo_gui.sh"),
    ("scripts/dev_targets.sh", "code/scripts/dev_targets.sh"),
    ("scripts/dev_launch_only.sh", "code/scripts/dev_launch_only.sh"),
    ("scripts/dev_one_episode.sh", "code/scripts/dev_one_episode.sh"),
    ("scripts/dev_probe_cmd.py", "code/scripts/dev_probe_cmd.py"),
    ("scripts/dev_probe_costmap.py", "code/scripts/dev_probe_costmap.py"),
    ("scripts/dev_dump_costmap.py", "code/scripts/dev_dump_costmap.py"),
    ("scripts/gen_92_tables_v3.py", "code/scripts/gen_92_tables_v3.py"),
    ("scripts/build_upload_package_v3.py", "code/scripts/build_upload_package_v3.py"),
    ("scripts/build_teb_ascii.sh", "code/scripts/build_teb_ascii.sh"),
    ("scripts/build_mpc_ascii.sh", "code/scripts/build_mpc_ascii.sh"),
    # ---- 评分清单与场景模板 ----
    ("scenarios/instances.yaml", "data/instances.yaml"),
    ("scenarios/mixed.yaml", "data/templates/mixed.yaml"),
    ("scenarios/static_avoid.yaml", "data/templates/static_avoid.yaml"),
    ("scenarios/dynamic_pedestrian.yaml", "data/templates/dynamic_pedestrian.yaml"),
    ("scenarios/narrow_passage.yaml", "data/templates/narrow_passage.yaml"),
    ("scenarios/dead_end.yaml", "data/templates/dead_end.yaml"),
]

DOCS = [
    ("docs/9-1参数定稿_2026-11-18_v3.md", "results/9-1参数定稿.md"),
    ("docs/9-2正式成绩_2026-11-18_v3.md", "results/9-2正式成绩.md"),
    ("docs/9-3行人动态障碍.md", "results/9-3行人动态障碍.md"),
    ("docs/9-3时限冻结_执行记录.md", "results/9-3时限冻结_执行记录.md"),
    ("docs/9-1工程执行台账.md", "results/9-1工程执行台账.md"),
    ("docs/工程侧致算法侧说明_9-1与9-2.md", "results/工程侧致算法侧说明.md"),
]


def sha256(path: str) -> str:
    h = hashlib.sha256()
    with open(path, "rb") as fh:
        for chunk in iter(lambda: fh.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def load_summary(results_root: str, baseline: str) -> Dict:
    p = os.path.join(results_root, baseline, "summary.json")
    if not os.path.isfile(p):
        raise SystemExit("缺少汇总：%s（先跑 robot_benchmark.summarize）" % p)
    with open(p, encoding="utf-8") as fh:
        return json.load(fh)


def _ok_n(ps: Dict) -> Tuple[int, int]:
    n = int(ps.get("runs", 0))
    ok = int(round(ps.get("success_rate", 0.0) * n))
    return ok, n


def table(summaries: Dict[str, Dict]) -> str:
    lines = ["| 场景 | DWA | MPPI | TEB |", "|---|---|---|---|"]
    for s in SCEN:
        cells = []
        for b in BASE:
            ps = summaries[b]["per_scenario"].get(s) or {}
            ok, n = _ok_n(ps)
            cells.append("%d/%d = %.0f%%" % (ok, n, 100.0 * ok / max(n, 1)))
        lines.append("| %s | %s |" % (s, " | ".join(cells)))
    tot = []
    for b in BASE:
        ok = sum(_ok_n(summaries[b]["per_scenario"].get(s) or {})[0] for s in SCEN)
        n = sum(_ok_n(summaries[b]["per_scenario"].get(s) or {})[1] for s in SCEN)
        tot.append("%d/%d = %.1f%%" % (ok, n, 100.0 * ok / max(n, 1)))
    lines.append("| **合计** | %s |" % " | ".join(tot))
    return "\n".join(lines)


def detail_table(summaries: Dict[str, Dict]) -> str:
    lines = ["| 场景 | 基线 | 到达 [s] | 路径 [m] | 平滑度 | 间距中位 [m] | 间距最坏 [m] | 急停合计 |",
             "|---|---|---|---|---|---|---|---|"]
    for s in SCEN:
        for b in BASE:
            ps = summaries[b]["per_scenario"].get(s) or {}

            def f(k, nd=2):
                v = ps.get(k)
                return "-" if v is None else ("%." + str(nd) + "f") % v
            lines.append("| %s | %s | %s | %s | %s | %s | %s | %s |" % (
                s, b, f("time_to_goal_s_median"), f("path_length_m_median"),
                f("smoothness_median"), f("min_clearance_m_median", 3),
                f("min_clearance_m_worst", 3), ps.get("estop_count_total", 0)))
    return "\n".join(lines)


def dynamic_suite(summaries: Dict[str, Dict]) -> str:
    out = []
    for b in BASE:
        ps = summaries[b]["per_scenario"]
        n = 0
        ok = 0
        for s in ("dynamic_pedestrian", "mixed"):
            a, c = _ok_n(ps.get(s) or {})
            ok += a
            n += c
        out.append("%s %d/%d = %.1f%%" % (b.upper(), ok, n, 100.0 * ok / max(n, 1)))
    return "；".join(out)


def totals(summaries: Dict[str, Dict]) -> Tuple[int, int]:
    ok = sum(_ok_n(summaries[b]["per_scenario"].get(s) or {})[0] for b in BASE for s in SCEN)
    n = sum(_ok_n(summaries[b]["per_scenario"].get(s) or {})[1] for b in BASE for s in SCEN)
    return ok, n


def rewrite_doc_refs(out: str) -> None:
    """包内文档只引用**包内**文档：把历史/仓库路径的文档名改成本包内的 results/ 名。"""
    subs = [
        ("docs/9-1参数定稿_2026-11-15.md", "results/9-1参数定稿.md"),
        ("docs/9-1参数定稿_2026-11-16_v2.md", "results/9-1参数定稿.md"),
        ("docs/9-1参数定稿_2026-11-18_v3.md", "results/9-1参数定稿.md"),
        ("9-1参数定稿_2026-11-15.md", "9-1参数定稿.md"),
        ("9-1参数定稿_2026-11-16_v2.md", "9-1参数定稿.md"),
        ("9-1参数定稿_2026-11-18_v3.md", "9-1参数定稿.md"),
        ("docs/9-2正式成绩_2026-11-15.md", "results/9-2正式成绩.md"),
        ("docs/9-2正式成绩_2026-11-16.md", "results/9-2正式成绩.md"),
        ("docs/9-2正式成绩_2026-11-18_v3.md", "results/9-2正式成绩.md"),
        ("9-2正式成绩_2026-11-15.md", "9-2正式成绩.md"),
        ("9-2正式成绩_2026-11-16.md", "9-2正式成绩.md"),
        ("9-2正式成绩_2026-11-18_v3.md", "9-2正式成绩.md"),
        ("docs/9-3行人动态障碍.md", "results/9-3行人动态障碍.md"),
        ("docs/9-3时限冻结_执行记录.md", "results/9-3时限冻结_执行记录.md"),
        ("docs/9-1工程执行台账.md", "results/9-1工程执行台账.md"),
        ("docs/工程侧致算法侧说明_9-1与9-2.md", "results/工程侧致算法侧说明.md"),
        ("docs/9-1参数定稿_2026-11-16_v2.md` §4", "results/9-1参数定稿.md` §4"),
    ]
    res = os.path.join(out, "results")
    for fn in os.listdir(res):
        if not fn.endswith(".md"):
            continue
        p = os.path.join(res, fn)
        t = open(p, encoding="utf-8").read()
        t2 = t
        for a, b in subs:
            t2 = t2.replace(a, b)
        if t2 != t:
            open(p, "w", encoding="utf-8").write(t2)


def sanitize_local_paths(out: str) -> None:
    subs = [
        ("<repo>", "<repo>"),
        ("<repo>", "<repo>"),
        ("$HOME", "$HOME"),
    ]
    hits = 0
    for root, _dirs, files in os.walk(out):
        for fn in files:
            if not fn.endswith((".md", ".py", ".sh", ".yaml", ".yml", ".txt", ".cpp", ".xml")):
                continue
            p = os.path.join(root, fn)
            if fn == "MANIFEST.txt":
                continue
            try:
                t = open(p, encoding="utf-8").read()
            except UnicodeDecodeError:
                continue
            t2 = t
            for a, b in subs:
                t2 = t2.replace(a, b)
            if t2 != t:
                open(p, "w", encoding="utf-8").write(t2)
                hits += 1
    if hits:
        print("  已替换本机绝对路径的文件：%d 个" % hits)


def check_no_abs(out: str) -> List[str]:
    bad = []
    for root, _dirs, files in os.walk(out):
        for fn in files:
            p = os.path.join(root, fn)
            if fn == "MANIFEST.txt" or not fn.endswith((".md", ".py", ".sh", ".yaml", ".yml", ".cpp", ".xml")):
                continue
            try:
                t = open(p, encoding="utf-8").read()
            except UnicodeDecodeError:
                continue
            # 只查**本机**路径；`/home/robot/…` 是算法侧机器的合法引用，不算违规
            if "$HOME" in t:
                bad.append(os.path.relpath(p, out))
    return bad


def check_no_history(out: str) -> List[str]:
    """包内文档不得出现"取代 v1/v2"之类的历史版本表述。"""
    bad = []
    pat = ("取代 v1", "取代 `docs/9-1参数定稿_2026-11-15", "取代 `docs/9-2正式成绩_2026-11-15",
           "取代 `docs/9-2正式成绩_2026-11-16", "v1 时", "v1：", "与 v1 的差异")
    for fn in os.listdir(os.path.join(out, "results")):
        if not fn.endswith(".md"):
            continue
        p = os.path.join(out, "results", fn)
        t = open(p, encoding="utf-8").read()
        for s in pat:
            if s in t:
                bad.append("%s: %s" % (fn, s))
    return bad


def main(argv=None) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", required=True, help="目标文件夹（会被清空重建）")
    ap.add_argument("--results", required=True, help="正式采集根目录（含 <baseline>/summary.json）")
    args = ap.parse_args(argv)

    out = os.path.abspath(args.out)
    results = os.path.abspath(args.results)
    summaries = {b: load_summary(results, b) for b in BASE}

    if os.path.isdir(out):
        shutil.rmtree(out)
    os.makedirs(out)

    copied = 0
    for src_rel, dst_rel in FILES:
        src = os.path.join(WS, src_rel)
        if not os.path.isfile(src):
            raise SystemExit("找不到源文件：%s" % src)
        dst = os.path.join(out, dst_rel)
        os.makedirs(os.path.dirname(dst), exist_ok=True)
        shutil.copy2(src, dst)
        copied += 1

    for src_rel, dst_rel in DOCS:
        src = os.path.join(ROOT, src_rel)
        if not os.path.isfile(src):
            raise SystemExit("找不到文档：%s" % src)
        dst = os.path.join(out, dst_rel)
        os.makedirs(os.path.dirname(dst), exist_ok=True)
        shutil.copy2(src, dst)
        copied += 1

    for b in BASE:
        for ext in ("md", "json"):
            src = os.path.join(results, b, "summary." + ext)
            if not os.path.isfile(src):
                raise SystemExit("缺少 %s" % src)
            shutil.copy2(src, os.path.join(out, "results", "summary_%s.%s" % (b, ext)))
            copied += 1

    ok, n = totals(summaries)
    readme = """# 9-1 / 9-2 / 9-3 工程侧交付包（2026-11-18，道口栏杆版）

本包只含**主代码 + 配置 + 评分清单 + 结果文档与汇总**，不含构建产物、原始
`trajectory.npz`、Isaac 场景 `.usda/.pgm`、任何本机绝对路径。

## 0. 这是什么

- **9-1**：基线接入与参数定稿（共享执行层、三基线参数、冻结 sha256）。
- **9-2**：正式成绩入库（3 基线 × 5 场景 × 20 种子 = 300 run）。
- **9-3**：行人动态障碍（道口栏杆机制）+ 时限冻结
  （`T_s = max(T_min,s , 1.5·max_j t̃(s,j))`，只用成功 run）。

## 1. 目录

```
README.md                     本文件
MANIFEST.txt                  逐文件 sha256（上传前核对用）
code/robot_bringup/           共享执行层：Nav2 launch、代价地图/安全层配置、四份控制器参数、
                              道口栏杆层 ped_corridor_layer.cpp（+ plugins.xml / CMakeLists / package.xml）
code/robot_benchmark/         场景生成、判分（judge）、采集（nav2_driver/runner）、汇总（summarize）
code/robot_sim/               Isaac Sim 桥、真值 odom、map→odom 静态 TF、场景导出
code/scripts/                 批量采集（run_workset / run_300）、口径切换（run_mixed）、
                              观测工具（demo_gui / dev_* / cleanup_sim）、9-3 时限冻结
data/instances.yaml           100 条冻结实例（含 9-3 时限 T_s）
data/templates/               5 个场景模板（重建 instances.yaml 用）
results/                      冻结快照、正式成绩、执行台账、9-3 行人动态障碍与时限冻结说明、
                              各基线 summary.md（含逐条失败 run 异常清单）
```

## 2. 正式成绩（5 场景 × 20 seed × 3 基线 = 300 run）

%s

全体 **%d/%d = %.1f%%**。

套件级 `dynamic_stress`（dynamic_pedestrian + mixed）：%s。

## 3. 指标明细（各场景 20 run 中位 / 最坏）

%s

## 4. 怎么复现

```bash
# 1) 环境
source nav_ws_core/scripts/env.sh                    # 系统 ROS 2 Humble
export ISAAC_PYTHON=$HOME/miniconda3/envs/env_isaaclab/bin/python3
source ~/teb_prebuilt/env.sh && source ~/teb_build_ws/install/setup.bash   # TEB
bash nav_ws_core/scripts/build.sh                    # 含自研代价地图层 ped_corridor_layer

# 2) 一次性 300 run（每场景生成 .usda + .pgm/.yaml，三基线同栈跑完；可续跑）
bash nav_ws_core/scripts/run_300.sh 9-2_official_v3

# 3) 只想看 mixed（走廊口径 / 雷达口径 A/B）
bash nav_ws_core/scripts/run_mixed.sh mixed_v3
bash nav_ws_core/scripts/run_mixed.sh mixed_v3_lidar nav2_common_lidar.yaml

# 4) 在 Isaac GUI 里逐基线看动态避障（一条命令一个基线）
bash nav_ws_core/scripts/demo_gui.sh mixed_01 dwa     # 换成 mppi / teb 验证另外两个

# 5) 9-3 时限复核（dry-run，不会改 instances.yaml）
python3 nav_ws_core/scripts/freeze_time_limits.py --workset out/9-2_official_v3
```

## 5. 口径提醒（写论文/汇报时照这四条说）

- 速度契约：平面速度 `√(vx²+vy²) ≤ 0.6 m/s`、`|wz| ≤ 1.5 rad/s`、加速度 1.0/1.0/2.0 m/s²、
  控制 20 Hz、到达判据 0.25 m / 0.25 rad（控制器**内部**收到 0.18 m / 0.15 rad，见 9-1 §2.1）。
- 失败分类：`collision`（与行人最小间距 ≤ 0）、`timeout`（超 `time_limit_s`）、
  `other`（其余，主要为卡死/局部极小）。
- 行人：契约给定匀速直线轨迹（`x=x_cross`、`y=dir·speed·(t−t_meet)`）。v3 把行人的
  **横穿走廊**当作"道口栏杆"注入局部代价地图（`/ped/corridor` → `PedCorridorLayer`），
  三个基线共用同一份信息；判分**不看**代价地图，只看轨迹与契约几何。
- 每份 `summary.md` 都带逐条失败 run 的异常清单，不得静默丢弃失败。
""" % (table(summaries), ok, n, 100.0 * ok / max(n, 1), dynamic_suite(summaries),
       detail_table(summaries))

    with open(os.path.join(out, "README.md"), "w", encoding="utf-8") as fh:
        fh.write(readme)
    copied += 1

    rewrite_doc_refs(out)
    sanitize_local_paths(out)

    bad = check_no_abs(out)
    if bad:
        raise SystemExit("包内仍有本机绝对路径：%s" % bad)
    hist = check_no_history(out)
    if hist:
        print("  ! 包内文档含历史版本表述（将保留，请人工确认）：%s" % hist)

    mf = []
    for dirpath, _dirnames, filenames in os.walk(out):
        for fn in sorted(filenames):
            p = os.path.join(dirpath, fn)
            rel = os.path.relpath(p, out)
            if rel == "MANIFEST.txt":
                continue
            mf.append("%s  %s" % (sha256(p), rel))
    with open(os.path.join(out, "MANIFEST.txt"), "w", encoding="utf-8") as fh:
        fh.write("# sha256  相对路径\n" + "\n".join(sorted(mf, key=lambda x: x.split()[-1])) + "\n")

    size = sum(os.path.getsize(os.path.join(dp, f))
               for dp, _dn, fns in os.walk(out) for f in fns)
    print("打包完成：%s" % out)
    print("  文件 %d 个（另加 MANIFEST.txt），合计 %.1f KB" % (copied + 1, size / 1024.0))
    print("  正式成绩：%d/%d = %.1f%%" % (ok, n, 100.0 * ok / max(n, 1)))
    return 0


if __name__ == "__main__":
    sys.exit(main())
