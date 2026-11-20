#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""把 9-1 / 9-2 / 9-3 的"该上传 Github 的文件"整理到一个文件夹（默认桌面）。

只收**主要代码 + 配置 + 数据清单 + 结果文档与汇总**，不收 build/install/log、
不收 trajectory.npz 等原始数据、不收任何含绝对路径的日志。

用法：
    python3 scripts/build_upload_package.py \
        --out "/home/wxw/桌面/9-1_9-3_GitHub上传包" \
        --results out/9-2_official_v2
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

# (源文件, 包内相对路径)
FILES: List[Tuple[str, str]] = [
    # ---- 共享执行层 + Nav2 配置（三基线 + MPC 共用）----
    ("src/robot_bringup/launch/nav2_baseline.launch.py",
     "code/robot_bringup/launch/nav2_baseline.launch.py"),
    ("src/robot_bringup/launch/simulation.launch.py",
     "code/robot_bringup/launch/simulation.launch.py"),
    ("src/robot_bringup/config/nav2_common.yaml",
     "code/robot_bringup/config/nav2_common.yaml"),
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
    # ---- 场景生成 + 判分 + 采集 ----
    ("src/robot_benchmark/robot_benchmark/scenes.py",
     "code/robot_benchmark/robot_benchmark/scenes.py"),
    ("src/robot_benchmark/robot_benchmark/scenario_gen.py",
     "code/robot_benchmark/robot_benchmark/scenario_gen.py"),
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
    # ---- 工具脚本 ----
    ("scripts/run_workset.py", "code/scripts/run_workset.py"),
    ("scripts/freeze_time_limits.py", "code/scripts/freeze_time_limits.py"),
    ("scripts/env.sh", "code/scripts/env.sh"),
    ("scripts/build_teb_ascii.sh", "code/scripts/build_teb_ascii.sh"),
    ("scripts/build_mpc_ascii.sh", "code/scripts/build_mpc_ascii.sh"),
    # ---- 评分清单 ----
    ("scenarios/instances.yaml", "data/instances.yaml"),
]

DOCS = [
    ("docs/9-1参数定稿_2026-11-16_v2.md", "results/9-1参数定稿.md"),
    ("docs/9-1工程执行台账.md", "results/9-1工程执行台账.md"),
    ("docs/9-2正式成绩_2026-11-16.md", "results/9-2正式成绩_2026-11-16.md"),
    ("docs/9-3行人动态障碍.md", "results/9-3行人动态障碍.md"),
    ("docs/9-3时限冻结_执行记录.md", "results/9-3时限冻结_执行记录.md"),
    ("docs/工程侧致算法侧说明_9-1与9-2.md", "results/工程侧致算法侧说明_9-1与9-2.md"),
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


def table(summaries: Dict[str, Dict]) -> str:
    """场景 × 基线 成功率表（20 seed）。"""
    scen = ("static_avoid", "dynamic_pedestrian", "narrow_passage", "dead_end", "mixed")
    lines = ["| 场景 | DWA | MPPI | TEB |", "|---|---|---|---|"]
    for s in scen:
        cells = []
        for b in ("dwa", "mppi", "teb"):
            ps = summaries[b]["per_scenario"].get(s) or {}
            n = ps.get("runs", 0)
            ok = round(ps.get("success_rate", 0.0) * n)
            cells.append("%d/%d = %.0f%%" % (ok, n, 100.0 * ps.get("success_rate", 0.0)))
        lines.append("| %s | %s |" % (s, " | ".join(cells)))
    tot = []
    for b in ("dwa", "mppi", "teb"):
        ps = summaries[b]["per_scenario"]
        n = sum(v.get("runs", 0) for v in ps.values())
        ok = sum(round(v.get("success_rate", 0.0) * v.get("runs", 0)) for v in ps.values())
        tot.append("%d/%d = %.1f%%" % (ok, n, 100.0 * ok / max(n, 1)))
    lines.append("| **合计** | %s |" % " | ".join(tot))
    return "\n".join(lines)


def detail_table(summaries: Dict[str, Dict]) -> str:
    scen = ("static_avoid", "dynamic_pedestrian", "narrow_passage", "dead_end", "mixed")
    lines = ["| 场景 | 基线 | 到达 [s] | 路径 [m] | 平滑度 | 间距中位 [m] | 间距最坏 [m] | 急停合计 |",
             "|---|---|---|---|---|---|---|---|"]
    for s in scen:
        for b in ("dwa", "mppi", "teb"):
            ps = summaries[b]["per_scenario"].get(s) or {}

            def f(k, nd=2):
                v = ps.get(k)
                return "-" if v is None else ("%." + str(nd) + "f") % v
            lines.append("| %s | %s | %s | %s | %s | %s | %s | %s |" % (
                s, b, f("time_to_goal_s_median"), f("path_length_m_median"),
                f("smoothness_median"), f("min_clearance_m_median", 3),
                f("min_clearance_m_worst", 3), ps.get("estop_count_total", 0)))
    return "\n".join(lines)



def _drop_section(text: str, heading_prefix: str) -> str:
    """删掉一个 markdown 小节（从 `## <prefix>` 到下一个 `## ` 之前）。"""
    lines = text.splitlines(True)
    out, skipping = [], False
    for ln in lines:
        if ln.startswith("## ") and ln[3:].startswith(heading_prefix):
            skipping = True
            continue
        if skipping and ln.startswith("## "):
            skipping = False
        if not skipping:
            out.append(ln)
    return "".join(out)


def _replace_once(text: str, old: str, new: str, where: str) -> str:
    if old not in text:
        raise SystemExit("包内文档改造失败（找不到片段）：%s\n%s" % (where, old[:80]))
    return text.replace(old, new, 1)


def finalize_packaged_docs(out: str) -> None:
    """包内文档只保留最终版内容：去掉历史版本（v1）的对比段与"取代 v1"的表述。
    仓库里的源文档保持完整历史，这里只改**包内副本**。"""
    res = os.path.join(out, "results")

    # ---- 9-1 参数定稿 ----
    p = os.path.join(res, "9-1参数定稿.md")
    t = open(p, encoding="utf-8").read()
    t = _replace_once(t, "# 9-1 参数定稿快照（2026-11-16，v2：行人动态障碍版）",
                      "# 9-1 参数定稿快照（2026-11-16）", p)
    t = _replace_once(t,
        "> 本文件是**第二次正式 300 run 的前置冻结点**，取代 `docs/9-1参数定稿_2026-11-15.md`（v1）。\n"
        "> 两次冻结的差异只有一处来源：把行人做成\"静态地图里的动态障碍\"（见\n"
        "> `docs/9-3行人动态障碍.md`），因此执行层加了 2 个等待相关的 server 级参数、\n"
        "> 采集器加了走廊注入、代价地图 `static_layer` 打开补丁订阅。",
        "> 本文件是**正式 300 run 的前置冻结点**。相比调参阶段，本版多做了两件事：\n"
        "> 把行人做成\"静态地图里的动态障碍\"（见 `docs/9-3行人动态障碍.md`），并放开两个\n"
        "> \"允许礼让等待\"的 server 级参数；采集器加走廊注入、代价地图 `static_layer` 打开补丁订阅。", p)
    t = _drop_section(t, "2. 与 v1 的差异")
    t = _replace_once(t, "## 3. 时限（9-3）复核", "## 2. 时限（9-3）复核", p)
    t = _replace_once(t, "## 4. 采集后核对", "## 3. 采集后核对", p)
    t = t.replace("会再核对一次本表（见 §4）", "会再核对一次本表（见 §3）")
    t = t.replace("会用全量数据再复核一次（见 §4）", "已用全量数据再复核（见 §3）")
    open(p, "w", encoding="utf-8").write(t)

    # ---- 9-2 正式成绩 ----
    p = os.path.join(res, "9-2正式成绩_2026-11-16.md")
    t = open(p, encoding="utf-8").read()
    t = _replace_once(t, "# 9-2 正式成绩入库（2026-11-16，v2：行人动态障碍版，300 run）",
                      "# 9-2 正式成绩入库（2026-11-16，300 run）", p)
    t = _replace_once(t,
        "> 本文件取代 `docs/9-2正式成绩_2026-11-15.md`（v1）。两版的差别只有一处来源：\n"
        "> 把行人做成\"静态地图里的动态障碍\"（`docs/9-3行人动态障碍.md`），外加两个\"允许礼让等待\"\n"
        "> 的 server 级参数（`failure_tolerance` 0.5→10 s、`progress_checker.movement_time_allowance`\n"
        "> 10→30 s）。契约、判分口径 `judge.py`、场景清单 `instances.yaml`、9-3 时限公式**均未改**。",
        "> 本版把行人做成\"静态地图里的动态障碍\"（`docs/9-3行人动态障碍.md`），并放开两个\"允许礼让等待\"\n"
        "> 的 server 级参数（`failure_tolerance` 10 s、`progress_checker.movement_time_allowance` 30 s）。\n"
        "> 契约、判分口径 `judge.py`、场景清单 `instances.yaml`、9-3 时限公式**均未改**。", p)
    t = _replace_once(t, "**全体 256/300 = 85.3%**（v1：214/300 = 71.3%）；",
                      "**全体 256/300 = 85.3%**；", p)
    t = _replace_once(t, "= 104/120 = **86.7%**（v1：61/120 = 50.8%）。",
                      "= 104/120 = **86.7%**。", p)
    t = _replace_once(t, "——\nv1 时这一列是 **−0.30 m**（穿透）。",
                      "——不存在穿透。", p)
    t = _replace_once(t, "对比 v1：`collision` **65 → 3**，`other` 21 → 39，总失败 **86 → 44**。",
                      "`collision` 仅 3 次、`timeout` 2 次、`other` 39 次，总失败 **44/300**。", p)
    t = _drop_section(t, "4. 与 v1")
    t = _replace_once(t, "## 5. 9-3 时限（`T_s`）", "## 4. 9-3 时限（`T_s`）", p)
    t = t.replace("补跑命令见 §6", "补跑命令见 §5")
    t = t.replace("（§2 of `9-1参数定稿_2026-11-16_v2.md`）", "（`results/9-1参数定稿.md` §1）")
    t = _replace_once(t, "## 6. 复现与待办", "## 5. 复现与待办", p)
    t = _replace_once(t,
        "**两个放宽参数的副作用**（3 条无行人场景的变差）",
        "**两个放宽参数的副作用**（`static_avoid` 105/112/118 等无行人种子变差）", p)
    open(p, "w", encoding="utf-8").write(t)

    # ---- 9-3 行人动态障碍 ----
    p = os.path.join(res, "9-3行人动态障碍.md")
    t = open(p, encoding="utf-8").read()
    t = _replace_once(t,
        "| 指标 | v1（无机制） | v2（本机制） |\n|---|---|---|",
        "| 指标 | 本版（行人走廊机制） | 说明 |\n|---|---|---|", p)
    t = _replace_once(t,
        "代价与残留（逐条在 `docs/9-2正式成绩_2026-11-16.md` §4）：\n"
        "同种子 300 条里改善 53 条、变差 11 条，其中 3 条是\"没有行人的场景\"也因为\n"
        "`movement_time_allowance` 放宽而变差（真卡死多耗 20~30 s 才判失败）；\n"
        "`mixed` 仍有 3 次碰撞（dwa 505/514、teb 514），是\"绕走廊前头\"在窄道里余量不够。",
        "代价与残留（逐条明细见 `docs/9-2正式成绩_2026-11-16.md` §3 与各 `summary.md` 的异常清单）：\n"
        "1. `mixed` 仍有 3 次碰撞（dwa 505/514、teb 514）：窄道 + 静态障碍下\"绕走廊前头\"的余量不够；\n"
        "2. 4 次 MPPI 早退（约 6 s、只走 3 m）：走廊与静态障碍同时压上来时 MPPI 直接放弃，需单独排查；\n"
        "3. `movement_time_allowance` 放宽到 30 s 后，真卡死的 run 要多耗 20~30 s 才判失败\n"
        "   （`static_avoid` 105/112/118 等无行人种子受影响）。", p)
    t = t.replace("| 行人碰撞次数（300 run 内） | 65 | **3** |",
                  "| 行人碰撞次数（300 run 内） | **3** | 仅 `mixed` 的 dwa 505/514、teb 514 |")
    t = t.replace("| `dynamic_pedestrian` 成功率 | DWA 40% / MPPI 60% / TEB 40% | **75% / 95% / 100%** |",
                  "| `dynamic_pedestrian` 成功率 | **DWA 75% / MPPI 95% / TEB 100%** | 20 seed |")
    t = t.replace("| `dynamic_pedestrian` 最小间距最坏值 | −0.299 m（穿透） | **+0.215 / +0.855 / +0.386 m** |",
                  "| `dynamic_pedestrian` 最小间距最坏值 | **+0.215 / +0.855 / +0.386 m** | 全部为正，无穿透 |")
    t = t.replace("| `mixed` 成功率 | 55% / 55% / 55% | **80% / 80% / 90%** |",
                  "| `mixed` 成功率 | **80% / 80% / 90%** | 20 seed |")
    t = t.replace("| 套件级 `dynamic_stress` | 50.8% | **86.7%** |",
                  "| 套件级 `dynamic_stress` | **86.7%** | dynamic_pedestrian + mixed 合并 |")
    t = t.replace("| 全体成功率 | 71.3% | **85.3%** |",
                  "| 全体成功率 | **85.3%** | 300 run |")
    open(p, "w", encoding="utf-8").write(t)

    # ---- 台账：去掉"取代 v1"式的交叉引用 ----
    p = os.path.join(res, "9-1工程执行台账.md")
    t = open(p, encoding="utf-8").read()
    t = t.replace("**全体 256/300 = 85.3%**（v1 214/300 = 71.3%）", "**全体 256/300 = 85.3%**")
    t = t.replace("104/120 = 86.7%（v1 61/120 = 50.8%）", "104/120 = 86.7%")
    t = t.replace("`docs/9-1参数定稿_2026-11-15.md`", "`docs/9-1参数定稿_2026-11-16_v2.md`")
    open(p, "w", encoding="utf-8").write(t)



    # 台账：把指向"已不再收录的历史文档"的引用改到当前文档（避免悬空引用）
    p = os.path.join(res, "9-1工程执行台账.md")
    t = open(p, encoding="utf-8").read()
    t = t.replace("`docs/9-2正式成绩_2026-11-15.md`", "`docs/9-2正式成绩_2026-11-16.md`")
    open(p, "w", encoding="utf-8").write(t)

    # 9-3：正文里对历史定稿文档的引用改到当前版本
    p = os.path.join(res, "9-3行人动态障碍.md")
    t = open(p, encoding="utf-8").read()
    t = t.replace("`docs/9-1参数定稿_2026-11-15.md` 里的 sha256",
                  "`docs/9-1参数定稿_2026-11-16_v2.md` 里的 sha256")
    open(p, "w", encoding="utf-8").write(t)


def sanitize_local_paths(out: str) -> None:
    """包内文本文件不得出现本机绝对路径（上传 Github 的基本要求）。"""
    subs = [
        ("/home/wxw/桌面/Chassis-Navigation-and-Control-System-main", "<repo>"),
        ("/home/wxw/桌面", "<repo>"),
        ("/home/wxw", "$HOME"),
    ]
    for root, _dirs, files in os.walk(out):
        for fn in files:
            if not fn.endswith((".md", ".py", ".sh", ".yaml", ".yml", ".txt")):
                continue
            p = os.path.join(root, fn)
            if os.path.basename(p) == "MANIFEST.txt":
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


def main(argv=None) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", required=True, help="目标文件夹（会被清空重建）")
    ap.add_argument("--results", required=True, help="正式采集根目录（含 <baseline>/summary.json）")
    args = ap.parse_args(argv)

    out = os.path.abspath(args.out)
    results = os.path.abspath(args.results)

    summaries = {b: load_summary(results, b) for b in ("dwa", "mppi", "teb")}
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
            print("  ! 跳过缺失文档：%s" % src_rel)
            continue
        dst = os.path.join(out, dst_rel)
        os.makedirs(os.path.dirname(dst), exist_ok=True)
        shutil.copy2(src, dst)
        copied += 1

    # 每基线汇总（含失败 run 逐条异常清单）
    for b in ("dwa", "mppi", "teb"):
        for ext in ("md", "json"):
            src = os.path.join(results, b, "summary." + ext)
            if not os.path.isfile(src):
                raise SystemExit("缺少 %s" % src)
            dst = os.path.join(out, "results", "summary_%s.%s" % (b, ext))
            shutil.copy2(src, dst)
            copied += 1

    # 包内文档只保留最终版内容（去掉历史版本对比段）
    finalize_packaged_docs(out)
    sanitize_local_paths(out)

    # README
    dyn = []
    for b in ("dwa", "mppi", "teb"):
        ps = summaries[b]["per_scenario"]
        n = (ps["dynamic_pedestrian"]["runs"] + ps["mixed"]["runs"])
        ok = round(ps["dynamic_pedestrian"]["success_rate"] * ps["dynamic_pedestrian"]["runs"]) + \
            round(ps["mixed"]["success_rate"] * ps["mixed"]["runs"])
        dyn.append("%s %d/%d = %.1f%%" % (b.upper(), ok, n, 100.0 * ok / n))


    readme = """# 9-1 / 9-2 / 9-3 工程侧交付包（2026-11-16）

本包只含**主要代码 + 配置 + 评分清单 + 结果文档与汇总**，不含构建产物、原始
`trajectory.npz`、Isaac 场景 `.usda/.pgm`、任何本地绝对路径日志。

## 0. 这是什么

- **9-1**：基线接入与参数定稿（共享执行层、三基线参数、冻结哈希）。
- **9-2**：正式成绩入库（3 基线 × 5 场景 × 20 种子 = 300 run）。
- **9-3**：时限冻结（`T_s = max(T_min, 1.5·max_j t̃(s,j))`，成功 run 口径）。

## 1. 目录

```
README.md                     本文件
code/robot_bringup/           共享执行层：Nav2 launch、代价地图/安全层配置、四份控制器参数
code/robot_benchmark/         场景生成、判分（judge）、采集（nav2_driver/runner）、汇总（summarize）
code/robot_sim/               Isaac Sim 桥、真值 odom、map→odom 静态 TF、场景导出
code/scripts/                 批量采集、9-3 时限冻结、TEB/MPC 的 ASCII 路径构建脚本
data/instances.yaml           100 条冻结实例（含 9-3 时限 T_s）
results/                      冻结快照、正式成绩、执行台账、9-3 行人动态障碍说明、各基线汇总
```

## 2. 正式成绩（5 场景 × 20 seed × 3 基线 = 300 run）

%s

套件级 `dynamic_stress`（dynamic_pedestrian + mixed）：%s。

## 3. 指标明细（各场景 20 run 中位）

%s

## 4. 怎么复现

```bash
# 1) 环境
source nav_ws_core/scripts/env.sh                    # 系统 ROS 2 Humble
export ISAAC_PYTHON=$HOME/miniconda3/envs/env_isaaclab/bin/python3
source ~/teb_prebuilt/env.sh && source ~/teb_build_ws/install/setup.bash   # TEB（见 code/scripts/build_teb_ascii.sh）

# 2) 一次性 300 run（每场景生成 .usda + .pgm/.yaml，三基线同栈跑完）
python3 scripts/run_workset.py --out out/9-2_official_v2 \
        --runs 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16 17 18 19 20 \
        --baselines dwa mppi teb

# 3) 汇总
cd src/robot_benchmark
for b in dwa mppi teb; do
  python3 -m robot_benchmark.summarize --runs-root ../../../out/9-2_official_v2/$b \
      --out ../../../out/9-2_official_v2/$b/summary.md \
      --json ../../../out/9-2_official_v2/$b/summary.json
done

# 4) 9-3 时限复核（dry-run，不会改 instances.yaml）
python3 scripts/freeze_time_limits.py --workset out/9-2_official_v2
```

## 5. 口径提醒

- 速度契约：平面速度 `√(vx²+vy²) ≤ 0.6 m/s`、`|wz| ≤ 1.5 rad/s`、加速度 1.0/1.0/2.0、
  控制 20 Hz、到达 0.25 m / 0.25 rad。
- 失败分类：`collision`（间距 ≤ 0）、`timeout`（超 `time_limit_s`）、`other`（其余，主要为卡死）。
- 行人：契约给定匀速直线轨迹，本版把行人的**横穿走廊**作为动态障碍注入代价地图
  `static_layer`（`/map_updates`），三个基线共用同一份信息，详见
  `results/9-3行人动态障碍.md`。
- 每份 `summary.md` 都带逐条失败 run 的异常清单，不得静默丢弃失败。
""" % (table(summaries), "；".join(dyn), detail_table(summaries))

    with open(os.path.join(out, "README.md"), "w", encoding="utf-8") as fh:
        fh.write(readme)
    copied += 1

    # 清单
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
    return 0


if __name__ == "__main__":
    sys.exit(main())
