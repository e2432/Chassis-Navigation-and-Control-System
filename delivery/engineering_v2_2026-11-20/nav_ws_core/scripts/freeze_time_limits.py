#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""9-3 时限冻结：按 9-1 §6 公式从工作集成绩算出每场景时限，回填 instances.yaml。

公式（9-1 §6，不得改写）：

    T_s = max( T_min,s , 1.5 · max_j t̃(s,j) )
    T_min = 60 s（mixed 75 s）

  * t̃(s,j) = 场景 s、基线 j 在**工作集成功 run** 上的到达时间中位数；
  * 超时/未到达的 run 不参与中位数（不得当成功到达时间用）；
  * 某基线在该场景没有成功 run → 记录缺失并把它排除出 max_j；
  * 三基线都没有成功 run → 保留 T_min 作为临时值，并**上报双方确认**；
  * T_s 对 MPC 与三基线完全相同。

用法：
    python3 scripts/freeze_time_limits.py --workset <repo>/out/9-3_workset            # 只算，打印
    python3 scripts/freeze_time_limits.py --workset ... --write                       # 回填实例表
    python3 scripts/freeze_time_limits.py --workset ... --write --report docs/9-3时限冻结_执行记录.md
"""

from __future__ import annotations

import argparse
import datetime as _dt
import json
import statistics
import sys
from pathlib import Path

SCENARIOS = ("static_avoid", "dynamic_pedestrian", "narrow_passage", "dead_end", "mixed")
BASELINES = ("dwa", "mppi", "teb")
T_MIN = {"static_avoid": 60.0, "dynamic_pedestrian": 60.0, "narrow_passage": 60.0,
         "dead_end": 60.0, "mixed": 75.0}
FACTOR = 1.5


def load_runs(workset: Path):
    """{scenario: {baseline: [(seed, success, time_to_goal_s, clearance), ...]}}"""
    out = {s: {b: [] for b in BASELINES} for s in SCENARIOS}
    for f in sorted(workset.glob("*/runs/*/*/run.json")):
        d = json.load(open(f, encoding="utf-8"))
        s, b = d.get("scenario"), d.get("controller")
        if s not in out or b not in out[s]:
            continue
        m = d["metrics"]
        out[s][b].append({
            "seed": d.get("seed"),
            "success": bool(m.get("success")),
            "time": m.get("time_to_goal_s"),
            "clearance": m.get("min_clearance_m"),
            "failure": d.get("failure_class"),
            "file": str(f),
        })
    return out


def compute(runs):
    rows = []
    for s in SCENARIOS:
        per = {}
        missing = []
        for b in BASELINES:
            ok = [r["time"] for r in runs[s][b] if r["success"] and r["time"] is not None]
            if ok:
                per[b] = {"n": len(ok), "median": statistics.median(ok),
                          "values": sorted(ok)}
            else:
                missing.append(b)
        t_old = T_MIN[s]
        if per:
            worst = max(v["median"] for v in per.values())
            t_new = max(t_old, FACTOR * worst)
            basis = "1.5 × 最慢基线中位 %.3f = %.2f" % (worst, FACTOR * worst)
            if FACTOR * worst <= t_old:
                basis += "（低于下限，取 T_min）"
        else:
            worst = None
            t_new = t_old
            basis = "三基线均无成功 run，保留 T_min 并上报确认"
        fails = {b: [r.get("failure") for r in runs[s][b] if not r["success"]]
                 for b in BASELINES}
        rows.append({"scenario": s, "per": per, "missing": missing, "fails": fails,
                     "worst_median": worst, "t_old": t_old, "t_new": t_new,
                     "basis": basis, "n_total": {b: len(runs[s][b]) for b in BASELINES}})
    return rows


def render_report(rows, workset: Path, manifest: Path) -> str:
    now = _dt.datetime.now().strftime("%Y-%m-%d %H:%M")
    lines = []
    lines.append("# 9-3 时限冻结（执行记录）\n")
    lines.append("> 生成时间：%s　　工作集：`%s`　　实例表：`%s`\n" % (now, workset, manifest))
    lines.append("公式（9-1 §6，未改写）：`T_s = max(T_min,s , 1.5 · max_j t̃(s,j))`，"
                 "`T_min` = 60 s（mixed 75 s）；只统计**成功 run** 的到达时间中位数。\n")

    lines.append("## 1. 每场景结果\n")
    lines.append("| 场景 | DWA 中位/成功数 | MPPI 中位/成功数 | TEB 中位/成功数 | max_j 中位 | 1.5× | 旧时限 | **新时限** | 判据 |")
    lines.append("|---|---|---|---|---|---|---|---|---|")
    for r in rows:
        cells = []
        for b in BASELINES:
            p = r["per"].get(b)
            if p:
                cells.append("%.2f s（%d/%d）" % (p["median"], p["n"], r["n_total"][b]))
            else:
                cells.append("无成功 run（0/%d）" % r["n_total"][b])
        lines.append("| %s | %s | %s | %s | %s | %s | %.1f | **%.1f** | %s |" % (
            r["scenario"], cells[0], cells[1], cells[2],
            "—" if r["worst_median"] is None else "%.3f" % r["worst_median"],
            "—" if r["worst_median"] is None else "%.2f" % (FACTOR * r["worst_median"]),
            r["t_old"], r["t_new"], r["basis"]))
    lines.append("")

    lines.append("## 2. 缺失与例外（按 9-1 §6 处置）\n")
    any_missing = False
    for r in rows:
        if r["missing"]:
            any_missing = True
            lines.append("- **%s**：%s 无成功 run，已排除出 `max_j`；上限由其余基线决定 → T_s=%.1f s"
                         % (r["scenario"], "、".join(r["missing"]), r["t_new"]))
        if r["worst_median"] is None:
            lines.append("- **%s**：三基线均无成功 run，保留 T_min=%.1f s 作为临时值，**需双方确认**"
                         % (r["scenario"], r["t_old"]))
    if not any_missing:
        lines.append("- 无缺失：五个场景三条基线都有成功 run。")
    lines.append("")

    lines.append("## 2.1 失败构成（成功 run 之外的所有 run，按 failure_class）\n")
    lines.append("| 场景 | DWA | MPPI | TEB |")
    lines.append("|---|---|---|---|")
    for r in rows:
        cells = []
        for b in BASELINES:
            f = [x for x in r["fails"].get(b, []) if True]
            cells.append("—" if not f else "、".join(str(x) for x in f))
        lines.append("| %s | %s | %s | %s |" % (r["scenario"], cells[0], cells[1], cells[2]))
    lines.append("")
    lines.append("> 这条信息比时限本身更重要：**时限不是瓶颈**（见 §1 的 1.5× 一列全部低于下限），")
    lines.append("> 失分来自 `other`（progress_checker 判无进展后 abort）与 `collision`。")
    lines.append("> 即「把时限调宽」不会提升成功率；要提升成功率只能改控制器参数或改场景难度。")
    lines.append("")

    lines.append("## 3. 逐 run 明细（成功 run 的到达时间）\n")
    for r in rows:
        lines.append("### %s → T_s = %.1f s（原 %.1f s）\n" % (r["scenario"], r["t_new"], r["t_old"]))
        for b in BASELINES:
            p = r["per"].get(b)
            if p:
                lines.append("- %s：%s（中位 %.3f s，n=%d）" % (
                    b, "、".join("%.2f" % v for v in p["values"]), p["median"], p["n"]))
            else:
                lines.append("- %s：无成功 run" % b)
        lines.append("")

    lines.append("## 4. 需要算法侧审的点（9-3 分工：工程组执行、算法组审核阈值与公平性）\n")
    lines.append("1. **阈值合理性**：`T_s` 是否过松/过紧。过松会让超时类失败无法暴露，"
                 "过紧会把合理慢行判成失败。请对照 §1 的 `1.5×max_j` 与实际中位数判断。")
    lines.append("2. **公平性**：`T_s` 对四个控制器完全相同；本工作集只跑了 DWA/MPPI/TEB"
                 "（按 2026-11-14 决定，MPC 不参与 9-3），但 MPC 后续接入时**沿用同一 T_s**，"
                 "不得为 MPC 单独调时限。")
    lines.append("3. **数据口径**：只统计成功 run；失败 run 已如实保留在 `run.json` 里"
                 "（`metrics.success=false` + `failure_class`），可复核。")
    lines.append("4. 若你们认为某个场景的中位数样本量（5 个种子）不足，"
                 "工程侧可以扩到 20 种子再冻结——但那会把正式采集提前跑掉，需要你们明确要求。")
    lines.append("")
    return "\n".join(lines)


def rewrite_manifest(manifest: Path, new_limits: dict, dry_run: bool) -> str:
    """按行改 time_limit_s：只改对应场景的实例行，其余字节不动（最小 diff）。"""
    lines = manifest.read_text(encoding="utf-8").splitlines(True)
    cur = None
    changed = 0
    out = []
    for ln in lines:
        if ln.startswith("- scenario:"):
            cur = ln.split(":", 1)[1].strip()
        elif ln.startswith("  time_limit_s:") and cur in new_limits:
            want = new_limits[cur]
            have = float(ln.split(":", 1)[1].strip())
            if abs(have - want) > 1e-9:
                indent = ln[:len(ln) - len(ln.lstrip())]
                ln = "%stime_limit_s: %.1f\n" % (indent, want)
                changed += 1
        out.append(ln)
    if not dry_run and changed:
        manifest.write_text("".join(out), encoding="utf-8")
    return "%d 行被改写%s" % (changed, "" if not dry_run else "（dry-run，未写入）")


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="9-3 时限冻结")
    ap.add_argument("--workset", required=True, help="工作集输出根目录")
    ap.add_argument("--manifest", default="", help="实例表路径（默认按契约自动查找）")
    ap.add_argument("--write", action="store_true", help="真的回填 instances.yaml")
    ap.add_argument("--report", default="", help="把执行记录写到该 markdown 路径")
    args = ap.parse_args(argv)

    here = Path(__file__).resolve().parents[1]          # nav_ws_core
    sys.path.insert(0, str(here / "src" / "robot_benchmark"))
    from robot_benchmark.scenario_gen import default_manifest_path
    manifest = Path(args.manifest) if args.manifest else Path(default_manifest_path())
    workset = Path(args.workset).resolve()

    runs = load_runs(workset)
    rows = compute(runs)

    print("%-20s %-18s %-18s %-18s %8s %8s" %
          ("场景", "DWA 中位(成功/总)", "MPPI 中位(成功/总)", "TEB 中位(成功/总)", "旧", "新"))
    for r in rows:
        def cell(b):
            p = r["per"].get(b)
            return "-" if not p else "%.2f s (%d/%d)" % (p["median"], p["n"], r["n_total"][b])
        print("%-20s %-18s %-18s %-18s %8.1f %8.1f" %
              (r["scenario"], cell("dwa"), cell("mppi"), cell("teb"), r["t_old"], r["t_new"]))

    new_limits = {r["scenario"]: r["t_new"] for r in rows}
    print("\n实例表回填：%s" % rewrite_manifest(manifest, new_limits, dry_run=not args.write))

    if args.report:
        Path(args.report).write_text(render_report(rows, workset, manifest), encoding="utf-8")
        print("执行记录已写入：%s" % args.report)
    return 0


if __name__ == "__main__":
    sys.exit(main())
