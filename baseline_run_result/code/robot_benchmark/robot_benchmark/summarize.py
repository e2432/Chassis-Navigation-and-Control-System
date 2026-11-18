#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""基线成绩汇总（9-2 交付物之一）：把 runs/ 汇总成"可入库"的表格 + 异常清单。

聚合口径（`docs/指标定义.md` v1.0）：
  * 单场景：成功率 = 成功数 / run 数；到达时间/路径长度/平滑度/最小间距取**中位数**（到达时间仅成功 run）；
    急停次数取 run 内合计；
  * 套件级 `dynamic_stress` = (dynamic_pedestrian + mixed) 成功数 / 两者总 run 数；
  * 最小间距同时给"20 次中位数"和"最坏值"；
  * 失败 run 一律进异常清单（failure_class），不得静默丢弃。

用法：
    python3 -m robot_benchmark.summarize --runs-root out/9-2 --out out/9-2/summary.md
    python3 -m robot_benchmark.summarize --runs-root out/9-2 --json out/9-2/summary.json
"""

from __future__ import annotations

import argparse
import json
import os
import statistics as st
import sys
from typing import Dict, List, Optional

SCENARIOS = ("static_avoid", "dynamic_pedestrian", "narrow_passage", "dead_end", "mixed")


def _median(vals: List[float]) -> Optional[float]:
    v = [x for x in vals if x is not None]
    return round(st.median(v), 4) if v else None


def load_runs(runs_root: str) -> Dict[str, List[Dict]]:
    """按场景收集 run.json（目录结构 <root>/runs/<scenario>/<seed>/run.json）。"""
    out: Dict[str, List[Dict]] = {s: [] for s in SCENARIOS}
    # 布局 A：<root>/runs/<scenario>/<seed>/（单一控制器）
    # 布局 B：<root>/<controller>/runs/<scenario>/<seed>/（多控制器并存，推荐）
    roots = []
    if os.path.isdir(os.path.join(runs_root, "runs")):
        roots.append(os.path.join(runs_root, "runs"))
    for name in sorted(os.listdir(runs_root)):
        cand = os.path.join(runs_root, name, "runs")
        if os.path.isdir(cand):
            roots.append(cand)
    if not roots:
        raise SystemExit("找不到 runs/ 目录（先跑 robot_benchmark.runner）：%s" % runs_root)
    for base in roots:
        for scen in sorted(os.listdir(base)):
            dpath = os.path.join(base, scen)
            if not os.path.isdir(dpath):
                continue
            for seed in sorted(os.listdir(dpath)):
                f = os.path.join(dpath, seed, "run.json")
                if os.path.isfile(f):
                    with open(f, encoding="utf-8") as fh:
                        out.setdefault(scen, []).append(json.load(fh))
    return out


def summarize(runs: Dict[str, List[Dict]]) -> Dict:
    per = {}
    anomalies = []
    for scen, items in runs.items():
        if not items:
            continue
        n = len(items)
        succ = [r for r in items if r["metrics"]["success"]]
        per[scen] = {
            "runs": n,
            "success_rate": round(len(succ) / n, 4),
            "time_to_goal_s_median": _median([r["metrics"]["time_to_goal_s"] for r in succ]),
            "path_length_m_median": _median([r["metrics"]["path_length_m"] for r in succ]),
            "smoothness_median": _median([r["metrics"]["smoothness"] for r in succ]),
            "min_clearance_m_median": _median([r["metrics"]["min_clearance_m"] for r in items]),
            "min_clearance_m_worst": round(min(r["metrics"]["min_clearance_m"] for r in items), 4),
            "estop_count_total": int(sum(r["metrics"]["estop_count"] for r in items)),
            "controllers": sorted({r["controller"] for r in items}),
            "backends": sorted({r.get("backend", "?") for r in items}),
        }
        for r in items:
            if not r["metrics"]["success"]:
                anomalies.append({
                    "scenario": scen,
                    "seed": r["seed"],
                    "run": r.get("run"),
                    "controller": r["controller"],
                    "failure_class": r["failure_class"],
                    "min_clearance_m": r["metrics"]["min_clearance_m"],
                    "t_end_s": r.get("judge_detail", {}).get("t_end_s"),
                })
    dyn = [s for s in ("dynamic_pedestrian", "mixed") if runs.get(s)]
    if dyn:
        tot = sum(len(runs[s]) for s in dyn)
        ok = sum(1 for s in dyn for r in runs[s] if r["metrics"]["success"])
        dynamic_stress = round(ok / tot, 4) if tot else None
    else:
        dynamic_stress = None
    return {"per_scenario": per, "dynamic_stress": dynamic_stress, "anomalies": anomalies}


def render_markdown(summary: Dict, title: str) -> str:
    lines = ["# %s" % title, ""]
    lines.append("| 场景 | run 数 | 成功率 | 到达中位 [s] | 路径中位 [m] | 平滑度中位 | 间距中位 [m] | 间距最坏 [m] | 急停合计 |")
    lines.append("|---|---|---|---|---|---|---|---|---|")
    for scen, v in summary["per_scenario"].items():
        lines.append("| %s | %d | %.1f%% | %s | %s | %s | %s | %s | %d |" % (
            scen, v["runs"], 100.0 * v["success_rate"],
            "-" if v["time_to_goal_s_median"] is None else v["time_to_goal_s_median"],
            "-" if v["path_length_m_median"] is None else v["path_length_m_median"],
            "-" if v["smoothness_median"] is None else v["smoothness_median"],
            v["min_clearance_m_median"], v["min_clearance_m_worst"], v["estop_count_total"]))
    lines.append("")
    ds = summary["dynamic_stress"]
    lines.append("套件级 `dynamic_stress`（dynamic_pedestrian + mixed 合并成功率）：**%s**"
                 % ("-" if ds is None else "%.1f%%" % (100.0 * ds)))
    lines.append("")
    lines.append("## 异常清单（失败 run，必须逐条说明）")
    if not summary["anomalies"]:
        lines.append("- 无")
    else:
        lines.append("| 场景 | seed | run | 控制器 | failure_class | 最小间距 [m] | 结束时刻 [s] |")
        lines.append("|---|---|---|---|---|---|---|")
        for a in summary["anomalies"]:
            lines.append("| %s | %s | %s | %s | %s | %s | %s |" % (
                a["scenario"], a["seed"], a["run"], a["controller"], a["failure_class"],
                a["min_clearance_m"], a["t_end_s"]))
    return "\n".join(lines) + "\n"


def main(argv=None) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--runs-root", default="out/9-2")
    ap.add_argument("--out", default=None, help="markdown 输出路径")
    ap.add_argument("--json", default=None)
    ap.add_argument("--title", default="基线成绩汇总（工程侧采集）")
    a = ap.parse_args(argv)
    runs = load_runs(a.runs_root)
    summary = summarize(runs)
    backends = sorted({b for v in summary["per_scenario"].values() for b in v["backends"]})
    md = render_markdown(summary, a.title)
    if backends and all("dryrun" in b for b in backends):
        md = ("> ⚠️ 本表来自 **replay dry-run**（合成轨迹），只用于验证评分口径与落盘格式，"
              "**不是官方成绩**，不得入库或用于 M4 判读。\n\n") + md
    if a.out:
        os.makedirs(os.path.dirname(os.path.abspath(a.out)), exist_ok=True)
        with open(a.out, "w", encoding="utf-8") as f:
            f.write(md)
        print("汇总表 → %s" % a.out)
    else:
        print(md)
    if a.json:
        with open(a.json, "w", encoding="utf-8") as f:
            json.dump(summary, f, ensure_ascii=False, indent=2)
        print("汇总 JSON → %s" % a.json)
    return 0


if __name__ == "__main__":
    sys.exit(main())
