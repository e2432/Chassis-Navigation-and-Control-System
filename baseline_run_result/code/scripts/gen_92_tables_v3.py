#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""生成 9-2 正式成绩文档（v3 道口栏杆版）需要的全部表格片段，输出 markdown 到 stdout。

用法：
    python3 scripts/gen_92_tables_v3.py \
        --new out/9-2_official_v3 [--ref out/9-2_official_v2]
"""
from __future__ import annotations

import argparse
import glob
import json
from typing import Dict, List

SC = ("static_avoid", "dynamic_pedestrian", "narrow_passage", "dead_end", "mixed")
BS = ("dwa", "mppi", "teb")


def load(root: str) -> List[Dict]:
    return [json.load(open(f, encoding="utf-8")) for f in glob.glob(root + "/*/runs/*/*/run.json")]


def index(rows):
    return {(d["controller"], d["scenario"], d["seed"]): d for d in rows}


def succ(rows, sc, b):
    rs = [d for d in rows if d["scenario"] == sc and d["controller"] == b]
    ok = sum(1 for d in rs if d["metrics"]["success"])
    return ok, len(rs)


def med(vals):
    vals = sorted(v for v in vals if v is not None)
    if not vals:
        return None
    m = len(vals) // 2
    return vals[m] if len(vals) % 2 else 0.5 * (vals[m - 1] + vals[m])


def f(v, nd=2):
    return "-" if v is None else ("%." + str(nd) + "f") % v


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--new", required=True)
    ap.add_argument("--ref", default="")
    a = ap.parse_args()
    new = load(a.new)
    ref = load(a.ref) if a.ref else []

    print("<!-- 采集 -->")
    print("new=%d 条%s" % (len(new), (" ref=%d 条" % len(ref)) if ref else ""))
    print()
    print("<!-- 总表（场景 × 基线） -->")
    print("| 场景 | DWA | MPPI | TEB |")
    print("|---|---|---|---|")
    for sc in SC:
        cells = []
        for b in BS:
            ok, n = succ(new, sc, b)
            cells.append("%d/%d = %.0f%%" % (ok, n, 100.0 * ok / max(n, 1)))
        print("| %s | %s |" % (sc, " | ".join(cells)))
    tot = []
    for b in BS:
        ok = sum(succ(new, sc, b)[0] for sc in SC)
        n = sum(succ(new, sc, b)[1] for sc in SC)
        tot.append("%d/%d = %.1f%%" % (ok, n, 100.0 * ok / max(n, 1)))
    print("| **合计** | %s |" % " | ".join(tot))
    print()
    ok = sum(succ(new, sc, b)[0] for sc in SC for b in BS)
    n = sum(succ(new, sc, b)[1] for sc in SC for b in BS)
    dyn_ok = sum(succ(new, sc, b)[0] for sc in ("dynamic_pedestrian", "mixed") for b in BS)
    dyn_n = sum(succ(new, sc, b)[1] for sc in ("dynamic_pedestrian", "mixed") for b in BS)
    print("<!-- 汇总行 -->")
    print("全体 %d/%d = %.1f%%；dynamic_stress %d/%d = %.1f%%" % (
        ok, n, 100.0 * ok / max(n, 1), dyn_ok, dyn_n, 100.0 * dyn_ok / max(dyn_n, 1)))
    print()
    print("<!-- 失败构成 -->")
    print("| 场景 | 基线 | collision | timeout | other | 合计失败 |")
    print("|---|---|---|---|---|---|")
    for sc in SC:
        for b in BS:
            rs = [d for d in new if d["scenario"] == sc and d["controller"] == b]
            c = sum(1 for d in rs if d["failure_class"] == "collision")
            t = sum(1 for d in rs if d["failure_class"] == "timeout")
            o = sum(1 for d in rs if d["failure_class"] == "other")
            print("| %s | %s | %d | %d | %d | %d |" % (sc, b, c, t, o, c + t + o))
    print()
    print("<!-- 指标明细 -->")
    print("| 场景 | 基线 | 到达 [s] | 路径 [m] | 平滑度 | 间距中位 [m] | 间距最坏 [m] | 急停合计 |")
    print("|---|---|---|---|---|---|---|---|")
    for sc in SC:
        for b in BS:
            rs = [d for d in new if d["scenario"] == sc and d["controller"] == b]
            clr = [d["metrics"]["min_clearance_m"] for d in rs
                   if d["metrics"].get("min_clearance_m") is not None]
            estop = sum(d["metrics"].get("estop_count", 0) for d in rs)
            print("| %s | %s | %s | %s | %s | %s | %s | %d |" % (
                sc, b,
                f(med([d["metrics"].get("time_to_goal_s") for d in rs])),
                f(med([d["metrics"].get("path_length_m") for d in rs])),
                f(med([d["metrics"].get("smoothness") for d in rs])),
                f(med(clr), 3), f(min(clr) if clr else None, 3), estop))
    print()
    print("<!-- 逐条失败清单 -->")
    bad = [d for d in new if not d["metrics"]["success"]]
    print("失败 %d 条：" % len(bad))
    for d in sorted(bad, key=lambda x: (x["controller"], x["scenario"], x["seed"])):
        print("- %s / %s / seed=%d → %s（到达 %s，间距 %s，急停 %s）" % (
            d["controller"], d["scenario"], d["seed"], d["failure_class"],
            f(d["metrics"].get("time_to_goal_s")), f(d["metrics"].get("min_clearance_m"), 3),
            d["metrics"].get("estop_count")))
    print()
    if ref:
        print("<!-- 判定翻转（ref→new，同基线同种子） -->")
        i1, i2 = index(ref), index(new)
        keys = sorted(set(i1) & set(i2))
        better, worse = [], []
        for k in keys:
            x, y = bool(i1[k]["metrics"]["success"]), bool(i2[k]["metrics"]["success"])
            if x == y:
                continue
            (better if y else worse).append((k, i1[k]["failure_class"], i2[k]["failure_class"]))
        print("可比 run %d 条；改善 %d 条，变差 %d 条" % (len(keys), len(better), len(worse)))
        for k, fa, fb in better:
            print("- 改善：%s %s seed=%d %s → 成功" % (k[0], k[1], k[2], fa))
        for k, fa, fb in worse:
            print("- 变差：%s %s seed=%d 成功 → %s" % (k[0], k[1], k[2], fb))
        print()
        print("<!-- 场景对比 -->")
        print("| 场景 | DWA | MPPI | TEB |")
        print("|---|---|---|---|")
        for sc in SC:
            cells = []
            for b in BS:
                a_ok, a_n = succ(ref, sc, b)
                c_ok, c_n = succ(new, sc, b)
                cells.append("%.0f%% → **%.0f%%**" % (100.0 * a_ok / max(a_n, 1),
                                                     100.0 * c_ok / max(c_n, 1)))
            print("| %s | %s |" % (sc, " | ".join(cells)))
        a_ok = sum(succ(ref, sc, b)[0] for sc in SC for b in BS)
        a_n = sum(succ(ref, sc, b)[1] for sc in SC for b in BS)
        print("| **合计** | **%d/%d = %.1f%%** → **%d/%d = %.1f%%** | | |" % (
            a_ok, a_n, 100.0 * a_ok / max(a_n, 1), ok, n, 100.0 * ok / max(n, 1)))
        print()
        print("<!-- 行人间距对比（dynamic_pedestrian + mixed，300 run 里取最坏） -->")
        for tag, rows in (("ref", ref), ("new", new)):
            vals = [d["metrics"]["min_clearance_m"] for d in rows
                    if d["scenario"] in ("dynamic_pedestrian", "mixed")
                    and d["metrics"].get("min_clearance_m") is not None]
            print("%s：最坏 %s，中位 %s（n=%d）" % (
                tag, f(min(vals), 3), f(med(vals), 3), len(vals)))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
