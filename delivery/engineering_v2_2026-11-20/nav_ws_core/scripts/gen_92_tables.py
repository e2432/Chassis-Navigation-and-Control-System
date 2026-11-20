#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""生成 9-2 正式成绩文档需要的全部表格片段（v2 与 v1 对照），输出 markdown 到 stdout。"""
import glob
import json
import sys

V1 = "out/9-2_official"
V2 = "out/9-2_official_v2"
SC = ("static_avoid", "dynamic_pedestrian", "narrow_passage", "dead_end", "mixed")
BS = ("dwa", "mppi", "teb")


def load(root):
    out = []
    for f in glob.glob(root + "/*/runs/*/*/run.json"):
        out.append(json.load(open(f)))
    return out


def idx(rows):
    return {(d["controller"], d["scenario"], d["seed"]): d for d in rows}


def succ(rows, sc, b):
    rs = [d for d in rows if d["scenario"] == sc and d["controller"] == b]
    ok = sum(1 for d in rs if d["metrics"]["success"])
    return ok, len(rs)


def main():
    v1, v2 = load(V1), load(V2)
    print("<!-- 采集 -->")
    print("v1=%d 条 v2=%d 条" % (len(v1), len(v2)))
    print()
    print("<!-- 总表 -->")
    print("| 场景 | DWA | MPPI | TEB |")
    print("|---|---|---|---|")
    for sc in SC:
        cells = []
        for b in BS:
            ok, n = succ(v2, sc, b)
            cells.append("%d/%d = %.0f%%" % (ok, n, 100.0 * ok / max(n, 1)))
        print("| %s | %s |" % (sc, " | ".join(cells)))
    tot = []
    for b in BS:
        ok = sum(succ(v2, sc, b)[0] for sc in SC)
        n = sum(succ(v2, sc, b)[1] for sc in SC)
        tot.append("%d/%d = %.1f%%" % (ok, n, 100.0 * ok / max(n, 1)))
    print("| **合计** | %s |" % " | ".join(tot))
    print()
    ok = sum(succ(v2, sc, b)[0] for sc in SC for b in BS)
    n = sum(succ(v2, sc, b)[1] for sc in SC for b in BS)
    dyn_ok = sum(succ(v2, sc, b)[0] for sc in ("dynamic_pedestrian", "mixed") for b in BS)
    dyn_n = sum(succ(v2, sc, b)[1] for sc in ("dynamic_pedestrian", "mixed") for b in BS)
    print("<!-- 汇总行 -->")
    print("v2 全体 %d/%d = %.1f%%；dynamic_stress %d/%d = %.1f%%" % (
        ok, n, 100.0 * ok / max(n, 1), dyn_ok, dyn_n, 100.0 * dyn_ok / max(dyn_n, 1)))
    print()
    print("<!-- v1 vs v2 -->")
    print("| 场景 | DWA | MPPI | TEB |")
    print("|---|---|---|---|")
    for sc in SC:
        cells = []
        for b in BS:
            a_ok, a_n = succ(v1, sc, b)
            c_ok, c_n = succ(v2, sc, b)
            cells.append("%.0f%% → **%.0f%%**" % (100.0 * a_ok / max(a_n, 1), 100.0 * c_ok / max(c_n, 1)))
        print("| %s | %s |" % (sc, " | ".join(cells)))
    a_ok = sum(succ(v1, sc, b)[0] for sc in SC for b in BS)
    a_n = sum(succ(v1, sc, b)[1] for sc in SC for b in BS)
    print("| **合计** | **%d/%d = %.1f%%** | **%d/%d = %.1f%%** | — |" % (
        a_ok, a_n, 100.0 * a_ok / max(a_n, 1), ok, n, 100.0 * ok / max(n, 1)))
    print()
    print("<!-- 失败构成 -->")
    print("| 场景 | 基线 | collision | timeout | other |")
    print("|---|---|---|---|---|")
    for sc in SC:
        for b in BS:
            rs = [d for d in v2 if d["scenario"] == sc and d["controller"] == b]
            c = sum(1 for d in rs if d["failure_class"] == "collision")
            t = sum(1 for d in rs if d["failure_class"] == "timeout")
            o = sum(1 for d in rs if d["failure_class"] == "other")
            print("| %s | %s | %d | %d | %d |" % (sc, b, c, t, o))
    print()
    print("<!-- 指标明细 -->")
    print("| 场景 | 基线 | 到达 [s] | 路径 [m] | 平滑度 | 间距中位 [m] | 间距最坏 [m] | 急停合计 |")
    print("|---|---|---|---|---|---|---|---|")
    for sc in SC:
        for b in BS:
            rs = [d for d in v2 if d["scenario"] == sc and d["controller"] == b]
            def med(key):
                vals = [d["metrics"][key] for d in rs if d["metrics"].get(key) is not None]
                if not vals:
                    return None
                vals.sort()
                m = len(vals) // 2
                return vals[m] if len(vals) % 2 else 0.5 * (vals[m - 1] + vals[m])
            clr = [d["metrics"]["min_clearance_m"] for d in rs if d["metrics"].get("min_clearance_m") is not None]
            estop = sum(d["metrics"].get("estop_count", 0) for d in rs)
            def f(v, nd=2):
                return "-" if v is None else ("%." + str(nd) + "f") % v
            print("| %s | %s | %s | %s | %s | %s | %s | %d |" % (
                sc, b, f(med("time_to_goal_s")), f(med("path_length_m")), f(med("smoothness")),
                f(med("min_clearance_m"), 3), f(min(clr) if clr else None, 3), estop))
    print()
    print("<!-- 判定翻转（v1→v2，同种子） -->")
    i1, i2 = idx(v1), idx(v2)
    flips = []
    for k in sorted(set(i1) & set(i2)):
        a = bool(i1[k]["metrics"]["success"])
        b = bool(i2[k]["metrics"]["success"])
        if a != b:
            flips.append((k, a, b, i1[k]["failure_class"], i2[k]["failure_class"]))
    worse = [f for f in flips if f[1] and not f[2]]
    better = [f for f in flips if f[2] and not f[1]]
    print("可比 run %d 条；改善 %d 条，变差 %d 条" % (len(set(i1) & set(i2)), len(better), len(worse)))
    for k, _a, _b, fa, fb in worse:
        print("- 变差：%s %s seed=%d 成功 → %s" % (k[0], k[1], k[2], fb))
    print()
    print("<!-- 碰撞对比 -->")
    print("v1 碰撞 %d 次（%d run），v2 碰撞 %d 次（%d run）" % (
        sum(1 for d in v1 if d["failure_class"] == "collision"), len(v1),
        sum(1 for d in v2 if d["failure_class"] == "collision"), len(v2)))
    return 0


if __name__ == "__main__":
    sys.exit(main())
