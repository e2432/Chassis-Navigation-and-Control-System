#!/usr/bin/env python3
"""W4 运行汇总：run.json/trajectory.npz → 逐场景指标 + 套件 dynamic_stress + 四阈值自查。

口径（docs/指标定义.md、docs/接口契约.md v1.4）：
- 成功率 = 成功数 / 找到的 run 数；
- 到达时间 / 路径长度 / 平滑度 = 仅成功 run 的中位数；
- 最小间距 = 每次 run 先取 run 内最小，再取 20 次中位数（附最坏值）；
- 急停 = 合计；
- QP 计时优先从 trajectory.npz 的 qp_ms 池化，缺则回退 run.json；
- dynamic_stress = (dynamic_pedestrian + mixed 成功数) / (两场景总 runs)，不进单 run dict。

用法（conda pro）：
  python summarize_runs.py --runs-root out/runs --expected 20 --check
  python summarize_runs.py --runs-root out/tuning/round0/runs --expected 5 --check
  python summarize_runs.py --runs-root out/runs --scenarios static_avoid --expected 20
  python summarize_runs.py --runs-root out/runs --md out/w4_summary.md

退出码：无 run → 1；--check 时仅在「覆盖完整 + 四阈值全过 + 参数/消融/元信息一致」才 0。
"""
from __future__ import annotations

import argparse
import json
import sys
from collections import Counter
from datetime import date
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT))

from mpc_proto.scenarios import SCENARIO_KEYS

RUN_KEYS = {
    "schema_version", "controller", "scenario", "seed", "run", "ablation",
    "metrics", "qp_timing", "failure_class", "params",
}
METRIC_KEYS = {
    "success", "time_to_goal_s", "path_length_m", "smoothness",
    "min_clearance_m", "estop_count",
}
DYNAMIC_KEYS = ("dynamic_pedestrian", "mixed")
# params 中的 run 级元信息（instance/navfn_*）每次不同，不能拿来判参数一致性；
# 只比较调参快照本身（与 MPCConfig.params_snapshot + preset 对应）。
PARAM_KEYS = {
    "N", "dt", "v_ref", "q_lat", "q_long", "q_theta", "r_v", "r_omega",
    "r_dv", "r_domega", "terminal_factor", "rho_slack", "d_safe", "n_sqp",
    "eps_abs", "eps_rel", "max_iter", "preset",
}


def parse_scenarios(text: str) -> list[str]:
    if text.strip().lower() == "all":
        return list(SCENARIO_KEYS)
    keys = [k.strip() for k in text.split(",") if k.strip()]
    unknown = [k for k in keys if k not in SCENARIO_KEYS]
    if not keys or unknown:
        raise ValueError(f"--scenarios 应为 all 或 {SCENARIO_KEYS} 的子集；未知 {unknown}")
    return keys


def med(values) -> float | None:
    vals = [float(v) for v in values if v is not None and np.isfinite(v)]
    return float(np.median(vals)) if vals else None


def fmt(value, digits: int = 3, suffix: str = "") -> str:
    return "—" if value is None else f"{value:.{digits}f}{suffix}"


def load_scenario(runs_root: Path, key: str, expected: int) -> dict:
    idx = SCENARIO_KEYS.index(key) + 1  # 1-based：seed = 场景键序×100 + run
    expected_seeds = {idx * 100 + run for run in range(1, expected + 1)}
    records = []
    for seed_dir in sorted((runs_root / key).glob("*")):
        if not seed_dir.is_dir() or not seed_dir.name.isdigit():
            continue
        run_json = seed_dir / "run.json"
        if not run_json.is_file():
            continue
        try:
            rec = json.loads(run_json.read_text(encoding="utf-8"))
        except json.JSONDecodeError as exc:
            rec = {"_load_error": str(exc), "_path": str(run_json)}
        rec["_seed_dir"] = seed_dir
        records.append(rec)

    found_seeds = set()
    meta_ok = True
    schema_ok = True
    metrics_ok = True
    for rec in records:
        if "_load_error" in rec:
            schema_ok = meta_ok = metrics_ok = False
            continue
        seed = rec.get("seed")
        if not isinstance(seed, int):
            meta_ok = False
            continue
        found_seeds.add(seed)
        if rec.get("scenario") != key or seed != int(rec["_seed_dir"].name):
            meta_ok = False
        if rec.get("run") != seed - idx * 100:
            meta_ok = False
        if set(rec) - {"_seed_dir"} != RUN_KEYS:
            schema_ok = False
        metrics = rec.get("metrics", {})
        if not isinstance(metrics, dict) or set(metrics) != METRIC_KEYS:
            metrics_ok = False

    params_keys = {
        json.dumps(
            {k: r.get("params", {}).get(k) for k in PARAM_KEYS},
            sort_keys=True, ensure_ascii=False,
        )
        for r in records if "_load_error" not in r
    }
    ablation_keys = {
        json.dumps(r.get("ablation", {}), sort_keys=True, ensure_ascii=False)
        for r in records if "_load_error" not in r
    }
    controllers = {r.get("controller") for r in records if "_load_error" not in r}

    return {
        "key": key,
        "records": records,
        "found": len(records),
        "expected": expected,
        "missing": sorted(expected_seeds - found_seeds),
        "meta_ok": meta_ok,
        "schema_ok": schema_ok,
        "metrics_ok": metrics_ok,
        "params_consistent": len(params_keys) <= 1,
        "ablation_consistent": len(ablation_keys) <= 1,
        "controllers": controllers,
        "runs_root": runs_root,
    }


def aggregate(st: dict) -> dict:
    recs = [r for r in st["records"] if "_load_error" not in r]
    succ = [r for r in recs if r["metrics"]["success"] is True]

    clearance = [r["metrics"]["min_clearance_m"] for r in recs if r["metrics"]["min_clearance_m"] is not None]
    estop = sum(int(r["metrics"]["estop_count"]) for r in recs)
    failure_classes = Counter(r.get("failure_class") or "success" for r in recs)
    infra = sum(1 for r in recs if r.get("params", {}).get("failure_phase"))

    # QP：优先池化 npz 的 qp_ms；缺失则回退 run.json 汇总
    qp_arrays = []
    for r in recs:
        npz = r["_seed_dir"] / "trajectory.npz"
        if npz.is_file():
            with np.load(npz) as data:
                arr = np.asarray(data.get("qp_ms", np.zeros(0)), dtype=float).ravel()
            if arr.size:
                qp_arrays.append(arr)
    fallback_means = [float(r.get("qp_timing", {}).get("mean_ms", 0.0)) for r in recs]
    fallback_p95 = [float(r.get("qp_timing", {}).get("p95_ms", 0.0)) for r in recs]
    if qp_arrays:
        qp_all = np.concatenate(qp_arrays)
        qp_mean = float(qp_all.mean())
        qp_p95 = float(np.percentile(qp_all, 95))
        qp_method = "npz 池化"
    else:
        qp_mean = float(np.mean(fallback_means)) if fallback_means else None
        qp_p95 = float(np.max(fallback_p95)) if fallback_p95 else None
        qp_method = "run.json 回退"

    return {
        "found": st["found"],
        "expected": st["expected"],
        "success_rate": (len(succ) / len(recs)) if recs else None,
        "successes": len(succ),
        "time_median": med(r["metrics"]["time_to_goal_s"] for r in succ),
        "path_median": med(r["metrics"]["path_length_m"] for r in succ),
        "smooth_median": med(r["metrics"]["smoothness"] for r in succ),
        "clearance_median": med(clearance),
        "clearance_worst": min(clearance) if clearance else None,
        "clearance_missing": len(recs) - len(clearance),
        "estop_sum": estop,
        "failure_classes": failure_classes,
        "infra_failures": infra,
        "qp_mean": qp_mean,
        "qp_p95": qp_p95,
        "qp_method": qp_method,
    }


def render(stats: list[dict], runs_root: Path) -> str:
    aggs = {st["key"]: aggregate(st) for st in stats}
    complete = all(st["found"] == st["expected"] for st in stats)
    consistent = all(
        st["meta_ok"] and st["schema_ok"] and st["metrics_ok"]
        and st["params_consistent"] and st["ablation_consistent"]
        and len(st["controllers"]) <= 1
        for st in stats
    )

    lines = [
        "# W4 运行汇总",
        "",
        f"- 日期：{date.today().isoformat()}",
        f"- runs 根：`{runs_root}`",
        f"- 覆盖：{'完整' if complete else '**不完整**'}"
        + ("" if complete else "（缺失 run 见下）"),
        f"- 元信息/schema/参数/消融一致性：{'OK' if consistent else '**异常**'}",
        "",
        "| 场景 | 找到/期望 | 成功率 | 到达中位 [s] | 路径中位 [m] | 平滑中位 | 间距中位 [m] | 间距最坏 [m] | 急停合计 | QP mean [ms] | QP p95 [ms] |",
        "|---|---|---|---|---|---|---|---|---|---|---|",
    ]
    for st in stats:
        a = aggs[st["key"]]
        rate = "—" if a["success_rate"] is None else f"{a['success_rate'] * 100:.1f}%"
        lines.append(
            f"| {st['key']} | {a['found']}/{a['expected']} | {rate} | "
            f"{fmt(a['time_median'], 2)} | {fmt(a['path_median'], 3)} | "
            f"{fmt(a['smooth_median'], 3)} | {fmt(a['clearance_median'], 3)} | "
            f"{fmt(a['clearance_worst'], 3)} | {a['estop_sum']} | "
            f"{fmt(a['qp_mean'], 1)} | {fmt(a['qp_p95'], 1)} |"
        )

    # dynamic_stress（套件级）
    dyn = [aggs[k] for k in DYNAMIC_KEYS if k in aggs]
    dyn_found = sum(a["found"] for a in dyn)
    dyn_success = sum(a["successes"] for a in dyn)
    dyn_rate = (dyn_success / dyn_found) if dyn_found else None
    lines += [
        "",
        f"- **dynamic_stress**（dynamic_pedestrian + mixed 合并成功率）："
        f"{'—' if dyn_rate is None else f'{dyn_rate * 100:.1f}%'}（{dyn_success}/{dyn_found}）",
        f"- QP 计时方法：{next((a['qp_method'] for a in aggs.values()), '—')}；"
        + ("**p95 > 50 ms（20 Hz 控制拍预算预警）**" if any(
            a["qp_p95"] is not None and a["qp_p95"] > 50.0 for a in aggs.values()
        ) else "p95 ≤ 50 ms"),
        "- 注：动态场景成绩为**乐观上界**（合成障碍状态、无跟踪噪声）；estop_count 在原型中恒 0（无安全急停层）。",
        "- 注：基线补比待 DWA/TEB/MPPI 入库，本表不伪造基线数字。",
    ]

    # 缺失 / 异常 / 失败明细
    issues = []
    for st in stats:
        if st["missing"]:
            issues.append(f"- {st['key']} 缺失 seed：{st['missing']}")
        if not st["meta_ok"]:
            issues.append(f"- {st['key']} run.json 的 scenario/seed/run 与目录不一致")
        if not st["schema_ok"]:
            issues.append(f"- {st['key']} run.json 顶层字段不符合契约（多/缺字段）")
        if not st["metrics_ok"]:
            issues.append(f"- {st['key']} metrics 字段不符合契约")
        if not st["params_consistent"]:
            issues.append(f"- {st['key']} 各 run 的 params 快照不一致（拒绝据此聚合）")
        if not st["ablation_consistent"]:
            issues.append(f"- {st['key']} 各 run 的 ablation 快照不一致（正式 runs 应全开）")
        if len(st["controllers"]) > 1:
            issues.append(f"- {st['key']} controller 混用：{sorted(st['controllers'])}")
        if aggs[st["key"]]["clearance_missing"]:
            issues.append(
                f"- {st['key']} 有 {aggs[st['key']]['clearance_missing']} 条 run 的 min_clearance 为空（failure run），"
                "未计入间距中位数"
            )
        for r in st["records"]:
            if "_load_error" in r:
                issues.append(f"- {st['key']}/{r['_seed_dir'].name} run.json 解析失败：{r['_load_error']}")
                continue
            if r["metrics"]["success"] is not True:
                issues.append(
                    f"- {st['key']}/{r['_seed_dir'].name} 失败：{r.get('failure_class')}"
                    f"（min_clearance={r['metrics']['min_clearance_m']}, "
                    f"t={r['metrics']['time_to_goal_s']}）"
                    + (f"，phase={r['params'].get('failure_phase')}"
                       if r.get("params", {}).get("failure_phase") else "")
                )
    if issues:
        lines += ["", "## 缺失 / 异常 / 失败", ""] + issues

    # 四阈值自查（仅在覆盖完整时给出结论）
    if dyn_rate is not None:
        dyn_ok = dyn_rate >= 0.90
    else:
        dyn_ok = False
    checks = []
    for st in stats:
        a = aggs[st["key"]]
        checks.append((f"{st['key']} 成功率 ≥95%", a["success_rate"] is not None and a["success_rate"] >= 0.95, a["success_rate"]))
        checks.append((f"{st['key']} 间距中位 ≥0.30 m", a["clearance_median"] is not None and a["clearance_median"] >= 0.30, a["clearance_median"]))
        checks.append((f"{st['key']} 急停 ≤1", a["estop_sum"] <= 1, a["estop_sum"]))
    checks.append(("dynamic_stress ≥90%", dyn_ok, dyn_rate))

    lines += ["", "## 四阈值自查（基线缺位的必要条件预检）", ""]
    if not complete:
        lines.append("**INCOMPLETE — 覆盖不完整，四阈值门不适用。**")
    else:
        for name, ok, value in checks:
            shown = "—" if value is None else (
                f"{value * 100:.1f}%" if name.endswith("%") and ("成功率" in name or "dynamic_stress" in name)
                else f"{value:.3f}" if isinstance(value, float) else str(value)
            )
            lines.append(f"- {'PASS' if ok else 'FAIL'}  {name}：{shown}")
        all_ok = all(ok for _, ok, _ in checks) and consistent
        lines += [
            "",
            f"**结论：{'调通（待基线补比）' if all_ok else '自查未过（需调参/复核）'}**",
            "",
            "> 正式达标判定留 M3/W6 同环境 `benchmark.run()`；本自查只是 M4① 必要条件预检。",
        ]

    return "\n".join(lines) + "\n"


def main() -> int:
    ap = argparse.ArgumentParser(description="W4 run.json 汇总（逐场景指标 + dynamic_stress + 四阈值）")
    ap.add_argument("--runs-root", default=str(ROOT / "out/runs"), help="包含 <scenario>/<seed>/ 的 runs 根")
    ap.add_argument("--scenarios", default="all", help="all 或逗号分隔场景键")
    ap.add_argument("--expected", type=int, default=20, help="每场景期望 run 数（调参工作集用 5）")
    ap.add_argument("--check", action="store_true", help="输出并判定四阈值自查")
    ap.add_argument("--md", default=None, help="另存 markdown 到该路径")
    args = ap.parse_args()

    runs_root = Path(args.runs_root)
    try:
        keys = parse_scenarios(args.scenarios)
    except ValueError as exc:
        print(f"参数错误: {exc}", file=sys.stderr)
        return 2
    if not runs_root.is_dir():
        print(f"找不到 runs 根：{runs_root}", file=sys.stderr)
        return 1

    stats = [load_scenario(runs_root, key, args.expected) for key in keys]
    if not any(st["found"] for st in stats):
        print(f"没有找到任何 run.json（{runs_root}）", file=sys.stderr)
        return 1

    text = render(stats, runs_root)
    print(text)
    if args.md:
        out = Path(args.md)
        out.parent.mkdir(parents=True, exist_ok=True)
        out.write_text(text, encoding="utf-8")
        print(f"markdown → {out}")

    if not args.check:
        return 0

    aggs = {st["key"]: aggregate(st) for st in stats}
    complete = all(st["found"] == st["expected"] for st in stats)
    consistent = all(
        st["meta_ok"] and st["schema_ok"] and st["metrics_ok"]
        and st["params_consistent"] and st["ablation_consistent"]
        and len(st["controllers"]) <= 1
        for st in stats
    )
    dyn = [aggs[k] for k in DYNAMIC_KEYS if k in aggs]
    dyn_found = sum(a["found"] for a in dyn)
    dyn_rate = (sum(a["successes"] for a in dyn) / dyn_found) if dyn_found else None
    ok = complete and consistent
    for st in stats:
        a = aggs[st["key"]]
        ok &= a["success_rate"] is not None and a["success_rate"] >= 0.95
        ok &= a["clearance_median"] is not None and a["clearance_median"] >= 0.30
        ok &= a["estop_sum"] <= 1
    ok &= dyn_rate is not None and dyn_rate >= 0.90
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
