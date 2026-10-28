#!/usr/bin/env python3
"""W2 numpy 开环模拟入口：S1/S2/S3 滚动推演 + 验收四条。

用法（conda pro 环境）：
  python run_sim.py --all                 # 三场景 + 验收表 + 诊断图
  python run_sim.py --scenario S2 --N 20
  python run_sim.py --all --chassis chassis/diff.yaml   # 迁移冒烟预演（vy_max=0）
退出码 0 ⟺ 全部场景过验收②③④（①一条命令由 run_w2.sh 保证）。
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT))

from mpc_proto.chassis import load_chassis
from mpc_proto.mpc import MPCConfig
from mpc_proto.plotting import plot_run
from mpc_proto.scenarios import ALL, by_name
from mpc_proto.sim import RunResult, run


def fmt_row(r: RunResult) -> dict:
    acc = r.acceptance()
    return {
        "场景": r.scenario,
        "到达": "是" if r.arrived else "否",
        "t [s]": f"{r.t_arrival:.2f}" if r.t_arrival is not None else "—",
        "解算 ok/fail": f"{r.solve_ok}/{r.solve_fail}",
        "min间隙 [m]": f"{r.min_clearance:.3f}",
        "TV/拍": f"{r.tv_per_tick:.3f}",
        "max slack": f"{r.max_slack:.3f}",
        "②逐步解算": "PASS" if acc["solve_every_tick"] else "FAIL",
        "③间隙>0": "PASS" if acc["clearance_positive"] else "FAIL",
        "④Δu有界": "PASS" if acc["tv_bounded"] else "FAIL",
    }


def print_table(rows: list[dict]) -> None:
    keys = list(rows[0].keys())
    widths = {k: max(len(k), *(len(str(r[k])) for r in rows)) for k in keys}
    line = " | ".join(k.ljust(widths[k]) for k in keys)
    print("\n" + line)
    print("-" * len(line))
    for r in rows:
        print(" | ".join(str(r[k]).ljust(widths[k]) for k in keys))


def main() -> int:
    ap = argparse.ArgumentParser(description="W2 numpy MPC 开环模拟")
    ap.add_argument("--all", action="store_true", help="跑 S1/S2/S3 全部场景")
    ap.add_argument("--scenario", choices=["S1", "S2", "S3"], help="单场景")
    ap.add_argument("--N", type=int, default=40, help="预测时域步数（初值 40）")
    ap.add_argument("--chassis", default=str(ROOT / "chassis/omni.yaml"))
    ap.add_argument("--out", default=str(ROOT / "out"))
    ap.add_argument("--no-plot", action="store_true")
    ap.add_argument("--quiet", action="store_true", help="不打印逐步日志")
    args = ap.parse_args()

    if not args.all and not args.scenario:
        ap.error("需 --all 或 --scenario")

    chassis = load_chassis(args.chassis)
    cfg = MPCConfig(N=args.N)
    out_dir = Path(args.out)
    print(
        f"MPC 开环模拟 | N={cfg.N} dt={cfg.dt}s 视界={cfg.N * cfg.dt:.1f}s | "
        f"chassis={chassis.type} r={chassis.radius} vy_max={chassis.vy_max} | "
        f"q=({cfg.q_lat},{cfg.q_long},{cfg.q_theta}) ρ={cfg.rho_slack} d_safe={cfg.d_safe}"
    )

    scenes = list(ALL) if args.all else [lambda: by_name(args.scenario)]
    rows = []
    all_pass = True
    for fn in scenes:
        sc = fn()
        print(f"\n=== {sc.name} {sc.desc} ===")
        res = run(sc, chassis, cfg, log_every=0 if args.quiet else 100)
        rows.append(fmt_row(res))
        all_pass &= res.acceptance_pass() and res.arrived
        if not args.no_plot:
            p = plot_run(sc, chassis, res, out_dir / f"{sc.name}.png")
            print(f"  诊断图 → {p}")

    print_table(rows)
    print(
        "\n验收②逐步解算成功 ③全程间隙>0 ④Δu总变差有界：",
        "ALL PASS" if all_pass else "FAIL",
        "（①一条命令见 run_w2.sh；到达判定 0.25m/0.25rad）",
    )
    return 0 if all_pass else 1


if __name__ == "__main__":
    raise SystemExit(main())
