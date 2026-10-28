#!/usr/bin/env python3
"""预测时域敏感性扫描 N ∈ {20, 40, 60}（docs/MPC公式化.md §10 定案依据）。

用法：python sweep_horizon.py
输出：三场景 × 三 N 的指标表 + 定 N 建议（成功/间隙/TV/到达时间齐看）。
"""
from __future__ import annotations

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT))

from mpc_proto.chassis import load_chassis
from mpc_proto.mpc import MPCConfig
from mpc_proto.scenarios import ALL
from mpc_proto.sim import run

N_LIST = [20, 40, 60]


def main() -> int:
    chassis = load_chassis(ROOT / "chassis/omni.yaml")
    print(f"视界敏感性扫描 | chassis={chassis.type} | v_ref=0.6 dt=0.05")
    header = f"{'N':>3} | {'场景':<26} | {'到达':<4} | {'t [s]':>6} | {'min间隙':>8} | {'TV/拍':>7} | {'slack':>6} | {'解算fail':>8}"
    print(header)
    print("-" * len(header))
    rows = []
    for N in N_LIST:
        cfg = MPCConfig(N=N)
        for fn in ALL:
            sc = fn()
            res = run(sc, chassis, cfg, dt_sim=0.01, log_every=0)
            rows.append((N, sc.name, res))
            print(
                f"{N:>3} | {sc.name + ' ' + sc.desc:<26} | "
                f"{'是' if res.arrived else '否':<4} | "
                f"{(res.t_arrival if res.t_arrival is not None else float('nan')):6.2f} | "
                f"{res.min_clearance:8.3f} | {res.tv_per_tick:7.3f} | "
                f"{res.max_slack:6.3f} | {res.solve_fail:8d}"
            )

    print("\n各 N 汇总（到达数 / 最小间隙下限 / TV/拍 上限）：")
    for N in N_LIST:
        sub = [r for (n, _, r) in rows if n == N]
        n_arr = sum(r.arrived for r in sub)
        print(
            f"  N={N:<3} 视界={N * 0.05:.1f}s  到达 {n_arr}/3  "
            f"min间隙 {min(r.min_clearance for r in sub):.3f}  "
            f"TV/拍 max {max(r.tv_per_tick for r in sub):.3f}"
        )
    print("\n定案写入 docs/MPC公式化.md §10「视界定案」（初值 40，扫后定死）。")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
