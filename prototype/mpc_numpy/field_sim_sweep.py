#!/usr/bin/env python3
"""Parameter sensitivity sweep for the static-field MPC (conda pro).

Grid: d_safe x rho_slack x terminal_factor on bump + narrow.
Line: v_ref on bump + narrow + pedestrian.
Writes out/w6_field_sweep/sweep.csv and prints a compact table.
"""

from __future__ import annotations

import argparse
import csv
from itertools import product
from pathlib import Path

from field_sim_lib import run_case


def case_ok(result) -> bool:
    ok = result.reached and result.min_static_clearance_m >= -1e-3
    if result.kind == "pedestrian":
        ok = ok and result.min_dynamic_clearance_m >= -1e-3
    return ok


def row_for(result, **params):
    return {
        "kind": result.kind,
        "reached": int(result.reached),
        "ok": int(case_ok(result)),
        "time_s": round(result.time_s, 3),
        "min_static_clearance_m": round(result.min_static_clearance_m, 4),
        "min_dynamic_clearance_m": (
            round(result.min_dynamic_clearance_m, 4)
            if result.kind == "pedestrian" else ""),
        "min_y": round(result.min_y, 3),
        "max_y": round(result.max_y, 3),
        "max_slack": float(f"{result.max_slack:.6g}"),
        **params,
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--timeout", type=float, default=25.0)
    args = parser.parse_args()

    out_dir = Path(__file__).resolve().parent / "out" / "w6_field_sweep"
    out_dir.mkdir(parents=True, exist_ok=True)
    rows = []

    print("=== grid: d_safe x rho_slack x terminal_factor (bump + narrow) ===")
    for d_safe, rho, terminal in product(
        (0.2, 0.3, 0.4), (20.0, 80.0, 200.0), (1.0, 2.0, 4.0)
    ):
        statuses = []
        worst_static = float("inf")
        worst_slack = 0.0
        for kind in ("bump", "narrow"):
            result = run_case(
                kind, d_safe=d_safe, rho_slack=rho,
                terminal_factor=terminal, timeout_s=args.timeout)
            rows.append(row_for(
                result, sweep="grid", d_safe=d_safe, rho_slack=rho,
                terminal_factor=terminal, v_ref=0.5))
            statuses.append("Y" if case_ok(result) else "N")
            worst_static = min(worst_static, result.min_static_clearance_m)
            worst_slack = max(worst_slack, result.max_slack)
        print(
            f"d_safe={d_safe:.2f} rho={rho:5.1f} term={terminal:.0f} "
            f"bump/narrow={statuses[0]}/{statuses[1]} "
            f"worst_static={worst_static:.3f} m worst_slack={worst_slack:.3e}")

    print("\n=== v_ref sweep (defaults, bump + narrow + pedestrian) ===")
    for v_ref in (0.4, 0.5, 0.6):
        statuses = []
        times = []
        for kind in ("bump", "narrow", "pedestrian"):
            result = run_case(kind, v_ref=v_ref, timeout_s=args.timeout)
            rows.append(row_for(
                result, sweep="v_ref", d_safe=0.3, rho_slack=80.0,
                terminal_factor=2.0, v_ref=v_ref))
            statuses.append("Y" if case_ok(result) else "N")
            times.append(result.time_s)
        print(
            f"v_ref={v_ref:.2f} bump/narrow/ped={statuses[0]}/{statuses[1]}/"
            f"{statuses[2]} times=" + "/".join(f"{t:.2f}" for t in times))

    csv_path = out_dir / "sweep.csv"
    with csv_path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)

    failed = [r for r in rows if not r["ok"]]
    print(f"\nrows={len(rows)} failed={len(failed)} -> {csv_path}")
    if failed:
        for row in failed:
            print(
                f"FAIL {row['sweep']} {row['kind']} d_safe={row['d_safe']} "
                f"rho={row['rho_slack']} term={row['terminal_factor']} "
                f"v_ref={row['v_ref']} reached={row['reached']} "
                f"min_static={row['min_static_clearance_m']} "
                f"min_dynamic={row['min_dynamic_clearance_m']}")

    print("RESULT:", "PASS" if not failed else "FAIL")
    return 0 if not failed else 1


if __name__ == "__main__":
    raise SystemExit(main())
