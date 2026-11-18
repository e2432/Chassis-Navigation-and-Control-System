#!/usr/bin/env python3
"""Per-cycle timing of the Python MPC with static field + dynamic obstacle.

This measures the Python prototype (conda pro), not the C++ plugin. C++ real
time is covered by the ROS smoke runs. Output: out/w6_bench/py_mpc_bench.json.
"""

from __future__ import annotations

import argparse
import json
import time
from pathlib import Path

import numpy as np

from field_sim_lib import build_map
from mpc_proto.chassis import Chassis
from mpc_proto.distance import Obstacle
from mpc_proto.mpc import MPCConfig, MPCSolver
from mpc_proto.reference import ReferencePath


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--cycles", type=int, default=200)
    parser.add_argument("--N", type=int, default=40)
    args = parser.parse_args()

    field = build_map("bump")
    chassis = Chassis(
        type="omni", radius=0.30, v_min=-1.0, v_max=1.0, vy_max=1.0,
        omega_max=1.5, acc_max=1.0, alpha_max=2.0)
    cfg = MPCConfig(
        N=args.N, dt=0.05, v_ref=0.5, d_safe=0.30, n_sqp=2,
        eps_abs=1e-3, eps_rel=1e-3, max_iter=10000, slack=True)
    reference = ReferencePath(
        waypoints=np.array([[0.0, 0.0], [5.5, 0.0]]),
        goal_yaw=0.0, v_ref=cfg.v_ref, N=cfg.N, dt=cfg.dt)
    solver = MPCSolver(chassis, cfg)

    z = np.array([0.0, 0.0, 0.0])
    samples = []
    statuses = {}
    fallbacks = 0
    iterations = []
    max_slack = 0.0

    for cycle in range(args.cycles):
        t = cycle * cfg.dt
        ped_y = 0.8 * (t - 6.0)
        obstacles = [
            Obstacle(1, np.array([2.8, ped_y]), np.array([0.0, 0.8]), 0.3)]

        start = time.perf_counter()
        result = solver.solve(
            z, obstacles, reference, t_now=t, static_field=field)
        elapsed_ms = (time.perf_counter() - start) * 1000.0
        samples.append(elapsed_ms)
        statuses[result.status] = statuses.get(result.status, 0) + 1

        if result.success:
            u = result.u
            max_slack = max(max_slack, result.max_slack)
            iterations.append(int(result.qp_iters))
            fallbacks += int(result.fallback)
        else:
            u = np.zeros(3)

        x, y, theta = z
        vx, vy, omega = u
        z = np.array([
            x + (vx * np.cos(theta) - vy * np.sin(theta)) * cfg.dt,
            y + (vx * np.sin(theta) + vy * np.cos(theta)) * cfg.dt,
            theta + omega * cfg.dt,
        ])

    samples = np.asarray(samples, dtype=float)
    cold = samples[:5]
    warm = samples[5:]
    summary = {
        "source": "python_mpc_pro",
        "N": cfg.N,
        "dx": cfg.dt,
        "cycles": args.cycles,
        "field_used": True,
        "dynamic_obstacles": 1,
        "mean_ms": float(np.mean(samples)),
        "p50_ms": float(np.percentile(samples, 50)),
        "p95_ms": float(np.percentile(samples, 95)),
        "max_ms": float(np.max(samples)),
        "cold_mean_ms": float(np.mean(cold)),
        "warm_mean_ms": float(np.mean(warm)) if warm.size else None,
        "warm_p95_ms": float(np.percentile(warm, 95)) if warm.size else None,
        "iterations_mean": float(np.mean(iterations)) if iterations else None,
        "iterations_max": int(np.max(iterations)) if iterations else None,
        "fallback_count": fallbacks,
        "status_counts": statuses,
        "max_slack": max_slack,
        "final_state": [float(v) for v in z],
    }

    out_dir = Path(__file__).resolve().parent / "out" / "w6_bench"
    out_dir.mkdir(parents=True, exist_ok=True)
    out_path = out_dir / "py_mpc_bench.json"
    out_path.write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")

    print(json.dumps(summary, indent=2))
    print(f"written: {out_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
