"""评测记录落盘（docs/接口契约.md v1.2「评测记录 schema」）。

每次 run 写 runs/<scenario>/<seed>/run.json + trajectory.npz；
Python 原型与 C++ 插件同口径。模拟场景（S1~S3）seed/run 置 null、目录 dev/。
"""
from __future__ import annotations

import json
from pathlib import Path

import numpy as np

from .mpc import MPCConfig
from .sim import RunResult

SCHEMA_VERSION = 1


def _failure_class(res: RunResult) -> str | None:
    """按运行判定自动推导；dead_end_reverse 等语义类留分析阶段标注。"""
    if res.arrived and res.min_clearance > 0.0:
        return None
    if res.min_clearance <= 0.0:
        return "collision"
    if res.solve_fail > 0:
        return "solver_fail"
    return "timeout"


def write_run(
    out_root,
    scenario_key: str,
    cfg: MPCConfig,
    res: RunResult,
    seed: int | None = None,
    run: int | None = None,
    params: dict | None = None,
) -> Path:
    """写一次 run 的评测记录，返回落盘目录。"""
    seed_dir = str(seed) if seed is not None else "dev"
    run_dir = Path(out_root) / "runs" / scenario_key / seed_dir
    run_dir.mkdir(parents=True, exist_ok=True)

    qp = np.asarray(res.qp_ms, dtype=float)
    record = {
        "schema_version": SCHEMA_VERSION,
        "controller": "mpc_proto",
        "scenario": scenario_key,
        "seed": seed,
        "run": run,
        "ablation": {
            "vo": cfg.vo,
            "esdf": cfg.esdf,
            "slack": cfg.slack,
            "terminal": cfg.terminal,
            "N": cfg.N,
            "q_ratio": round(cfg.q_long / cfg.q_lat, 4),
        },
        "metrics": {
            # success = 时限内到达 ∧ 无物理碰撞（真值几何裁判；时限由推演步数截断）
            "success": bool(res.arrived and res.min_clearance > 0.0),
            "time_to_goal_s": round(res.t_arrival, 3) if res.t_arrival is not None else None,
            "path_length_m": round(res.path_length, 3),
            "smoothness": round(res.tv, 3),
            "min_clearance_m": round(res.min_clearance, 3) if np.isfinite(res.min_clearance) else None,
            "estop_count": 0,  # 原型无安全急停层，恒 0
        },
        "qp_timing": {
            "solves": int(qp.size),
            "mean_ms": round(float(qp.mean()), 3) if qp.size else 0.0,
            "p95_ms": round(float(np.percentile(qp, 95)), 3) if qp.size else 0.0,
            "max_ms": round(float(qp.max()), 3) if qp.size else 0.0,
        },
        "failure_class": _failure_class(res),
        "params": params or {},
    }
    with (run_dir / "run.json").open("w", encoding="utf-8") as f:
        json.dump(record, f, ensure_ascii=False, indent=2)
    np.savez(
        run_dir / "trajectory.npz",
        t=np.asarray(res.t_ctrl, dtype=float),
        state=np.asarray(res.zs, dtype=float),
        control=np.asarray(res.us, dtype=float),
        qp_ms=qp,
    )
    return run_dir
