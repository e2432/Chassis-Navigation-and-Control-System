#!/usr/bin/env python3
"""Aggregate /mpc/diagnostics CSV into contract-style qp_timing (conda pro).

Input CSV is produced by mpc_controller/scripts/record_mpc_diagnostics.py.
Writes {"qp_timing": {...}, "diagnostics": {...}} and an optional markdown table.
"""

from __future__ import annotations

import argparse
import csv
import json
from collections import Counter
from pathlib import Path

import numpy as np


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--csv", required=True, help="diagnostics CSV path")
    parser.add_argument("--json-out", default="", help="optional JSON output path")
    parser.add_argument("--md-out", default="", help="optional markdown output path")
    args = parser.parse_args()

    csv_path = Path(args.csv)
    if not csv_path.exists():
        print(f"FAIL: CSV not found: {csv_path}")
        return 2

    solve_ms = []
    statuses = Counter()
    fallback_count = 0
    alarm_count = 0
    field_used = 0
    obstacle_counts = []
    with csv_path.open(encoding="utf-8") as stream:
        for row in csv.DictReader(stream):
            solve_ms.append(float(row["solve_ms"]))
            statuses[row["status"]] += 1
            fallback_count += int(row["fallback"])
            alarm_count += int(row["alarm"])
            field_used += int(row["static_field_used"])
            obstacle_counts.append(int(row["obstacle_count"]))

    if not solve_ms:
        print("FAIL: no diagnostic rows")
        return 2

    samples = np.asarray(solve_ms, dtype=float)
    summary = {
        "qp_timing": {
            "solves": int(samples.size),
            "mean_ms": float(np.mean(samples)),
            "p95_ms": float(np.percentile(samples, 95)),
            "max_ms": float(np.max(samples)),
        },
        "diagnostics": {
            "status_counts": dict(statuses),
            "fallback_count": fallback_count,
            "alarm_count": alarm_count,
            "static_field_used_count": field_used,
            "obstacle_count_mean": float(np.mean(obstacle_counts)),
            "obstacle_count_max": int(np.max(obstacle_counts)),
        },
    }

    text = json.dumps(summary, indent=2)
    print(text)
    if args.json_out:
        Path(args.json_out).parent.mkdir(parents=True, exist_ok=True)
        Path(args.json_out).write_text(text + "\n", encoding="utf-8")
    if args.md_out:
        timing = summary["qp_timing"]
        lines = [
            "| solves | mean [ms] | p95 [ms] | max [ms] | fallback | alarm |",
            "|---|---|---|---|---|---|",
            f"| {timing['solves']} | {timing['mean_ms']:.3f} | "
            f"{timing['p95_ms']:.3f} | {timing['max_ms']:.3f} | "
            f"{fallback_count} | {alarm_count} |",
        ]
        Path(args.md_out).parent.mkdir(parents=True, exist_ok=True)
        Path(args.md_out).write_text("\n".join(lines) + "\n", encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
