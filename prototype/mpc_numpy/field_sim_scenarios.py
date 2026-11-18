#!/usr/bin/env python3
"""Synthetic closed-loop scenarios for the static-field MPC (conda pro).

Cases:
- bump: one-sided wall, must steer around;
- narrow: 1.0 m free corridor (clearance 0.20 m < d_safe 0.30 m), soft slack;
- dead_end: enter a U trap and come back out along the reference polyline;
- pedestrian: contract pedestrian crossing the path at t_meet=6 s.
"""

from __future__ import annotations

import argparse

from field_sim_lib import run_case


def evaluate(result) -> tuple[bool, str]:
    ok = result.reached and result.min_static_clearance_m >= -1e-3
    notes = []
    if result.kind == "bump":
        ok = ok and result.min_y < -0.05
        notes.append("must pass below the wall")
    elif result.kind == "pedestrian":
        ok = ok and result.min_dynamic_clearance_m >= -1e-3
        notes.append("no pedestrian penetration")
    elif result.kind == "dead_end":
        notes.append("out-and-back reference")
    elif result.kind == "narrow":
        notes.append("soft slack expected")
    return ok, "; ".join(notes)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--timeout", type=float, default=30.0)
    args = parser.parse_args()

    ok = True
    for kind in ("bump", "narrow", "dead_end", "pedestrian"):
        result = run_case(kind, timeout_s=args.timeout)
        case_ok, note = evaluate(result)
        dynamic = (
            f"min_dynamic={result.min_dynamic_clearance_m:.3f} m "
            if kind == "pedestrian" else "")
        print(
            f"[{kind}] reached={result.reached} t={result.time_s:.2f}s "
            f"final=({result.final[0]:.2f},{result.final[1]:.2f}) "
            f"min_static={result.min_static_clearance_m:.3f} m "
            f"{dynamic}y_range=[{result.min_y:.2f},{result.max_y:.2f}] "
            f"max_slack={result.max_slack:.3e}  {note}")
        print(f"{'PASS' if case_ok else 'FAIL'} {kind}")
        ok &= case_ok

    print("RESULT:", "PASS" if ok else "FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
