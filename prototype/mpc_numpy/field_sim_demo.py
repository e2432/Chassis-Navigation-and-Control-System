#!/usr/bin/env python3
"""Quick bump/corridor closed-loop demo for the static-field MPC (conda pro).

For the expanded scenario set run field_sim_scenarios.py.
"""

from __future__ import annotations

import argparse

from field_sim_lib import run_case


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--timeout", type=float, default=20.0)
    args = parser.parse_args()

    ok = True
    for kind in ("bump", "corridor"):
        result = run_case(kind, timeout_s=args.timeout)
        print(
            f"[{kind}] reached={result.reached} t={result.time_s:.2f}s "
            f"final=({result.final[0]:.2f},{result.final[1]:.2f}) "
            f"min_clearance={result.min_static_clearance_m:.3f} m "
            f"y_range=[{result.min_y:.2f},{result.max_y:.2f}] "
            f"max_slack={result.max_slack:.3e}")
        case_ok = result.reached and result.min_static_clearance_m >= -1e-3
        if kind == "bump":
            case_ok = case_ok and result.min_y < -0.05
        print(f"{'PASS' if case_ok else 'FAIL'} {kind}")
        ok &= case_ok

    print("RESULT:", "PASS" if ok else "FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
