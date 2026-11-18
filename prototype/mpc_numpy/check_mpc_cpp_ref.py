#!/usr/bin/env python3
"""W6: compare the Python reference generator with C++ ReferencePath.

The C++ generator will feed MpcCore inside the Nav2 plugin. This test pins the
time law, polyline interpolation, goal truncation and angle wrapping to the
Python W4 prototype (mpc_proto/reference.py).
"""

from __future__ import annotations

import argparse
import subprocess
import tempfile
from pathlib import Path

import numpy as np

from mpc_proto.reference import ReferencePath


def fmt(value: float) -> str:
    return f"{float(value):.17g}"


def make_cases():
    return [
        (
            "normal",
            np.array([[0.0, 0.0], [3.0, 1.0], [6.0, 0.0]]),
            0.0, 0.6, 6, 0.05, 0.0,
        ),
        (
            "truncated",
            np.array([[0.0, 0.0], [3.0, 1.0], [6.0, 0.0]]),
            0.0, 0.6, 6, 0.05, 9.0,
        ),
        (
            "zero_length_segments",
            np.array([[0.0, 0.0], [0.0, 0.0], [2.0, 0.0], [2.0, 0.0], [4.0, 1.0]]),
            0.3, 0.6, 8, 0.05, 0.3,
        ),
        (
            "turns_and_wrap",
            np.array([[0.0, 0.0], [1.0, 0.0], [0.5, -1e-6]]),
            0.0, 0.6, 10, 0.05, 1.0,
        ),
        (
            "goal_on_vertex",
            np.array([[0.0, 0.0], [2.0, 0.0], [2.0, 2.0]]),
            1.5707963267948966, 0.6, 12, 0.05, 2.0 / 0.6,
        ),
    ]


def write_input(path: Path, case) -> None:
    waypoints, goal_yaw, v_ref, N, dt, t_now = case
    lines = [str(waypoints.shape[0])]
    for point in waypoints:
        lines.append(f"{fmt(point[0])} {fmt(point[1])}")
    lines.append(" ".join(fmt(x) for x in [goal_yaw, v_ref, N, dt, t_now]))
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def read_output(path: Path):
    lines = path.read_text(encoding="utf-8").splitlines()
    n_z = int(lines[0])
    z_ref = np.array(
        [[float(v) for v in lines[1 + k].split()] for k in range(n_z)],
        dtype=float,
    )
    offset = 1 + n_z
    n_u = int(lines[offset])
    u_ref = np.array(
        [[float(v) for v in lines[offset + 1 + k].split()] for k in range(n_u)],
        dtype=float,
    )
    return z_ref, u_ref


def run_case(dump: Path, case, tmp_dir: Path, label: str) -> bool:
    waypoints, goal_yaw, v_ref, N, dt, t_now = case
    input_path = tmp_dir / f"{label}_input.txt"
    output_path = tmp_dir / f"{label}_output.txt"
    write_input(input_path, case)

    proc = subprocess.run(
        [str(dump), str(input_path), str(output_path)],
        text=True, capture_output=True)
    if proc.stdout:
        print(proc.stdout, end="")
    if proc.stderr:
        print(proc.stderr, end="")
    if proc.returncode != 0:
        print(f"FAIL {label}: mpc_ref_dump exited {proc.returncode}")
        return False

    reference = ReferencePath(
        waypoints=waypoints, goal_yaw=goal_yaw, v_ref=v_ref, N=N, dt=dt)
    py_z, py_u = reference.sequence(t_now)
    cpp_z, cpp_u = read_output(output_path)

    shape_ok = cpp_z.shape == py_z.shape and cpp_u.shape == py_u.shape
    if not shape_ok:
        print(f"FAIL {label}: shape python z{py_z.shape}/u{py_u.shape} "
              f"cpp z{cpp_z.shape}/u{cpp_u.shape}")
        return False

    z_diff = float(np.max(np.abs(cpp_z - py_z))) if py_z.size else 0.0
    u_diff = float(np.max(np.abs(cpp_u - py_u))) if py_u.size else 0.0
    ok = z_diff <= 1e-12 and u_diff <= 1e-12
    print(f"--- {label} ---")
    print(f"z_ref: max abs diff={z_diff:.3e} (tol=1e-12)")
    print(f"u_ref: max abs diff={u_diff:.3e} (tol=1e-12)")
    print(f"{'PASS' if ok else 'FAIL'} {label}")
    return ok


def main() -> int:
    parser = argparse.ArgumentParser()
    root = Path(__file__).resolve().parents[2]
    parser.add_argument(
        "--dump", default=str(root / "nav_ws_core/build/mpc_controller/mpc_ref_dump"),
        help="path to the built mpc_ref_dump executable")
    args = parser.parse_args()
    dump = Path(args.dump)
    if not dump.exists():
        print(f"FAIL: mpc_ref_dump not found: {dump}")
        print("build with: cd nav_ws_core && colcon build --packages-select mpc_controller")
        return 2

    ok = True
    with tempfile.TemporaryDirectory(prefix="mpc_ref_check_") as tmp:
        tmp_dir = Path(tmp)
        for label, *case in make_cases():
            ok &= run_case(dump, tuple(case), tmp_dir, label)
    print("RESULT:", "PASS" if ok else "FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
