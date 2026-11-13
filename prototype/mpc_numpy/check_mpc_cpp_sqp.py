#!/usr/bin/env python3
"""W6: compare the Python SQP solve with the C++ MpcCore solve.

Matrices are already checked by check_mpc_cpp_qp.py; this test checks the
two-iteration SQP result and the zero-speed fallback path.
"""

from __future__ import annotations

import argparse
import subprocess
import tempfile
from pathlib import Path

import numpy as np

from mpc_proto.chassis import Chassis
from mpc_proto.distance import Obstacle, Wall
from mpc_proto.mpc import MPCConfig, MPCSolver
from mpc_proto.reference import ReferencePath


def fmt(value: float) -> str:
    return f"{float(value):.17g}"


def make_case(max_iter: int):
    N = 6
    dt = 0.05
    cfg = MPCConfig(
        N=N, dt=dt, v_ref=0.6, eps_abs=1e-4, eps_rel=1e-4,
        max_iter=max_iter, n_sqp=2)
    chassis = Chassis(
        type="omni", radius=0.30, v_min=-1.0, v_max=1.0, vy_max=1.0,
        omega_max=1.5, acc_max=1.0, alpha_max=2.0)
    z0 = np.array([0.0, 0.0, 0.0])
    reference = ReferencePath(
        waypoints=np.array([[0.0, 0.0], [3.0, 1.0], [6.0, 0.0]]),
        goal_yaw=0.0, v_ref=0.6, N=N, dt=dt)
    z_ref, u_ref = reference.sequence(0.0)
    obstacles = [
        Obstacle(1, np.array([4.0, 0.20]), np.array([0.0, 0.0]), 0.25),
        Wall(5.0, -1.0, 5.2, 1.0),
    ]
    solver = MPCSolver(chassis, cfg)
    result = solver.solve(z0, obstacles, reference, t_now=0.0)
    return {
        "N": N, "dt": dt, "cfg": cfg, "chassis": chassis, "z0": z0,
        "z_ref": z_ref, "u_ref": u_ref, "obstacles": obstacles,
        "result": result,
    }


def write_input(path: Path, case) -> None:
    cfg = case["cfg"]
    ch = case["chassis"]
    lines = [
        f"{case['N']} {fmt(case['dt'])} {len(case['obstacles'])}",
        " ".join(fmt(x) for x in [
            cfg.q_lat, cfg.q_long, cfg.q_theta, cfg.r_v, cfg.r_omega,
            cfg.r_dv, cfg.r_domega, cfg.terminal_factor, cfg.rho_slack,
            cfg.d_safe, cfg.eps_abs, cfg.eps_rel, cfg.max_iter, cfg.n_sqp,
            int(cfg.slack), int(cfg.terminal)]),
        " ".join(fmt(x) for x in [
            ch.radius, ch.v_min, ch.v_max, ch.vy_max, ch.omega_max,
            ch.acc_max, ch.alpha_max]),
        " ".join(fmt(x) for x in case["z0"]),
    ]
    for name in ("z_ref", "u_ref"):
        for row in case[name]:
            lines.append(" ".join(fmt(x) for x in row))
    for obstacle in case["obstacles"]:
        if isinstance(obstacle, Wall):
            lines.append("wall " + " ".join(fmt(x) for x in [
                obstacle.x0, obstacle.y0, obstacle.x1, obstacle.y1]))
        else:
            lines.append("circle " + " ".join(fmt(x) for x in [
                obstacle.obs_id, obstacle.p0[0], obstacle.p0[1],
                obstacle.v[0], obstacle.v[1], obstacle.radius]))
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def read_output(path: Path):
    lines = path.read_text(encoding="utf-8").splitlines()
    status = lines[0].strip()
    meta = lines[1].split()
    success = bool(int(meta[0]))
    fallback = bool(int(meta[1]))
    streak = int(meta[2])
    alarm = bool(int(meta[3]))
    qp_iters = int(meta[4])
    max_slack = float(meta[5])
    u = np.array([float(v) for v in lines[2].split()], dtype=float)
    return status, success, fallback, streak, alarm, qp_iters, max_slack, u


def run_case(dump: Path, case, tmp_dir: Path, label: str):
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
        print(f"FAIL {label}: mpc_sqp_dump exited {proc.returncode}")
        return False
    py = case["result"]
    cpp_status, cpp_success, cpp_fallback, cpp_streak, cpp_alarm, cpp_iters, \
        cpp_slack, cpp_u = read_output(output_path)
    print(f"--- {label} ---")
    print(f"python: success={py.success} fallback={py.fallback} "
          f"status={py.status} u={py.u}")
    print(f"C++:    success={cpp_success} fallback={cpp_fallback} "
          f"status={cpp_status} streak={cpp_streak} alarm={cpp_alarm} "
          f"qp_iters={cpp_iters} max_slack={cpp_slack:.3e} u={cpp_u}")
    ok = (cpp_success == bool(py.success)) and (cpp_fallback == bool(py.fallback))
    if py.success:
        diff = float(np.max(np.abs(cpp_u - py.u)))
        ok = ok and diff <= 1e-3
        print(f"control diff: {diff:.3e} (tol=1e-3)")
    else:
        ok = ok and np.allclose(cpp_u, np.zeros(3), atol=1e-9)
    print(f"{'PASS' if ok else 'FAIL'} {label}")
    return ok


def main() -> int:
    parser = argparse.ArgumentParser()
    root = Path(__file__).resolve().parents[2]
    parser.add_argument(
        "--dump", default=str(root / "nav_ws_core/build/mpc_controller/mpc_sqp_dump"),
        help="path to the built mpc_sqp_dump executable")
    args = parser.parse_args()
    dump = Path(args.dump)
    if not dump.exists():
        print(f"FAIL: mpc_sqp_dump not found: {dump}")
        print("build with: cd nav_ws_core && colcon build --packages-select mpc_controller")
        return 2

    ok = True
    with tempfile.TemporaryDirectory(prefix="mpc_sqp_check_") as tmp:
        tmp_dir = Path(tmp)
        ok &= run_case(dump, make_case(max_iter=10000), tmp_dir, "normal")
        ok &= run_case(dump, make_case(max_iter=1), tmp_dir, "fallback")
    print("RESULT:", "PASS" if ok else "FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
