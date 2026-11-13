#!/usr/bin/env python3
"""W6: compare the C++ QP builder with the W4 Python MPC QP on identical inputs.

The script captures the matrices passed to OSQP by the Python prototype, asks
the C++ `mpc_qp_dump` executable to build the same QP, and compares P/q/A/l/u
and the OSQP solution. Run in the conda `pro` environment.
"""

from __future__ import annotations

import argparse
import math
import subprocess
import tempfile
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import osqp
from scipy import sparse

from mpc_proto.chassis import Chassis
from mpc_proto.distance import Obstacle, Wall
from mpc_proto.mpc import MPCConfig, MPCSolver
import mpc_proto.mpc as mpc_module


class CaptureOSQP:
    """Capture setup() arguments instead of solving."""

    captured = None

    def setup(self, P, q, A, l, u, **kwargs):  # noqa: N803 - OSQP API names
        CaptureOSQP.captured = (P, q, A, l, u)
        self.nx = P.shape[0]

    def solve(self):
        return SimpleNamespace(
            x=np.zeros(self.nx),
            info=SimpleNamespace(status="solved", obj_val=0.0, iter=0),
        )


def fmt(value: float) -> str:
    return f"{float(value):.17g}"


def make_python_qp():
    N = 6
    dt = 0.05
    cfg = MPCConfig(N=N, dt=dt, v_ref=0.6, eps_abs=1e-6, eps_rel=1e-6,
                    max_iter=20000, n_sqp=1)
    chassis = Chassis(
        type="omni", radius=0.30, v_min=-1.0, v_max=1.0, vy_max=1.0,
        omega_max=1.5, acc_max=1.0, alpha_max=2.0)

    z0 = np.array([0.0, 0.0, 0.10])
    z_ref = np.zeros((N + 1, 3))
    for k in range(N + 1):
        z_ref[k] = [0.35 * k * dt, 0.03 * math.sin(0.4 * k), 0.04 * math.sin(0.3 * k)]
    zbar = z_ref.copy()
    zbar[:, 0] += 0.02
    zbar[:, 1] -= 0.01
    zbar[:, 2] += 0.005
    ubar = np.tile([0.5, 0.02, 0.03], (N, 1))
    ubar[:, 0] += 0.02 * np.sin(np.arange(N))
    ubar[:, 1] += 0.01 * np.cos(np.arange(N))
    u_prev = np.array([0.10, -0.02, 0.01])

    obstacles = [
        Obstacle(1, np.array([2.0, 0.30]), np.array([0.10, 0.0]), 0.25),
        Wall(2.5, -1.0, 2.7, 1.0),
    ]

    solver = MPCSolver(chassis, cfg)
    solver._u_prev = u_prev.copy()
    real_osqp = mpc_module.osqp.OSQP
    mpc_module.osqp.OSQP = CaptureOSQP
    try:
        solver._solve_qp(z0, zbar, ubar, z_ref, obstacles)
    finally:
        mpc_module.osqp.OSQP = real_osqp
    captured = CaptureOSQP.captured
    if captured is None:
        raise RuntimeError("Python QP capture failed")
    P, q, A, l, u = captured
    P_dense = P.toarray()
    A_dense = A.toarray()

    # OSQP C API expects the upper triangle of P. The Python prototype passes
    # the full symmetric matrix; solve both canonicalizations for diagnostics.
    solutions = {}
    for label, P_arg in (("full", P), ("upper", sparse.triu(P))):
        result = osqp.OSQP()
        result.setup(
            P=P_arg, q=q, A=A, l=l, u=u,
            eps_abs=cfg.eps_abs, eps_rel=cfg.eps_rel, max_iter=cfg.max_iter,
            polish=True, verbose=False)
        solved = result.solve()
        solutions[label] = (solved.x, solved.info.status, solved.info.iter)
    return {
        "N": N, "dt": dt, "cfg": cfg, "chassis": chassis,
        "z0": z0, "z_ref": z_ref, "zbar": zbar, "ubar": ubar,
        "u_prev": u_prev, "obstacles": obstacles,
        "P": P_dense, "q": np.asarray(q), "A": A_dense,
        "l": np.asarray(l), "u": np.asarray(u),
        "solutions": solutions,
    }


def write_input(path: Path, data) -> None:
    cfg = data["cfg"]
    ch = data["chassis"]
    lines = [
        f"{data['N']} {fmt(data['dt'])} {len(data['obstacles'])}",
        " ".join(fmt(x) for x in [
            cfg.q_lat, cfg.q_long, cfg.q_theta, cfg.r_v, cfg.r_omega,
            cfg.r_dv, cfg.r_domega, cfg.terminal_factor, cfg.rho_slack,
            cfg.d_safe, cfg.eps_abs, cfg.eps_rel, cfg.max_iter, cfg.n_sqp,
            int(cfg.slack), int(cfg.terminal)]),
        " ".join(fmt(x) for x in [
            ch.radius, ch.v_min, ch.v_max, ch.vy_max, ch.omega_max,
            ch.acc_max, ch.alpha_max]),
        " ".join(fmt(x) for x in data["z0"]),
    ]
    for name in ("z_ref", "zbar", "ubar"):
        for row in data[name]:
            lines.append(" ".join(fmt(x) for x in row))
    lines.append(" ".join(fmt(x) for x in data["u_prev"]))
    for obstacle in data["obstacles"]:
        if isinstance(obstacle, Wall):
            lines.append("wall " + " ".join(fmt(x) for x in [
                obstacle.x0, obstacle.y0, obstacle.x1, obstacle.y1]))
        else:
            lines.append("circle " + " ".join(fmt(x) for x in [
                obstacle.obs_id, obstacle.p0[0], obstacle.p0[1],
                obstacle.v[0], obstacle.v[1], obstacle.radius]))
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def read_qp(path: Path):
    tokens = path.read_text(encoding="utf-8").split()
    n_u, n_s, n_obs, nx, m = (int(tokens[i]) for i in range(5))
    index = 5

    def take(count):
        nonlocal index
        values = np.array([float(v) for v in tokens[index:index + count]], dtype=float)
        index += count
        return values

    P = take(nx * nx).reshape(nx, nx)
    q = take(nx)
    A = take(m * nx).reshape(m, nx)
    l = take(m)
    u = take(m)
    if index != len(tokens):
        raise RuntimeError(f"QP output has {len(tokens) - index} trailing tokens")
    return {"n_u": n_u, "n_s": n_s, "n_obs": n_obs, "P": P, "q": q,
            "A": A, "l": l, "u": u}


def read_solution(path: Path):
    lines = path.read_text(encoding="utf-8").splitlines()
    status = lines[0].strip()
    values = " ".join(lines[1:]).split()
    return status, np.array([float(v) for v in values], dtype=float)


def compare(name: str, lhs: np.ndarray, rhs: np.ndarray, tol: float) -> bool:
    if lhs.shape != rhs.shape:
        print(f"FAIL {name}: shape {lhs.shape} vs {rhs.shape}")
        return False
    if np.isinf(lhs).any() or np.isinf(rhs).any():
        same_inf = np.array_equal(np.isinf(lhs), np.isinf(rhs)) and \
            np.array_equal(np.sign(lhs[np.isinf(lhs)]), np.sign(rhs[np.isinf(rhs)]))
        finite = np.isfinite(lhs) & np.isfinite(rhs)
        diff = np.max(np.abs(lhs[finite] - rhs[finite])) if finite.any() else 0.0
    else:
        same_inf = True
        diff = np.max(np.abs(lhs - rhs)) if lhs.size else 0.0
    ok = same_inf and diff <= tol
    print(f"{'PASS' if ok else 'FAIL'} {name}: max abs diff={diff:.3e} (tol={tol:.1e})")
    return ok


def main() -> int:
    parser = argparse.ArgumentParser()
    root = Path(__file__).resolve().parents[2]
    parser.add_argument(
        "--dump", default=str(root / "nav_ws_core/build/mpc_controller/mpc_qp_dump"),
        help="path to the built mpc_qp_dump executable")
    parser.add_argument("--keep", action="store_true", help="keep temp input/output")
    args = parser.parse_args()

    dump = Path(args.dump)
    if not dump.exists():
        print(f"FAIL: mpc_qp_dump not found: {dump}")
        print("build with: cd nav_ws_core && colcon build --packages-select mpc_controller")
        return 2

    data = make_python_qp()
    with tempfile.TemporaryDirectory(prefix="mpc_qp_check_") as tmp:
        tmp_dir = Path(tmp)
        input_path = tmp_dir / "input.txt"
        qp_path = tmp_dir / "qp.txt"
        solution_path = tmp_dir / "solution.txt"
        write_input(input_path, data)
        proc = subprocess.run(
            [str(dump), str(input_path), str(qp_path), str(solution_path)],
            text=True, capture_output=True)
        if proc.stdout:
            print(proc.stdout, end="")
        if proc.stderr:
            print(proc.stderr, end="")
        if proc.returncode != 0:
            print(f"FAIL: mpc_qp_dump exited {proc.returncode}")
            return 1

        cpp = read_qp(qp_path)
        cpp_status, cpp_x = read_solution(solution_path)
        ok = True
        for name in ("P", "q", "A", "l", "u"):
            ok &= compare(name, cpp[name], data[name], tol=1e-8)
        py_x_full, py_status_full, py_iter_full = data["solutions"]["full"]
        py_x_upper, py_status_upper, py_iter_upper = data["solutions"]["upper"]
        compare("solution_x(full P)", cpp_x, py_x_full, tol=1e-3)
        n_u = cpp["n_u"]
        ok &= compare("control_u", cpp_x[:n_u], py_x_upper[:n_u], tol=1e-3)
        if len(cpp_x) > n_u:
            ok &= compare("slack_s", cpp_x[n_u:], py_x_upper[n_u:], tol=1e-3)
        print(f"python OSQP full:  status={py_status_full} iter={py_iter_full}")
        print(f"python OSQP upper: status={py_status_upper} iter={py_iter_upper}")
        print(f"C++ OSQP:          status={cpp_status}")
        print("note: P/q/A/l/u are exact; solution tolerance 1e-3 covers the")
        print("      Python osqp 0.6.7.post3 vs C OSQP 0.6.3 solver path difference.")
        if args.keep:
            kept = Path.cwd() / "out" / "w6_qp_check"
            kept.mkdir(parents=True, exist_ok=True)
            for path in (input_path, qp_path, solution_path):
                (kept / path.name).write_text(path.read_text(encoding="utf-8"),
                                              encoding="utf-8")
            print(f"kept files in {kept}")
    print("RESULT:", "PASS" if ok else "FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
