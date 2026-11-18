#!/usr/bin/env python3
"""W6: compare C++ static-field QP rows with the Python reference implementation.

Extends the W4 QP consistency test to the /map static distance field:
- Python field mirrors C++ GridDistanceField (chamfer, bilinear, gradient);
- Python MPC QP adds the same field soft constraints;
- C++ mpc_qp_dump gets the optional "field" block and must produce identical
  P/q/A/l/u (tol 1e-8), including a non-identity map->odom transform.

Run in the conda pro environment.
"""

from __future__ import annotations

import argparse
import subprocess
import tempfile
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import osqp
from scipy import sparse

from mpc_proto.chassis import Chassis
from mpc_proto.distance import Obstacle
from mpc_proto.distance_field import GridDistanceField
from mpc_proto.mpc import MPCConfig, MPCSolver
import mpc_proto.mpc as mpc_module


class CaptureOSQP:
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


def max_finite_diff(a, b) -> float:
    a = np.asarray(a, dtype=float)
    b = np.asarray(b, dtype=float)
    if a.shape != b.shape:
        return float("inf")
    a_finite = np.isfinite(a)
    b_finite = np.isfinite(b)
    if not np.array_equal(a_finite, b_finite):
        return float("inf")
    if not a_finite.any():
        return 0.0
    return float(np.max(np.abs(a[a_finite] - b[b_finite])))


def make_grid():
    width, height, resolution = 30, 30, 0.1
    occupied = np.zeros((height, width), dtype=np.uint8)
    occupied[:, 20] = 1  # wall at x = 2.05 m
    return width, height, resolution, occupied


def make_case(rotation: np.ndarray, translation: np.ndarray):
    N = 6
    dt = 0.05
    cfg = MPCConfig(
        N=N, dt=dt, v_ref=0.6, eps_abs=1e-6, eps_rel=1e-6,
        max_iter=20000, n_sqp=1)
    chassis = Chassis(
        type="omni", radius=0.30, v_min=-1.0, v_max=1.0, vy_max=1.0,
        omega_max=1.5, acc_max=1.0, alpha_max=2.0)
    z0 = np.array([0.0, 0.0, 0.05])
    z_ref = np.zeros((N + 1, 3))
    for k in range(N + 1):
        z_ref[k] = [0.3 * k * dt, 0.02 * np.sin(0.5 * k), 0.02 * np.cos(0.4 * k)]
    zbar = z_ref.copy()
    zbar[:, 0] += 0.01
    zbar[:, 1] -= 0.01
    zbar[:, 2] += 0.003
    ubar = np.tile([0.45, 0.01, 0.02], (N, 1))
    u_prev = np.array([0.08, -0.01, 0.005])
    obstacles = [Obstacle(1, np.array([2.0, 0.5]), np.array([0.0, 0.0]), 0.2)]

    width, height, resolution, occupied = make_grid()
    field = GridDistanceField.from_occupied(
        occupied, resolution, origin_x=0.0, origin_y=0.0, max_distance=5.0)

    solver = MPCSolver(chassis, cfg)
    solver._u_prev = u_prev.copy()
    real_osqp = mpc_module.osqp.OSQP
    mpc_module.osqp.OSQP = CaptureOSQP
    try:
        solver._solve_qp(
            z0, zbar, ubar, z_ref, obstacles,
            static_field=field,
            map_to_odom_rotation=rotation,
            map_to_odom_translation=translation)
    finally:
        mpc_module.osqp.OSQP = real_osqp
    captured = CaptureOSQP.captured
    if captured is None:
        raise RuntimeError("Python field QP capture failed")
    P, q, A, l, u = captured

    # Solve the same QP with the upper-triangle convention used by the C++ API.
    result = osqp.OSQP()
    result.setup(
        P=sparse.triu(P), q=q, A=A, l=l, u=u,
        eps_abs=cfg.eps_abs, eps_rel=cfg.eps_rel, max_iter=cfg.max_iter,
        polish=True, verbose=False)
    solved = result.solve()

    return {
        "N": N, "dt": dt, "cfg": cfg, "chassis": chassis,
        "z0": z0, "z_ref": z_ref, "zbar": zbar, "ubar": ubar,
        "u_prev": u_prev, "obstacles": obstacles,
        "field": field, "occupied": occupied,
        "rotation": rotation, "translation": translation,
        "P": P.toarray(), "q": np.asarray(q), "A": A.toarray(),
        "l": np.asarray(l), "u": np.asarray(u),
        "solution": solved.x, "status": solved.info.status,
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
    for name in ("z_ref", "zbar", "ubar"):
        for row in case[name]:
            lines.append(" ".join(fmt(x) for x in row))
    lines.append(" ".join(fmt(x) for x in case["u_prev"]))
    for obstacle in case["obstacles"]:
        lines.append("circle " + " ".join(fmt(x) for x in [
            obstacle.obs_id, obstacle.p0[0], obstacle.p0[1],
            obstacle.v[0], obstacle.v[1], obstacle.radius]))
    width = case["field"].width
    height = case["field"].height
    lines.append("field " + " ".join(fmt(x) for x in [
        width, height, case["field"].resolution,
        case["field"].origin_x, case["field"].origin_y,
        case["rotation"][0, 0], case["rotation"][0, 1],
        case["rotation"][1, 0], case["rotation"][1, 1],
        case["translation"][0], case["translation"][1]]))
    lines.append(" ".join(str(int(v)) for v in case["occupied"].ravel()))
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def read_qp(path: Path):
    tokens = path.read_text(encoding="utf-8").split()
    offset = 0
    n_u, n_s, n_obs, p_rows, a_rows = (int(x) for x in tokens[offset:offset + 5])
    offset += 5

    def take(count):
        nonlocal offset
        values = np.array([float(x) for x in tokens[offset:offset + count]])
        offset += count
        return values

    n = n_u + n_s
    P = take(p_rows * n).reshape(p_rows, n)
    q = take(n)
    A = take(a_rows * n).reshape(a_rows, n)
    l = take(a_rows)
    u = take(a_rows)
    return n_u, n_s, n_obs, P, q, A, l, u


def read_solution(path: Path):
    text = path.read_text(encoding="utf-8").split()
    status = text[0]
    x = np.array([float(v) for v in text[1:]])
    return status, x


def run_case(dump: Path, case, tmp_dir: Path, label: str) -> bool:
    input_path = tmp_dir / f"{label}_input.txt"
    qp_path = tmp_dir / f"{label}_qp.txt"
    solution_path = tmp_dir / f"{label}_solution.txt"
    write_input(input_path, case)
    proc = subprocess.run(
        [str(dump), str(input_path), str(qp_path), str(solution_path)],
        text=True, capture_output=True)
    if proc.stdout:
        print(proc.stdout, end="")
    if proc.stderr:
        print(proc.stderr, end="")
    if proc.returncode not in (0, 1):
        print(f"FAIL {label}: mpc_qp_dump exited {proc.returncode}")
        return False

    _, _, _, P, q, A, l, u = read_qp(qp_path)
    diffs = {
        "P": max_finite_diff(P, case["P"]),
        "q": max_finite_diff(q, case["q"]),
        "A": max_finite_diff(A, case["A"]),
        "l": max_finite_diff(l, case["l"]),
        "u": max_finite_diff(u, case["u"]),
    }
    status, x = read_solution(solution_path)
    control_diff = (
        float(np.max(np.abs(x[:3] - case["solution"][:3])))
        if x.size >= 3 else float("inf"))

    print(f"--- {label} ---")
    for name, diff in diffs.items():
        print(f"{name}: max abs diff={diff:.3e} (tol=1e-8)")
    print(f"solution: cpp_status={status} control_diff={control_diff:.3e} (tol=1e-3)")
    ok = all(diff <= 1e-8 for diff in diffs.values()) and control_diff <= 1e-3
    print(f"{'PASS' if ok else 'FAIL'} {label}")
    return ok


def main() -> int:
    parser = argparse.ArgumentParser()
    root = Path(__file__).resolve().parents[2]
    parser.add_argument(
        "--dump", default=str(root / "nav_ws_core/build/mpc_controller/mpc_qp_dump"),
        help="path to the built mpc_qp_dump executable")
    args = parser.parse_args()
    dump = Path(args.dump)
    if not dump.exists():
        print(f"FAIL: mpc_qp_dump not found: {dump}")
        print("build with: cd nav_ws_core && colcon build --packages-select mpc_controller")
        return 2

    identity = np.eye(2)
    zero = np.zeros(2)
    angle = 0.4
    rotation = np.array([
        [np.cos(angle), -np.sin(angle)],
        [np.sin(angle), np.cos(angle)],
    ])
    translation = np.array([0.3, -0.2])

    ok = True
    with tempfile.TemporaryDirectory(prefix="mpc_field_check_") as tmp:
        tmp_dir = Path(tmp)
        ok &= run_case(dump, make_case(identity, zero), tmp_dir, "identity")
        ok &= run_case(dump, make_case(rotation, translation), tmp_dir, "rotated")
    print("RESULT:", "PASS" if ok else "FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
