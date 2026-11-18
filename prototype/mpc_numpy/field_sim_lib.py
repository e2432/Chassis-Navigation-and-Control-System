"""Shared synthetic closed-loop harness for the static-field MPC (conda pro)."""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from mpc_proto.chassis import Chassis
from mpc_proto.distance import Obstacle
from mpc_proto.distance_field import GridDistanceField
from mpc_proto.mpc import MPCConfig, MPCSolver
from mpc_proto.reference import ReferencePath

RESOLUTION = 0.05
WIDTH = 140       # x: 0..7 m
HEIGHT = 80       # y: -2..2 m
ORIGIN_X = 0.0
ORIGIN_Y = -2.0


def build_map(kind: str) -> GridDistanceField:
    occupied = np.zeros((HEIGHT, WIDTH), dtype=bool)

    def mark(x0: float, x1: float, y0: float, y1: float) -> None:
        ix0 = int(np.floor((x0 - ORIGIN_X) / RESOLUTION))
        ix1 = int(np.ceil((x1 - ORIGIN_X) / RESOLUTION))
        iy0 = int(np.floor((y0 - ORIGIN_Y) / RESOLUTION))
        iy1 = int(np.ceil((y1 - ORIGIN_Y) / RESOLUTION))
        occupied[max(iy0, 0):min(iy1, HEIGHT), max(ix0, 0):min(ix1, WIDTH)] = True

    if kind == "bump":
        mark(2.0, 3.6, 0.35, 0.55)
    elif kind == "corridor":
        mark(0.0, 6.5, -0.65, -0.55)
        mark(0.0, 6.5, 0.55, 0.65)
    elif kind == "narrow":
        mark(0.0, 6.5, -0.60, -0.50)
        mark(0.0, 6.5, 0.50, 0.60)
    elif kind == "dead_end":
        mark(0.0, 4.1, 0.80, 0.90)
        mark(0.0, 4.1, -0.90, -0.80)
        mark(4.0, 4.1, -0.90, 0.90)
    elif kind == "pedestrian":
        pass
    else:
        raise ValueError(f"unknown map kind: {kind}")

    return GridDistanceField.from_occupied(
        occupied, RESOLUTION, ORIGIN_X, ORIGIN_Y, max_distance=5.0)


@dataclass
class CaseResult:
    kind: str
    reached: bool
    time_s: float
    steps: int
    final: np.ndarray
    min_static_clearance_m: float
    min_dynamic_clearance_m: float
    min_y: float
    max_y: float
    max_slack: float


def run_case(
    kind: str,
    d_safe: float = 0.30,
    rho_slack: float = 80.0,
    terminal_factor: float = 2.0,
    v_ref: float = 0.5,
    N: int = 30,
    timeout_s: float = 30.0,
    goal_x: float = 5.5,
) -> CaseResult:
    field = build_map(kind)
    chassis = Chassis(
        type="omni", radius=0.30, v_min=-1.0, v_max=1.0, vy_max=1.0,
        omega_max=1.5, acc_max=1.0, alpha_max=2.0)
    cfg = MPCConfig(
        N=N, dt=0.05, v_ref=v_ref, d_safe=d_safe, rho_slack=rho_slack,
        terminal_factor=terminal_factor, n_sqp=2, eps_abs=1e-3, eps_rel=1e-3,
        max_iter=10000, slack=True)

    if kind == "dead_end":
        waypoints = np.array([[0.0, 0.0], [3.0, 0.0], [0.3, 0.4]])
        goal = waypoints[-1].copy()
    else:
        waypoints = np.array([[0.0, 0.0], [goal_x, 0.0]])
        goal = waypoints[-1].copy()

    reference = ReferencePath(
        waypoints=waypoints, goal_yaw=0.0, v_ref=v_ref, N=N, dt=cfg.dt)
    solver = MPCSolver(chassis, cfg)

    # Contract pedestrian: x=2.8, y=0.8*(t-6.0), dir=+1, r=0.3.
    ped_x, ped_speed, ped_meet, ped_radius = 2.8, 0.8, 6.0, 0.3

    z = np.array([0.0, 0.0, 0.0])
    t = 0.0
    min_static = float("inf")
    min_dynamic = float("inf")
    min_y = 0.0
    max_y = 0.0
    max_slack = 0.0
    reached = False
    steps = 0

    while t < timeout_s:
        obstacles = []
        if kind == "pedestrian":
            ped_y = ped_speed * (t - ped_meet)
            obstacles = [
                Obstacle(1, np.array([ped_x, ped_y]), np.array([0.0, ped_speed]),
                         ped_radius)]

        result = solver.solve(
            z, obstacles, reference, t_now=t, static_field=field)
        if result.success:
            u = result.u
            max_slack = max(max_slack, result.max_slack)
        else:
            u = np.zeros(3)

        x, y, theta = z
        vx, vy, omega = u
        z = np.array([
            x + (vx * np.cos(theta) - vy * np.sin(theta)) * cfg.dt,
            y + (vx * np.sin(theta) + vy * np.cos(theta)) * cfg.dt,
            theta + omega * cfg.dt,
        ])

        sample = field.sample(z[:2])
        if sample is not None:
            min_static = min(min_static, sample[0] - chassis.radius)
        if kind == "pedestrian":
            ped_y = ped_speed * (t - ped_meet)
            clearance = np.hypot(z[0] - ped_x, z[1] - ped_y) - (
                chassis.radius + ped_radius)
            min_dynamic = min(min_dynamic, clearance)

        min_y = min(min_y, z[1])
        max_y = max(max_y, z[1])
        t += cfg.dt
        steps += 1
        if np.hypot(z[0] - goal[0], z[1] - goal[1]) <= 0.25:
            reached = True
            break

    return CaseResult(
        kind=kind, reached=reached, time_s=t, steps=steps, final=z,
        min_static_clearance_m=min_static,
        min_dynamic_clearance_m=min_dynamic,
        min_y=min_y, max_y=max_y, max_slack=max_slack)
