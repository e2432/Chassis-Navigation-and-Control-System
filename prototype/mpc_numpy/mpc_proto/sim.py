"""自包含滚动推演引擎 + W2 验收四条（docs/MPC公式化.md §9）。

闭环于完美运动学模型；障碍为场景脚本数据；dt_sim 可细于控制步长。
"""
from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from .chassis import Chassis
from .distance import Obstacle, clearance, min_clearance
from .mpc import MPCConfig, MPCSolver, rollout
from .reference import ReferencePath, wrap_angle
from .scenarios import Scenario

# 验收口径
ARRIVE_POS = 0.25  # m，指标定义
ARRIVE_YAW = 0.25  # rad
TV_PER_TICK_BOUND = 0.30  # Δu 总变差有界：归一化 TV / 拍数 < 0.30


@dataclass
class RunResult:
    scenario: str
    arrived: bool
    t_arrival: float | None
    ticks: int
    solve_ok: int
    solve_fail: int
    min_clearance: float  # 外缘间隙（真值几何）
    tv: float  # 归一化控制量总变差 Σ(‖Δv‖/v_max + |Δω|/ω_max)
    tv_per_tick: float
    path_length: float
    max_slack: float
    alarm: bool
    zs: np.ndarray = field(default_factory=lambda: np.zeros((0, 3)))  # 控制拍位姿
    us: np.ndarray = field(default_factory=lambda: np.zeros((0, 3)))
    t_ctrl: np.ndarray = field(default_factory=lambda: np.zeros(0))
    clearances: np.ndarray = field(default_factory=lambda: np.zeros(0))  # 仿真细步间隙

    def acceptance(self) -> dict:
        """验收四条（②③④；①一条命令由 run_w2.sh 保证）。"""
        return {
            "solve_every_tick": self.solve_fail == 0,
            "clearance_positive": self.min_clearance > 0.0,
            "tv_bounded": np.isfinite(self.tv_per_tick) and self.tv_per_tick < TV_PER_TICK_BOUND,
        }

    def acceptance_pass(self) -> bool:
        return all(self.acceptance().values())


def run(
    scenario: Scenario,
    chassis: Chassis,
    cfg: MPCConfig,
    dt_sim: float = 0.01,
    log_every: int = 0,
) -> RunResult:
    solver = MPCSolver(chassis, cfg)
    ref = ReferencePath(
        waypoints=scenario.waypoints,
        goal_yaw=float(scenario.goal[2]),
        v_ref=0.6,
        N=cfg.N,
        dt=cfg.dt,
    )

    z = scenario.start.astype(float).copy()
    dt_ctrl = cfg.dt
    substeps = max(1, int(round(dt_ctrl / dt_sim)))
    dt_sub = dt_ctrl / substeps

    zs, us, t_ctrl = [], [], []
    clearances = []  # 细步真值间隙
    solve_ok = solve_fail = 0
    max_slack = 0.0
    arrived = False
    t_arrival = None
    path_length = 0.0
    p_prev = z[:2].copy()

    n_ticks = int(round(scenario.t_max / dt_ctrl))
    for tick in range(n_ticks):
        t = tick * dt_ctrl
        # 障碍状态（场景脚本数据，t 时刻）
        obs_now = [
            Obstacle(o.obs_id, o.center_at(t), o.v, o.radius) for o in scenario.obstacles
        ]
        res = solver.solve(z, obs_now, ref, t_now=t)
        if res.success:
            solve_ok += 1
            max_slack = max(max_slack, res.max_slack)
        else:
            solve_fail += 1
        u = res.u

        zs.append(z.copy())
        us.append(u.copy())
        t_ctrl.append(t)

        # 到达判定（复用指标定义 0.25 m / 0.25 rad）
        pos_err = float(np.linalg.norm(z[:2] - scenario.goal[:2]))
        yaw_err = abs(wrap_angle(z[2] - scenario.goal[2]))
        if pos_err <= ARRIVE_POS and yaw_err <= ARRIVE_YAW:
            arrived = True
            t_arrival = t
            break

        # 细步积分（完美运动学，零阶保持 u）
        for s in range(substeps):
            tau = t + s * dt_sub
            z = rollout(z, u.reshape(1, 3), dt_sub)[1]
            clearances.append(
                min_clearance(z[:2], scenario.obstacles, tau, chassis.radius)
            )
            path_length += float(np.linalg.norm(z[:2] - p_prev))
            p_prev = z[:2].copy()

        if log_every and tick % log_every == 0:
            print(
                f"  [{scenario.name}] t={t:5.2f}s pos=({z[0]:6.2f},{z[1]:6.2f}) "
                f"u=({u[0]:5.2f},{u[1]:5.2f},{u[2]:5.2f}) "
                f"clear={min_clearance(z[:2], scenario.obstacles, t, chassis.radius):5.3f}"
            )

    # 到达时刻的最终间隙记录（若循环 break，补最后细步不必要；clearances 已含全程）
    if arrived and not clearances:
        clearances.append(min_clearance(z[:2], scenario.obstacles, t_arrival, chassis.radius))

    us_arr = np.asarray(us).reshape(-1, 3)
    # 归一化控制量总变差（指标 4 口径，控制序列 20 Hz）
    if us_arr.shape[0] >= 2:
        dv = np.linalg.norm(np.diff(us_arr[:, :2], axis=0), axis=1) / max(chassis.v_max, 1e-9)
        dw = np.abs(np.diff(us_arr[:, 2])) / max(chassis.omega_max, 1e-9)
        tv = float(np.sum(dv + dw))
    else:
        tv = 0.0
    ticks = us_arr.shape[0]
    tv_per_tick = tv / max(ticks, 1)

    min_c = float(np.min(clearances)) if clearances else float("inf")

    return RunResult(
        scenario=f"{scenario.name} {scenario.desc}",
        arrived=arrived,
        t_arrival=t_arrival,
        ticks=ticks,
        solve_ok=solve_ok,
        solve_fail=solve_fail,
        min_clearance=min_c,
        tv=tv,
        tv_per_tick=tv_per_tick,
        path_length=path_length,
        max_slack=max_slack,
        alarm=solver.alarm,
        zs=np.asarray(zs).reshape(-1, 3),
        us=us_arr,
        t_ctrl=np.asarray(t_ctrl),
        clearances=np.asarray(clearances),
    )
