"""解析距离场（圆近似障碍）：外缘间隙 d 与解析梯度 ∇d。

静态障碍：v = 0 的特例。动态障碍：匀速直线短期预测 p_j(t) = p_j0 + v_j t
（docs/MPC公式化.md §5，VO 管道的 QP 形式）。
"""
from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np


@dataclass
class Obstacle:
    """单圆近似障碍（契约 ObstacleState 的脚本数据形态）。"""

    obs_id: int
    p0: np.ndarray  # (2,) 当前时刻位置，m
    v: np.ndarray  # (2,) 速度，m/s（静态 = 0）
    radius: float  # m

    @property
    def dynamic(self) -> bool:
        return float(np.linalg.norm(self.v)) > 1e-9

    def center_at(self, tau: float) -> np.ndarray:
        """tau 秒后的预测中心（匀速直线）。"""
        return self.p0 + self.v * tau


@dataclass
class Clearance:
    d: float  # 外缘间隙 = ‖p − c‖ − (r_j + r_r)；> 0 无接触
    grad: np.ndarray  # ∇d 对机器人中心位置 (2,)


def clearance(p: np.ndarray, obs: Obstacle, tau: float, robot_radius: float) -> Clearance:
    """解析间隙与解析梯度。‖p−c‖ < 1e-9 时梯度取 (0,1)（奇异保护）。"""
    c = obs.center_at(tau)
    delta = np.asarray(p, dtype=float)[:2] - c
    dist = float(np.linalg.norm(delta))
    r_sum = obs.radius + robot_radius
    if dist < 1e-9:
        return Clearance(d=-r_sum, grad=np.array([0.0, 1.0]))
    return Clearance(d=dist - r_sum, grad=delta / dist)


def min_clearance(p: np.ndarray, obstacles: list[Obstacle], tau: float, robot_radius: float) -> float:
    if not obstacles:
        return float("inf")
    return min(clearance(p, o, tau, robot_radius).d for o in obstacles)
