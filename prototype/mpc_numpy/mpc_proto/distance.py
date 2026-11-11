"""解析距离场（圆障碍 + 轴对齐墙体）：外缘间隙 d 与解析梯度 ∇d。

静态障碍：v = 0 的圆特例。动态障碍：匀速直线短期预测 p_j(t) = p_j0 + v_j t
（docs/MPC公式化.md §5，VO 管道的 QP 形式）。

墙体（契约 v1.4 `walls: {x0,y0,x1,y1}`）：轴对齐实心矩形、含壁厚，用欧氏
box 有符号距离（外部正、内部负）——保窄道间距指标不失真；圆链近似仅退化保底。
"""
from __future__ import annotations

from collections.abc import Sequence
from dataclasses import dataclass

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
class Wall:
    """轴对齐实心矩形墙（契约 walls: {x0,y0,x1,y1}，含壁厚，静态）。"""

    x0: float
    y0: float
    x1: float
    y1: float

    def __post_init__(self) -> None:
        if not (float(self.x0) < float(self.x1) and float(self.y0) < float(self.y1)):
            raise ValueError(f"墙体角点须满足 x0<x1、y0<y1：{(self.x0, self.y0, self.x1, self.y1)}")

    @property
    def dynamic(self) -> bool:
        return False

    @property
    def center(self) -> np.ndarray:
        return np.array([(self.x0 + self.x1) / 2.0, (self.y0 + self.y1) / 2.0])

    @property
    def half_extents(self) -> np.ndarray:
        return np.array([(self.x1 - self.x0) / 2.0, (self.y1 - self.y0) / 2.0])

    def signed_distance(self, p: np.ndarray) -> tuple[float, np.ndarray]:
        """到矩形边界的欧氏有符号距离（外部 >0、内部 <0）与向外单位梯度。"""
        p = np.asarray(p, dtype=float)
        c = self.center
        h = self.half_extents
        dx, dy = float(p[0] - c[0]), float(p[1] - c[1])
        qx, qy = abs(dx) - float(h[0]), abs(dy) - float(h[1])
        sx = 1.0 if dx >= 0.0 else -1.0
        sy = 1.0 if dy >= 0.0 else -1.0

        if qx > 0.0 or qy > 0.0:
            ox, oy = max(qx, 0.0), max(qy, 0.0)
            d = float(np.hypot(ox, oy))
            if d < 1e-12:  # 角点奇异：确定性回退到 x 面法向
                return 0.0, np.array([sx, 0.0])
            return d, np.array([(ox / d) * sx, (oy / d) * sy])

        # 内部 / 边界：梯度取最近面的外法向
        if qx >= qy:
            return float(qx), np.array([sx, 0.0])
        return float(qy), np.array([0.0, sy])


@dataclass
class Clearance:
    d: float  # 外缘间隙 = 有符号距离 − r_r；> 0 无接触
    grad: np.ndarray  # ∇d 对机器人中心位置 (2,)


SceneGeometry = Obstacle | Wall


def clearance(p: np.ndarray, obs: SceneGeometry, tau: float, robot_radius: float) -> Clearance:
    """解析间隙与解析梯度。圆障碍 ‖p−c‖ < 1e-9 时梯度取 (0,1)（奇异保护）；
    墙体忽略 tau（静态），梯度为 box 外法向。"""
    if isinstance(obs, Wall):
        d, grad = obs.signed_distance(p)
        return Clearance(d=d - robot_radius, grad=grad)

    c = obs.center_at(tau)
    delta = np.asarray(p, dtype=float)[:2] - c
    dist = float(np.linalg.norm(delta))
    r_sum = obs.radius + robot_radius
    if dist < 1e-9:
        return Clearance(d=-r_sum, grad=np.array([0.0, 1.0]))
    return Clearance(d=dist - r_sum, grad=delta / dist)


def min_clearance(
    p: np.ndarray, obstacles: Sequence[SceneGeometry], tau: float, robot_radius: float
) -> float:
    if not obstacles:
        return float("inf")
    return min(clearance(p, o, tau, robot_radius).d for o in obstacles)
