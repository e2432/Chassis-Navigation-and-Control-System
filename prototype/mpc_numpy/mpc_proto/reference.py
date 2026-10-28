"""时变参考：折线参考几何 + 时间律推进 + 目标截断（docs/MPC公式化.md §2）。

参考几何 = 场景脚本给的折线路径（W2 起到 NavFn 全局路径的替身；全局规划负责
静态绕行的几何，局部控制器负责跟踪 + 避障余量 + 动态避让）。时间律
s(t) = min(v_ref·t, L)：x_ref(t_k) 取绝对时刻 t_now + k·dt 的弧长位姿，
不逐拍重新锚定到机器人投影——弱沿程权重（q_long ≤ 0.1·q_lat）下重新锚定会让
e_long 只看一步以远，陷入「停在障碍前」的短视最优；直参考穿障时横向代价
（q_lat）也压过沿程收益，静态绕行的几何必须由参考路径给出。
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np


def wrap_angle(a: float) -> float:
    return float((a + np.pi) % (2.0 * np.pi) - np.pi)


@dataclass
class ReferencePath:
    """折线参考：waypoints（含起点→目标）；s(t) = min(v_ref·t, L)，目标处截断。"""

    waypoints: np.ndarray  # (M,2)
    goal_yaw: float
    v_ref: float
    N: int
    dt: float

    def __post_init__(self):
        wp = np.asarray(self.waypoints, dtype=float)
        if wp.ndim != 2 or wp.shape[0] < 2:
            raise ValueError("waypoints 至少两个点")
        self.waypoints = wp
        seg = np.diff(wp, axis=0)
        self.seg_len = np.linalg.norm(seg, axis=1)
        self.cum = np.concatenate([[0.0], np.cumsum(self.seg_len)])
        self.length = float(self.cum[-1])
        # 零长段保护
        self.seg_dir = np.zeros_like(seg)
        for i, L in enumerate(self.seg_len):
            self.seg_dir[i] = seg[i] / L if L > 1e-12 else np.array([1.0, 0.0])

    def _pose_at(self, s: float) -> tuple[np.ndarray, float]:
        """弧长 s → (位姿 xy + θ_ref, 局部前馈速度 v）。目标处截断。"""
        if s >= self.length - 1e-12:
            goal_xy = self.waypoints[-1]
            return np.array([goal_xy[0], goal_xy[1], self.goal_yaw]), 0.0
        i = int(np.searchsorted(self.cum, s, side="right") - 1)
        i = min(max(i, 0), len(self.seg_len) - 1)
        frac = (s - self.cum[i]) / self.seg_len[i] if self.seg_len[i] > 1e-12 else 0.0
        xy = self.waypoints[i] + self.seg_dir[i] * (frac * self.seg_len[i])
        theta = float(np.arctan2(self.seg_dir[i][1], self.seg_dir[i][0]))
        return np.array([xy[0], xy[1], theta]), self.v_ref

    def sequence(self, t_now: float) -> tuple[np.ndarray, np.ndarray]:
        """返回 (z_ref (N+1,3), u_ref (N,3))；z_ref,k 对应绝对时刻 t_now + k·dt。

        前馈 u_ref：v_x = 局部速度（机体系沿切向）、v_y = 0、ω = 切向变化率；
        截断后全零。仅用于 LTV 名义序列，不进误差定义。
        """
        z_ref = np.zeros((self.N + 1, 3))
        u_ref = np.zeros((self.N, 3))
        thetas = np.zeros(self.N + 1)
        pinned = np.zeros(self.N + 1, dtype=bool)
        for k in range(self.N + 1):
            s_k = min(self.v_ref * (t_now + k * self.dt), self.length)
            pose, _ = self._pose_at(s_k)
            z_ref[k] = pose
            thetas[k] = pose[2]
            pinned[k] = s_k >= self.length - 1e-12
        for k in range(self.N):
            if pinned[k]:
                u_ref[k] = 0.0
            else:
                u_ref[k] = [self.v_ref, 0.0, wrap_angle(thetas[k + 1] - thetas[k]) / self.dt]
        return z_ref, u_ref

    def error_matrix(self, theta_ref: float) -> np.ndarray:
        """M_k：行序 e_lat, e_long, e_θ（docs/MPC公式化.md §3）。"""
        s, c = np.sin(theta_ref), np.cos(theta_ref)
        return np.array([[-s, c, 0.0], [c, s, 0.0], [0.0, 0.0, 1.0]])


def straight_path(start: np.ndarray, goal: np.ndarray, v_ref: float, N: int, dt: float) -> ReferencePath:
    return ReferencePath(
        waypoints=np.array([start[:2], goal[:2]]),
        goal_yaw=float(goal[2]),
        v_ref=v_ref,
        N=N,
        dt=dt,
    )
