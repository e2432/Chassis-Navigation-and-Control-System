"""W2 场景清单（docs/MPC公式化.md §9）：S1 静绕 / S2 动横穿 / S3 静+动混合。

参考几何（waypoints）= 场景脚本给的折线路径，NavFn 全局路径的 W2 替身：
静态绕行的几何由参考路径承担，MPC 负责跟踪 + 障碍余量 + 动态避让。
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .distance import Obstacle


@dataclass
class Scenario:
    name: str
    desc: str
    start: np.ndarray  # (3,)
    goal: np.ndarray  # (3,)
    waypoints: np.ndarray  # (M,2) 参考几何（含起点→目标）
    obstacles: list[Obstacle]
    t_max: float = 40.0  # 场景时限（s），W2 自定；W3 随 5 场景需求清单定稿


def s1_static_avoid() -> Scenario:
    # 静障 (4.0, 0.0) r=0.6：参考几何从上方绕行（最近点 y=1.35 → 间隙 0.45）
    return Scenario(
        name="S1",
        desc="静态绕障",
        start=np.array([0.0, 0.0, 0.0]),
        goal=np.array([8.0, 0.0, 0.0]),
        waypoints=np.array([[0.0, 0.0], [2.0, 1.0], [4.0, 1.35], [6.5, 1.0], [8.0, 0.0]]),
        obstacles=[Obstacle(1, np.array([4.0, 0.0]), np.zeros(2), 0.6)],
        t_max=40.0,
    )


def s2_dynamic_crossing() -> Scenario:
    # 直参考；行人 (4.0, −4.9) 以 0.7 m/s 横穿，与机器人约 t≈7 s 在 x≈4 相遇
    return Scenario(
        name="S2",
        desc="动态横穿（0.7 m/s 匀速行人）",
        start=np.array([0.0, 0.0, 0.0]),
        goal=np.array([8.0, 0.0, 0.0]),
        waypoints=np.array([[0.0, 0.0], [8.0, 0.0]]),
        obstacles=[Obstacle(2, np.array([4.0, -4.9]), np.array([0.0, 0.7]), 0.3)],
        t_max=40.0,
    )


def s3_mixed_passage() -> Scenario:
    # 通道：静障 (5.0, 0.8) r=0.45 从上方压道，对向动障 0.7 m/s 沿通道来
    return Scenario(
        name="S3",
        desc="静+动混合（通道 1 静障 + 1 对向动障，双障碍）",
        start=np.array([0.0, 0.0, 0.0]),
        goal=np.array([10.0, 0.0, 0.0]),
        waypoints=np.array([[0.0, 0.0], [3.0, -0.2], [5.0, -0.35], [7.5, -0.2], [10.0, 0.0]]),
        obstacles=[
            Obstacle(3, np.array([5.0, 0.8]), np.zeros(2), 0.45),
            Obstacle(4, np.array([10.5, -0.5]), np.array([-0.7, 0.0]), 0.3),
        ],
        t_max=50.0,
    )


ALL = [s1_static_avoid, s2_dynamic_crossing, s3_mixed_passage]


def by_name(name: str) -> Scenario:
    for fn in ALL:
        sc = fn()
        if sc.name == name:
            return sc
    raise KeyError(f"未知场景: {name}（可选 S1/S2/S3）")
