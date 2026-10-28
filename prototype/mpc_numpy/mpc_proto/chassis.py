"""底盘参数化描述载入（契约 schema v1）。

limits 全部进 QP；`type: diff` → `vy_max = 0`（迁移冒烟零改动的落点）。
"""
from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import yaml


@dataclass
class Chassis:
    type: str
    radius: float
    v_min: float
    v_max: float
    vy_max: float
    omega_max: float
    acc_max: float
    alpha_max: float

    def u_min(self):
        import numpy as np

        return np.array([self.v_min, -self.vy_max, -self.omega_max])

    def u_max(self):
        import numpy as np

        return np.array([self.v_max, self.vy_max, self.omega_max])

    def du_max(self, dt: float):
        """控制增量界（线性约束）：acc_max/alpha_max × dt。"""
        import numpy as np

        return np.array([self.acc_max * dt, self.acc_max * dt, self.alpha_max * dt])


def load_chassis(path: str | Path) -> Chassis:
    raw = yaml.safe_load(Path(path).read_text(encoding="utf-8"))["chassis"]
    limits = raw["limits"]
    ctype = str(raw["type"])
    vy_max = float(limits["vy_max"])
    # 契约：diff → vy_max 置 0，算法不感知 type
    if ctype == "diff":
        vy_max = 0.0
    return Chassis(
        type=ctype,
        radius=float(raw["footprint"]["radius"]),
        v_min=float(limits["v_min"]),
        v_max=float(limits["v_max"]),
        vy_max=vy_max,
        omega_max=float(limits["omega_max"]),
        acc_max=float(limits["acc_max"]),
        alpha_max=float(limits["alpha_max"]),
    )
