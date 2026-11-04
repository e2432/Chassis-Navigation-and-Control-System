"""基准系统的数据契约（W1/E04 定义，实现在 E14–E16）。

设计原则（来自路线书 E15/E16 的硬要求）：
  1. 失败任务也必须有结果文件，且失败原因可区分：成功/碰撞/超时/导航失败/基础设施故障；
  2. 基础设施故障**不得**悄悄记成成功，也不得从日志中消失；
  3. 缺失或无意义的指标存 null 并标明状态，不统一填 0；
  4. 每个结果都能追溯到 run_id、场景实例、控制器、种子、配置哈希与原始记录路径。

本文件只定义结构，不做任何 ROS 2 调用，因此可以被算法成员在没有仿真环境时直接复用。
"""

from __future__ import annotations

import json
from dataclasses import dataclass, field, asdict
from enum import Enum
from typing import Any, Dict, List, Optional

#: 基础设施故障的进程退出码。批量脚本据此区分“实验失败”和“环境坏了”。
INFRASTRUCTURE_EXIT_CODE = 3


class RunStatus(str, Enum):
    """一次基准任务的最终状态（路线书 E15 建议的 5 类，逐项对应）。"""

    SUCCESS = "success"                       # 到达目标且未碰撞、未超时
    COLLISION = "collision"                   # 发生碰撞/穿透
    TIMEOUT = "timeout"                        # 超过任务超时
    NAVIGATION_FAILURE = "navigation_failure"  # 导航栈明确失败/取消/无可行路径
    INFRASTRUCTURE_ERROR = "infrastructure_error"  # 仿真/ROS/采集故障，结果不可用


class BenchmarkNotImplementedError(NotImplementedError):
    """接口已定义但尚未实现时抛出，消息里必须写明对应的 E 编号。"""


class InfrastructureError(RuntimeError):
    """仿真或 ROS 基础设施故障。捕获方必须把它写成 infrastructure_error 结果，不得吞掉。"""


#: 七项指标键（路线书 E16）。公式与好坏方向由双方在 W3 前定稿，见 docs/指标定义.md。
METRIC_KEYS = (
    "success_rate",          # 成功率：成功任务数 / 有效任务数
    "time_to_goal",          # 到达时间：目标接受至成功结束的仿真时间
    "path_length",           # 路径长度：真值二维位置按统一采样间隔累加
    "smoothness",            # 平滑度：速度变化的固定定义（由算法侧定稿）
    "min_clearance",         # 最小间距：整机外缘与障碍外缘的最小距离
    "estop_count",           # 急停次数：由未触发变触发的事件数（连续触发算 1 次）
    "dynamic_response_rate",  # 动态应变率：预定义动态遭遇事件中成功应对的占比
)


@dataclass
class Pose2D:
    """平面位姿。角度单位 rad，位置单位 m，坐标系由容器字段声明。"""

    x: float
    y: float
    yaw: float
    frame_id: str = "map"


@dataclass
class Scenario:
    """一个可复现的场景实例（路线书 E14 要求保存的最小字段集）。

    注意：这是**实例**，不是模板。随机化后必须把实际取值写进来，
    只保存种子不足以还原（E21 明确要求）。
    """

    scenario_id: str
    category: str                  # static / pedestrian / narrow / deadend / mixed
    asset_version: str             # 静态资产版本（对应 robot_sim/config/scenarios_assets.yaml）
    map_version: str
    start_pose: Pose2D
    goal_pose: Pose2D
    seed: int
    task_timeout_s: float
    goal_tolerance_m: float
    goal_tolerance_yaw_rad: float
    obstacles: List[Dict[str, Any]] = field(default_factory=list)
    dynamic_tracks: List[Dict[str, Any]] = field(default_factory=list)
    solvable: bool = True          # 场景是否应当可解；无解任务作为恢复测试单列
    notes: str = ""

    def to_json(self) -> str:
        return json.dumps(asdict(self), ensure_ascii=False, indent=2)

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> "Scenario":
        payload = dict(data)
        payload["start_pose"] = Pose2D(**payload["start_pose"])
        payload["goal_pose"] = Pose2D(**payload["goal_pose"])
        return cls(**payload)


@dataclass
class RunResult:
    """一次运行的完整结果。字段与路线书 E15 的 JSON 示例一一对应。"""

    run_id: str                     # 例如 static_001_dwb_seed0001
    scenario_id: str
    controller: str                 # dwb / mppi / teb / mpc
    seed: int
    status: RunStatus
    reason: str                     # goal_reached / collision / timeout / ...
    metrics: Dict[str, Optional[float]] = field(default_factory=dict)
    metadata: Dict[str, Any] = field(default_factory=dict)   # git_commit / config_hash / asset_version
    raw_data_path: Optional[str] = None
    wall_time_s: Optional[float] = None      # 诊断用；统计口径一律用仿真时间

    def to_dict(self) -> Dict[str, Any]:
        data = asdict(self)
        data["status"] = self.status.value
        return data

    def to_json(self) -> str:
        return json.dumps(self.to_dict(), ensure_ascii=False, indent=2)


def run(scenario: Scenario, controller: str = "dwb") -> RunResult:
    """执行一次基准任务（路线书 `benchmark.run(scenario)`）。

    接口语义（E04 定契约，E15 实现）：

    输入：
        scenario   —— 场景实例；必须自带全部可复现字段
        controller —— 控制器名（dwb / mppi / teb / mpc）

    输出：
        RunResult  —— 无论成功、碰撞、超时、导航失败都返回一个结果；
                      基础设施故障抛 InfrastructureError，由调用方写成
                      status=infrastructure_error 的结果文件，不得丢弃。

    副作用：
        * 结束时保证底盘速度归零；
        * 结果与原始记录写入运行目录，并在 metadata 记录版本信息。

    当前状态：**未实现**。计划 E15（W4，2026-11-22 前）。
    """
    raise BenchmarkNotImplementedError(
        "benchmark.run 尚未实现：计划 E15（W4）。契约见 docs/接口契约.md 第 8 节。"
    )
