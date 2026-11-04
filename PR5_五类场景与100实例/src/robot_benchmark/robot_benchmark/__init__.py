"""robot_benchmark —— 基准系统（场景编排、自动执行、指标采集、结果汇总）。

W1（E04）只交付**契约**：数据结构、状态枚举、指标键、异常语义。
真正执行任务的实现分别在：
  * E14 —— 五类场景配置与实例生成
  * E15 —— 一次任务的自动执行（benchmark.run）
  * E16 —— 七项指标采集与汇总

契约文档：docs/接口契约.md。任何改动先改文档，再改这里。
"""

from .contract import (  # noqa: F401
    INFRASTRUCTURE_EXIT_CODE,
    METRIC_KEYS,
    Scenario,
    RunResult,
    RunStatus,
    BenchmarkNotImplementedError,
    InfrastructureError,
)
from .metrics import METRIC_SPECS  # noqa: F401

__all__ = [
    "INFRASTRUCTURE_EXIT_CODE",
    "METRIC_KEYS",
    "METRIC_SPECS",
    "Scenario",
    "RunResult",
    "RunStatus",
    "BenchmarkNotImplementedError",
    "InfrastructureError",
]
