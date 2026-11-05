"""七项指标的元数据（W1 草案，公式由双方在 W3 前定稿）。

这里只描述“指标是什么、单位、好坏方向、缺数据怎么办”，不实现计算。
计算实现在 E16。任何口径变化必须先改 docs/指标定义.md。
"""

from typing import Dict

#: higher_is_better: 数值越大越好 / 越小越好 / None 表示需要按公式定义（例如平滑度）
METRIC_SPECS: Dict[str, dict] = {
    "success_rate": {
        "unit": "ratio",
        "higher_is_better": True,
        "aggregation": "batch",          # 批量层指标，单次运行留 null
        "null_policy": "永不填 0；分母为 0 时记 null 并标 not_applicable",
        "open_questions": [
            "碰撞后仍到达是否算失败（草案：算失败）",
            "基础设施错误的任务是否计入有效任务数（草案：不计入，单独列报）",
            "必须同时给出分子/分母，不允许只给百分比",
        ],
    },
    "time_to_goal": {
        "unit": "s (simulation)",
        "higher_is_better": False,
        "aggregation": "per_run",
        "null_policy": "失败任务记 null，禁止伪造到达时间",
        "open_questions": ["超时阈值是否就用 scenario.task_timeout_s", "计时起点：目标被接受的那一帧"],
    },
    "path_length": {
        "unit": "m",
        "higher_is_better": False,
        "aggregation": "per_run",
        "null_policy": "采样异常时记 null 并附 error 说明",
        "open_questions": ["统一采样间隔取值", "重置跳变如何剔除（草案：相邻采样位移 > 1 m 视为跳变丢弃）"],
    },
    "smoothness": {
        "unit": "待定（归一化后无量纲）",
        "higher_is_better": None,
        "aggregation": "per_run",
        "null_policy": "采样不足时记 null",
        "open_questions": [
            "用速度一阶差的平方和还是总变差",
            "线速度(m/s)与角速度(rad/s)单位不同，如何归一化",
            "采样率是否需要重采样到固定频率",
        ],
    },
    "min_clearance": {
        "unit": "m",
        "higher_is_better": True,
        "aggregation": "per_run",
        "null_policy": "全程无有效障碍观测时记 null，不记 0 也不记无穷",
        "open_questions": ["地面是否排除", "接触/穿透如何编码（草案：<=0 记为接触并在 result 里标 collision）"],
    },
    "estop_count": {
        "unit": "count",
        "higher_is_better": False,
        "aggregation": "per_run",
        "null_policy": "安全层未启用时记 null（不能当作 0）",
        "open_questions": ["持续 30 帧急停算 1 次、恢复去抖窗口", "按次任务统计还是按遭遇事件统计"],
    },
    "dynamic_response_rate": {
        "unit": "ratio",
        "higher_is_better": True,
        "aggregation": "batch",
        "null_policy": "没有动态遭遇事件的场景记 N/A 而不是 0",
        "open_questions": ["事件窗口如何界定", "成功/失败条件（草案：不碰撞且最终到达）"],
    },
}

#: 指标方向汇总，供对比脚本按方向比较“不劣/更优”
DIRECTION = {k: v["higher_is_better"] for k, v in METRIC_SPECS.items()}
