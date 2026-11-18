# 基线成绩汇总（工程侧采集）

| 场景 | run 数 | 成功率 | 到达中位 [s] | 路径中位 [m] | 平滑度中位 | 间距中位 [m] | 间距最坏 [m] | 急停合计 |
|---|---|---|---|---|---|---|---|---|
| static_avoid | 20 | 90.0% | 19.375 | 10.2709 | 19.0858 | 0.167 | 0.0693 | 0 |
| dynamic_pedestrian | 20 | 75.0% | 28.1 | 9.75 | 18.1651 | 0.8034 | 0.6975 | 0 |
| narrow_passage | 20 | 75.0% | 21.65 | 11.6911 | 16.0449 | 0.2066 | 0.0397 | 5 |
| dead_end | 20 | 85.0% | 22.5333 | 12.117 | 22.7097 | 0.1144 | 0.0453 | 3 |
| mixed | 20 | 90.0% | 33.775 | 12.3733 | 33.6619 | 0.1249 | 0.0465 | 0 |

套件级 `dynamic_stress`（dynamic_pedestrian + mixed 合并成功率）：**82.5%**

## 异常清单（失败 run，必须逐条说明）
| 场景 | seed | run | 控制器 | failure_class | 最小间距 [m] | 结束时刻 [s] |
|---|---|---|---|---|---|---|
| static_avoid | 105 | 5 | dwa | other | 0.2543 | 43.4333 |
| static_avoid | 112 | 12 | dwa | other | 0.1947 | 45.4667 |
| dynamic_pedestrian | 202 | 2 | dwa | timeout | 0.6975 | 60.4833 |
| dynamic_pedestrian | 204 | 4 | dwa | timeout | 0.7055 | 60.1166 |
| dynamic_pedestrian | 210 | 10 | dwa | timeout | 0.7313 | 60.4834 |
| dynamic_pedestrian | 213 | 13 | dwa | timeout | 0.7029 | 60.5 |
| dynamic_pedestrian | 214 | 14 | dwa | timeout | 0.8148 | 60.4833 |
| narrow_passage | 303 | 3 | dwa | other | 1.2207 | 30.7833 |
| narrow_passage | 306 | 6 | dwa | other | 3.0718 | 30.7667 |
| narrow_passage | 311 | 11 | dwa | other | 4.5119 | 30.75 |
| narrow_passage | 316 | 16 | dwa | other | 0.1089 | 42.25 |
| narrow_passage | 318 | 18 | dwa | other | 0.0481 | 46.7667 |
| dead_end | 409 | 9 | dwa | other | 0.1244 | 49.5833 |
| dead_end | 410 | 10 | dwa | other | 0.0799 | 40.55 |
| dead_end | 416 | 16 | dwa | other | 1.853 | 33.5 |
| mixed | 502 | 2 | dwa | other | 0.2033 | 55.65 |
| mixed | 506 | 6 | dwa | other | 0.1995 | 51.3333 |
