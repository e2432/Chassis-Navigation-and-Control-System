# 基线成绩汇总（工程侧采集）

| 场景 | run 数 | 成功率 | 到达中位 [s] | 路径中位 [m] | 平滑度中位 | 间距中位 [m] | 间距最坏 [m] | 急停合计 |
|---|---|---|---|---|---|---|---|---|
| static_avoid | 20 | 90.0% | 18.25 | 10.8459 | 16.5453 | 0.2775 | 0.2098 | 0 |
| dynamic_pedestrian | 20 | 100.0% | 29.3666 | 15.7319 | 45.4926 | 0.9728 | 0.6224 | 0 |
| narrow_passage | 20 | 80.0% | 19.9167 | 11.7941 | 12.2343 | 0.3412 | 0.2632 | 5 |
| dead_end | 20 | 85.0% | 20.7167 | 12.3037 | 16.4606 | 0.3013 | 0.2214 | 5 |
| mixed | 20 | 70.0% | 32.9 | 18.089 | 49.934 | 0.2037 | -0.0283 | 5 |

套件级 `dynamic_stress`（dynamic_pedestrian + mixed 合并成功率）：**85.0%**

## 异常清单（失败 run，必须逐条说明）
| 场景 | seed | run | 控制器 | failure_class | 最小间距 [m] | 结束时刻 [s] |
|---|---|---|---|---|---|---|
| static_avoid | 107 | 7 | teb | other | 0.2098 | 50.0833 |
| static_avoid | 118 | 18 | teb | other | 0.2306 | 40.9333 |
| narrow_passage | 306 | 6 | teb | other | 3.0718 | 30.75 |
| narrow_passage | 311 | 11 | teb | other | 4.5119 | 30.75 |
| narrow_passage | 316 | 16 | teb | other | 0.2708 | 42.35 |
| narrow_passage | 318 | 18 | teb | other | 0.3736 | 42.8833 |
| dead_end | 409 | 9 | teb | other | 0.3237 | 48.1167 |
| dead_end | 410 | 10 | teb | other | 0.3292 | 40.5667 |
| dead_end | 416 | 16 | teb | other | 2.5609 | 30.75 |
| mixed | 502 | 2 | teb | collision | -0.014 | 40.25 |
| mixed | 505 | 5 | teb | timeout | 0.2411 | 75.5 |
| mixed | 512 | 12 | teb | other | 0.2464 | 54.1167 |
| mixed | 514 | 14 | teb | other | 0.0154 | 50.7167 |
| mixed | 516 | 16 | teb | collision | -0.0283 | 42.8834 |
| mixed | 520 | 20 | teb | collision | -0.0066 | 44.8834 |
