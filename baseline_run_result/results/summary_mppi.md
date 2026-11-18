# 基线成绩汇总（工程侧采集）

| 场景 | run 数 | 成功率 | 到达中位 [s] | 路径中位 [m] | 平滑度中位 | 间距中位 [m] | 间距最坏 [m] | 急停合计 |
|---|---|---|---|---|---|---|---|---|
| static_avoid | 20 | 95.0% | 21.75 | 10.5252 | 7.401 | 0.2361 | 0.0944 | 0 |
| dynamic_pedestrian | 20 | 100.0% | 30.55 | 9.8942 | 6.5544 | 0.9874 | 0.8695 | 0 |
| narrow_passage | 20 | 75.0% | 23.7 | 11.8754 | 6.9382 | 0.2254 | 0.1922 | 6 |
| dead_end | 20 | 85.0% | 24.9167 | 12.3648 | 7.9198 | 0.2157 | 0.1872 | 3 |
| mixed | 20 | 95.0% | 34.6 | 12.4853 | 10.0298 | 0.232 | 0.0834 | 0 |

套件级 `dynamic_stress`（dynamic_pedestrian + mixed 合并成功率）：**97.5%**

## 异常清单（失败 run，必须逐条说明）
| 场景 | seed | run | 控制器 | failure_class | 最小间距 [m] | 结束时刻 [s] |
|---|---|---|---|---|---|---|
| static_avoid | 105 | 5 | mppi | other | 0.3152 | 45.7167 |
| narrow_passage | 303 | 3 | mppi | other | 1.2239 | 30.7667 |
| narrow_passage | 306 | 6 | mppi | other | 3.0718 | 30.75 |
| narrow_passage | 311 | 11 | mppi | other | 4.5119 | 30.75 |
| narrow_passage | 316 | 16 | mppi | other | 0.2156 | 43.95 |
| narrow_passage | 318 | 18 | mppi | other | 0.2073 | 48.6833 |
| dead_end | 409 | 9 | mppi | other | 0.201 | 53.45 |
| dead_end | 410 | 10 | mppi | other | 0.1971 | 42.65 |
| dead_end | 416 | 16 | mppi | other | 1.8422 | 34.65 |
| mixed | 506 | 6 | mppi | other | 0.2661 | 52.3833 |
