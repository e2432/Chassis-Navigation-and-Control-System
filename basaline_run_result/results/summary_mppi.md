# 9-2 正式成绩：mppi（20 seed × 5 场景）

| 场景 | run 数 | 成功率 | 到达中位 [s] | 路径中位 [m] | 平滑度中位 | 间距中位 [m] | 间距最坏 [m] | 急停合计 |
|---|---|---|---|---|---|---|---|---|
| static_avoid | 20 | 95.0% | 21.6833 | 10.3871 | 7.3755 | 0.2626 | 0.1113 | 0 |
| dynamic_pedestrian | 20 | 60.0% | 19.65 | 9.8101 | 5.5189 | 0.1542 | -0.3 | 0 |
| narrow_passage | 20 | 85.0% | 23.6833 | 11.8446 | 7.2203 | 0.2273 | 0.191 | 2 |
| dead_end | 20 | 90.0% | 24.7 | 12.3387 | 7.9294 | 0.2121 | 0.196 | 0 |
| mixed | 20 | 55.0% | 25.2 | 12.4665 | 8.6273 | 0.1141 | -0.2994 | 0 |

套件级 `dynamic_stress`（dynamic_pedestrian + mixed 合并成功率）：**57.5%**

## 异常清单（失败 run，必须逐条说明）
| 场景 | seed | run | 控制器 | failure_class | 最小间距 [m] | 结束时刻 [s] |
|---|---|---|---|---|---|---|
| static_avoid | 105 | 5 | mppi | other | 0.3705 | 26.2 |
| dynamic_pedestrian | 203 | 3 | mppi | collision | -0.2529 | 19.6 |
| dynamic_pedestrian | 206 | 6 | mppi | collision | -0.2969 | 19.5167 |
| dynamic_pedestrian | 207 | 7 | mppi | collision | -0.2986 | 20.45 |
| dynamic_pedestrian | 212 | 12 | mppi | collision | -0.1487 | 19.05 |
| dynamic_pedestrian | 214 | 14 | mppi | collision | -0.0888 | 20.45 |
| dynamic_pedestrian | 218 | 18 | mppi | collision | -0.0842 | 19.9833 |
| dynamic_pedestrian | 219 | 19 | mppi | collision | -0.3 | 20.6333 |
| dynamic_pedestrian | 220 | 20 | mppi | collision | -0.129 | 19.6167 |
| narrow_passage | 306 | 6 | mppi | other | 3.0718 | 10.7667 |
| narrow_passage | 311 | 11 | mppi | other | 4.5119 | 10.75 |
| narrow_passage | 316 | 16 | mppi | other | 0.2127 | 30.6333 |
| dead_end | 409 | 9 | mppi | other | 0.1994 | 42.0667 |
| dead_end | 416 | 16 | mppi | other | 0.2692 | 20.2 |
| mixed | 501 | 1 | mppi | collision | -0.2976 | 25.7167 |
| mixed | 502 | 2 | mppi | collision | -0.1865 | 27.4166 |
| mixed | 506 | 6 | mppi | other | 0.0513 | 23.5667 |
| mixed | 507 | 7 | mppi | collision | -0.1054 | 25.8666 |
| mixed | 508 | 8 | mppi | collision | -0.2982 | 24.6 |
| mixed | 511 | 11 | mppi | collision | -0.2994 | 22.9333 |
| mixed | 513 | 13 | mppi | collision | -0.26 | 22.5 |
| mixed | 515 | 15 | mppi | collision | -0.298 | 22.9666 |
| mixed | 518 | 18 | mppi | collision | -0.1569 | 26.6333 |
