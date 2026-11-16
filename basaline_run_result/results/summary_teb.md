# 9-2 正式成绩：teb（20 seed × 5 场景）

| 场景 | run 数 | 成功率 | 到达中位 [s] | 路径中位 [m] | 平滑度中位 | 间距中位 [m] | 间距最坏 [m] | 急停合计 |
|---|---|---|---|---|---|---|---|---|
| static_avoid | 20 | 80.0% | 18.2583 | 10.6248 | 16.108 | 0.3097 | 0.2769 | 0 |
| dynamic_pedestrian | 20 | 40.0% | 16.6416 | 10.0201 | 13.1575 | -0.1985 | -0.2997 | 0 |
| narrow_passage | 20 | 85.0% | 20.6667 | 12.1642 | 13.9406 | 0.3417 | 0.2654 | 3 |
| dead_end | 20 | 90.0% | 20.8584 | 12.2802 | 14.8031 | 0.3039 | 0.2231 | 0 |
| mixed | 20 | 55.0% | 21.2667 | 12.6125 | 17.582 | 0.0389 | -0.2993 | 0 |

套件级 `dynamic_stress`（dynamic_pedestrian + mixed 合并成功率）：**47.5%**

## 异常清单（失败 run，必须逐条说明）
| 场景 | seed | run | 控制器 | failure_class | 最小间距 [m] | 结束时刻 [s] |
|---|---|---|---|---|---|---|
| static_avoid | 107 | 7 | teb | other | 0.3174 | 24.1333 |
| static_avoid | 108 | 8 | teb | other | 0.355 | 16.6167 |
| static_avoid | 118 | 18 | teb | other | 0.302 | 20.2833 |
| static_avoid | 119 | 19 | teb | other | 0.3037 | 28.1833 |
| dynamic_pedestrian | 201 | 1 | teb | collision | -0.043 | 16.65 |
| dynamic_pedestrian | 203 | 3 | teb | collision | -0.2981 | 16.6167 |
| dynamic_pedestrian | 206 | 6 | teb | collision | -0.2983 | 16.4833 |
| dynamic_pedestrian | 207 | 7 | teb | collision | -0.2238 | 17.55 |
| dynamic_pedestrian | 209 | 9 | teb | collision | -0.2997 | 17.4166 |
| dynamic_pedestrian | 212 | 12 | teb | collision | -0.2034 | 17.1166 |
| dynamic_pedestrian | 214 | 14 | teb | collision | -0.2996 | 17.3667 |
| dynamic_pedestrian | 215 | 15 | teb | collision | -0.2967 | 16.5333 |
| dynamic_pedestrian | 216 | 16 | teb | collision | -0.2988 | 17.8334 |
| dynamic_pedestrian | 218 | 18 | teb | collision | -0.297 | 16.9333 |
| dynamic_pedestrian | 219 | 19 | teb | collision | -0.2978 | 17.45 |
| dynamic_pedestrian | 220 | 20 | teb | collision | -0.1935 | 17.7 |
| narrow_passage | 306 | 6 | teb | other | 3.0718 | 10.7333 |
| narrow_passage | 311 | 11 | teb | other | 4.5119 | 10.7833 |
| narrow_passage | 314 | 14 | teb | other | 1.9312 | 10.7833 |
| dead_end | 409 | 9 | teb | other | 0.2977 | 30.3167 |
| dead_end | 416 | 16 | teb | other | 0.3338 | 17.1333 |
| mixed | 501 | 1 | teb | collision | -0.295 | 21.5333 |
| mixed | 502 | 2 | teb | collision | -0.298 | 23.8667 |
| mixed | 506 | 6 | teb | collision | -0.2986 | 21.3333 |
| mixed | 508 | 8 | teb | collision | -0.0654 | 21.1667 |
| mixed | 512 | 12 | teb | collision | -0.2969 | 21.2333 |
| mixed | 513 | 13 | teb | collision | -0.2993 | 19.8167 |
| mixed | 515 | 15 | teb | collision | -0.1482 | 20.05 |
| mixed | 516 | 16 | teb | collision | -0.2989 | 22.45 |
| mixed | 518 | 18 | teb | collision | -0.2563 | 22.9333 |
