# 9-2 正式成绩：dwa（20 seed × 5 场景）

| 场景 | run 数 | 成功率 | 到达中位 [s] | 路径中位 [m] | 平滑度中位 | 间距中位 [m] | 间距最坏 [m] | 急停合计 |
|---|---|---|---|---|---|---|---|---|
| static_avoid | 20 | 80.0% | 19.2334 | 10.2936 | 17.9122 | 0.1888 | 0.0467 | 0 |
| dynamic_pedestrian | 20 | 40.0% | 17.3583 | 9.7485 | 3.298 | -0.1022 | -0.2999 | 0 |
| narrow_passage | 20 | 70.0% | 21.5333 | 11.3688 | 14.1998 | 0.209 | 0.0437 | 3 |
| dead_end | 20 | 90.0% | 22.8916 | 11.998 | 18.4069 | 0.1048 | 0.0518 | 0 |
| mixed | 20 | 55.0% | 22.0167 | 12.3777 | 19.9698 | 0.0412 | -0.3 | 0 |

套件级 `dynamic_stress`（dynamic_pedestrian + mixed 合并成功率）：**47.5%**

## 异常清单（失败 run，必须逐条说明）
| 场景 | seed | run | 控制器 | failure_class | 最小间距 [m] | 结束时刻 [s] |
|---|---|---|---|---|---|---|
| static_avoid | 105 | 5 | dwa | other | 0.2601 | 23.6167 |
| static_avoid | 112 | 12 | dwa | other | 0.2218 | 24.8333 |
| static_avoid | 118 | 18 | dwa | other | 0.2777 | 22.65 |
| static_avoid | 119 | 19 | dwa | other | 0.1108 | 24.85 |
| dynamic_pedestrian | 201 | 1 | dwa | collision | -0.152 | 17.2 |
| dynamic_pedestrian | 203 | 3 | dwa | collision | -0.2978 | 17.2334 |
| dynamic_pedestrian | 204 | 4 | dwa | other | 0.8918 | 17.75 |
| dynamic_pedestrian | 206 | 6 | dwa | collision | -0.2999 | 17.15 |
| dynamic_pedestrian | 209 | 9 | dwa | collision | -0.2976 | 18.0334 |
| dynamic_pedestrian | 212 | 12 | dwa | collision | -0.0524 | 17.6833 |
| dynamic_pedestrian | 214 | 14 | dwa | collision | -0.2988 | 17.95 |
| dynamic_pedestrian | 215 | 15 | dwa | collision | -0.2969 | 17.0833 |
| dynamic_pedestrian | 216 | 16 | dwa | collision | -0.2965 | 18.55 |
| dynamic_pedestrian | 218 | 18 | dwa | collision | -0.2955 | 17.5167 |
| dynamic_pedestrian | 219 | 19 | dwa | collision | -0.2995 | 18.1667 |
| dynamic_pedestrian | 220 | 20 | dwa | collision | -0.2743 | 18.3 |
| narrow_passage | 301 | 1 | dwa | other | 0.228 | 22.0833 |
| narrow_passage | 303 | 3 | dwa | other | 1.2441 | 10.7667 |
| narrow_passage | 305 | 5 | dwa | other | 0.269 | 22.1833 |
| narrow_passage | 306 | 6 | dwa | other | 3.0718 | 10.7667 |
| narrow_passage | 311 | 11 | dwa | other | 4.5119 | 10.7667 |
| narrow_passage | 316 | 16 | dwa | other | 0.0976 | 25.0333 |
| dead_end | 409 | 9 | dwa | other | 0.1409 | 29.9333 |
| dead_end | 410 | 10 | dwa | other | 0.109 | 20.0333 |
| mixed | 501 | 1 | dwa | collision | -0.2989 | 22.7666 |
| mixed | 502 | 2 | dwa | collision | -0.2158 | 23.6334 |
| mixed | 506 | 6 | dwa | collision | -0.3 | 21.3667 |
| mixed | 508 | 8 | dwa | collision | -0.0142 | 23.7833 |
| mixed | 511 | 11 | dwa | collision | -0.0322 | 21.5833 |
| mixed | 512 | 12 | dwa | collision | -0.2989 | 22.1667 |
| mixed | 513 | 13 | dwa | collision | -0.2983 | 21.3166 |
| mixed | 516 | 16 | dwa | collision | -0.1613 | 24.5833 |
| mixed | 518 | 18 | dwa | collision | -0.0747 | 25.35 |
