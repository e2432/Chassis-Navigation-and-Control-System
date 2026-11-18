# 9-1 / 9-2 / 9-3 工程侧交付包（2026-11-18，道口栏杆版）

本包只含**主代码 + 配置 + 评分清单 + 结果文档与汇总**，不含构建产物、原始
`trajectory.npz`、Isaac 场景 `.usda/.pgm`、任何本机绝对路径。

## 0. 这是什么

- **9-1**：基线接入与参数定稿（共享执行层、三基线参数、冻结 sha256）。
- **9-2**：正式成绩入库（3 基线 × 5 场景 × 20 种子 = 300 run）。
- **9-3**：行人动态障碍（道口栏杆机制）+ 时限冻结
  （`T_s = max(T_min,s , 1.5·max_j t̃(s,j))`，只用成功 run）。

## 1. 目录

```
README.md                     本文件
MANIFEST.txt                  逐文件 sha256（上传前核对用）
code/robot_bringup/           共享执行层：Nav2 launch、代价地图/安全层配置、四份控制器参数、
                              道口栏杆层 ped_corridor_layer.cpp（+ plugins.xml / CMakeLists / package.xml）
code/robot_benchmark/         场景生成、判分（judge）、采集（nav2_driver/runner）、汇总（summarize）
code/robot_sim/               Isaac Sim 桥、真值 odom、map→odom 静态 TF、场景导出
code/scripts/                 批量采集（run_workset / run_300）、口径切换（run_mixed）、
                              观测工具（demo_gui / dev_* / cleanup_sim）、9-3 时限冻结
data/instances.yaml           100 条冻结实例（含 9-3 时限 T_s）
data/templates/               5 个场景模板（重建 instances.yaml 用）
results/                      冻结快照、正式成绩、执行台账、9-3 行人动态障碍与时限冻结说明、
                              各基线 summary.md（含逐条失败 run 异常清单）
```

## 2. 正式成绩（5 场景 × 20 seed × 3 基线 = 300 run）

| 场景 | DWA | MPPI | TEB |
|---|---|---|---|
| static_avoid | 18/20 = 90% | 19/20 = 95% | 18/20 = 90% |
| dynamic_pedestrian | 15/20 = 75% | 20/20 = 100% | 20/20 = 100% |
| narrow_passage | 15/20 = 75% | 15/20 = 75% | 16/20 = 80% |
| dead_end | 17/20 = 85% | 17/20 = 85% | 17/20 = 85% |
| mixed | 18/20 = 90% | 19/20 = 95% | 14/20 = 70% |
| **合计** | 83/100 = 83.0% | 90/100 = 90.0% | 85/100 = 85.0% |

全体 **258/300 = 86.0%**。

套件级 `dynamic_stress`（dynamic_pedestrian + mixed）：DWA 33/40 = 82.5%；MPPI 39/40 = 97.5%；TEB 34/40 = 85.0%。

## 3. 指标明细（各场景 20 run 中位 / 最坏）

| 场景 | 基线 | 到达 [s] | 路径 [m] | 平滑度 | 间距中位 [m] | 间距最坏 [m] | 急停合计 |
|---|---|---|---|---|---|---|---|
| static_avoid | dwa | 19.38 | 10.27 | 19.09 | 0.167 | 0.069 | 0 |
| static_avoid | mppi | 21.75 | 10.53 | 7.40 | 0.236 | 0.094 | 0 |
| static_avoid | teb | 18.25 | 10.85 | 16.55 | 0.278 | 0.210 | 0 |
| dynamic_pedestrian | dwa | 28.10 | 9.75 | 18.17 | 0.803 | 0.698 | 0 |
| dynamic_pedestrian | mppi | 30.55 | 9.89 | 6.55 | 0.987 | 0.870 | 0 |
| dynamic_pedestrian | teb | 29.37 | 15.73 | 45.49 | 0.973 | 0.622 | 0 |
| narrow_passage | dwa | 21.65 | 11.69 | 16.04 | 0.207 | 0.040 | 5 |
| narrow_passage | mppi | 23.70 | 11.88 | 6.94 | 0.225 | 0.192 | 6 |
| narrow_passage | teb | 19.92 | 11.79 | 12.23 | 0.341 | 0.263 | 5 |
| dead_end | dwa | 22.53 | 12.12 | 22.71 | 0.114 | 0.045 | 3 |
| dead_end | mppi | 24.92 | 12.36 | 7.92 | 0.216 | 0.187 | 3 |
| dead_end | teb | 20.72 | 12.30 | 16.46 | 0.301 | 0.221 | 5 |
| mixed | dwa | 33.77 | 12.37 | 33.66 | 0.125 | 0.046 | 0 |
| mixed | mppi | 34.60 | 12.49 | 10.03 | 0.232 | 0.083 | 0 |
| mixed | teb | 32.90 | 18.09 | 49.93 | 0.204 | -0.028 | 5 |

## 4. 怎么复现

```bash
# 1) 环境
source nav_ws_core/scripts/env.sh                    # 系统 ROS 2 Humble
export ISAAC_PYTHON=$HOME/miniconda3/envs/env_isaaclab/bin/python3
source ~/teb_prebuilt/env.sh && source ~/teb_build_ws/install/setup.bash   # TEB
bash nav_ws_core/scripts/build.sh                    # 含自研代价地图层 ped_corridor_layer

# 2) 一次性 300 run（每场景生成 .usda + .pgm/.yaml，三基线同栈跑完；可续跑）
bash nav_ws_core/scripts/run_300.sh 9-2_official_v3

# 3) 只想看 mixed（走廊口径 / 雷达口径 A/B）
bash nav_ws_core/scripts/run_mixed.sh mixed_v3
bash nav_ws_core/scripts/run_mixed.sh mixed_v3_lidar nav2_common_lidar.yaml

# 4) 在 Isaac GUI 里逐基线看动态避障（一条命令一个基线）
bash nav_ws_core/scripts/demo_gui.sh mixed_01 dwa     # 换成 mppi / teb 验证另外两个

# 5) 9-3 时限复核（dry-run，不会改 instances.yaml）
python3 nav_ws_core/scripts/freeze_time_limits.py --workset out/9-2_official_v3
```

## 5. 口径提醒（写论文/汇报时照这四条说）

- 速度契约：平面速度 `√(vx²+vy²) ≤ 0.6 m/s`、`|wz| ≤ 1.5 rad/s`、加速度 1.0/1.0/2.0 m/s²、
  控制 20 Hz、到达判据 0.25 m / 0.25 rad（控制器**内部**收到 0.18 m / 0.15 rad，见 9-1 §2.1）。
- 失败分类：`collision`（与行人最小间距 ≤ 0）、`timeout`（超 `time_limit_s`）、
  `other`（其余，主要为卡死/局部极小）。
- 行人：契约给定匀速直线轨迹（`x=x_cross`、`y=dir·speed·(t−t_meet)`）。v3 把行人的
  **横穿走廊**当作"道口栏杆"注入局部代价地图（`/ped/corridor` → `PedCorridorLayer`），
  三个基线共用同一份信息；判分**不看**代价地图，只看轨迹与契约几何。
- 每份 `summary.md` 都带逐条失败 run 的异常清单，不得静默丢弃失败。
