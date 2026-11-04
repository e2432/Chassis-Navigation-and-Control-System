# scenarios/

五类场景模板 + 生成器产出（路线书 **E14** 实装）。

## 文件

| 文件 | 内容 |
|---|---|
| `static_avoid.yaml` | 静态绕行模板：直线 10 m、4~6 个圆障碍（r 0.30~0.60 m）、横向偏置 ±1.5 m、时限 60 s |
| `dynamic_pedestrian.yaml` | 动态行人模板：1~2 个行人、匀速 0.6~1.0 m/s 垂直横穿、交叉点 ±2 m、相遇时刻 ±2 s |
| `narrow_passage.yaml` | 窄通道模板：通道长 3±0.5 m、净宽 1.3~1.5 m（硬下限 1.3）、单侧余量 ≥0.30 m |
| `dead_end.yaml` | 死角模板：U 形盲袋、袋口净宽 1.4±0.1 m、袋深 2.0 m、目标在袋底 |
| `mixed.yaml` | 混合模板：直线 12±1 m、静态障碍 4~6 个 + 行人 2 个、时限 75 s |
| `instances/<场景>_instances.yaml` | **已冻结的 20 个实例**（每个实例保存全部取值，不只保存种子） |
| `test_list.yaml` | 所有控制器共用的固定测试实例清单（5×20=100） |

## 生成 / 校验

```bash
cd ${NAV_WS} && source tools/engineering/e09_sys_env.sh

# 重新生成（确定性：同种子必然得到同样结果）
python3 -m robot_benchmark.gen_scenarios --out scenarios --runs 20

# 只校验（CI / 合并前）：
#   - 实例数与 runs 一致
#   - 用模板重新生成一遍必须与文件逐字段相同（确定性）
#   - 每个实例通过有效性检查
python3 -m robot_benchmark.gen_scenarios --out scenarios --check
```

装了工作空间后也可以：`ros2 run robot_benchmark benchmark_gen_scenarios --check`。

## 实例字段（E14 要求"每场景至少保存这些"）

```yaml
scenario_id: static_avoid_01      # <场景键>_<run 序 01..20>
scenario: static_avoid
seed: 101                         # = 场景序(1~5)×100 + run 序  ← 与算法侧种子表一致
asset_version: "2026-11-03"       # 场景 USD / 障碍几何版本
map_version: "e05_min_scene@2026-11-01"
robot_start: {x, y, yaw}
goal: {x, y, yaw}                 # 航向 = 行进方向（不随机化）
obstacles: [{type: circle|box, x, y, r|w,h}]   # 真值几何，给裁判算间距/碰撞
dynamic: [{type: pedestrian, radius, start, velocity, start_time_s, stop_at_y}]
timeout_s: 60                     # 场景级时限（60/60/60/60/75）
goal_tolerance: {pos_m: 0.25, yaw_rad: 0.25}
solvable: true                    # 是否应当可解（dead_end 也标 recovery_required）
passage: {net_width_m, length_m, margin_per_side_m, min_required_margin_m}   # 仅窄通道
dead_end: {mouth_width_m, depth_m}                                          # 仅死角
validity: {connectivity_checked, robot_radius_m, grid_m, resamples}
```

## 有效性检查（生成时强制，不通过就重抽样，最多 200 次）

1. 机器人起点 / 目标点**不在障碍内部**（外扩 0.10 m 判定）；
2. 用膨胀 0.35 m（底盘外接圆）的 0.10 m 栅格 + **A\* 连通性检查**，保证存在可通行路径
   ——对应 E14「不生成必然碰撞却标为正常避障的测试」；
3. 窄通道：净宽 ≥1.30 m 且单侧余量 ≥0.30 m（指标 5 阈值）；
4. `dead_end` 必须标注 `recovery_required: true`；
5. 行人速度落在 0.6~1.0 m/s、启动时刻非负。

## 在 Isaac Sim 里看这五个场景

场景资产由 `src/robot_sim/robot_sim/scripts/scene_from_scenario.py` 按实例搭出来
（地面 + 机器人 + 静态障碍 + 行人代理 + 起点/目标标记 + 灯光，相机自动摆到能看全的位置）。

```bash
cd ${NAV_WS} && source tools/engineering/e09_sys_env.sh

# ① 开 Isaac 窗口看某一个实例（推荐，能自己转视角）
${ISAAC_PYTHON} \
  src/robot_sim/robot_sim/scripts/scene_from_scenario.py --scenario static_avoid --run 1 --gui

# ② 五类场景自动巡览：每 12 秒换下一个
... scene_from_scenario.py --all --cycle 12 --gui

# ③ 不开窗口，把五个场景各导出一份 USD（之后可用 File > Open 打开）
... scene_from_scenario.py --all --save --headless
#    → assets/scenes/scenarios/{static_avoid,dynamic_pedestrian,narrow_passage,dead_end,mixed}_01.usd

# ④ 富化版（障碍更多，仍保证可达）——导出的 5 份 USD 就是富化版
... scene_from_scenario.py --all --save --rich --headless

# 换实例：--run 2..20
```

`--rich` 做的事（确定性的，同一实例每次都一样）：按 E14 每类场景的要素补障碍/行人，
**每加一组都会重新跑可达性校验**，不可达就回退，所以"更复杂但一定能走通"。

| 场景 | 原始实例 | 富化后（已导出的 USD） |
|---|---|---|
| `static_avoid` | 4 个障碍 | **17 个圆柱障碍**（两侧路边 6 + 交错绕桩 4 + 终点前三连挡 3 + 原 4） |
| `dynamic_pedestrian` | 2 个行人 | **4 个行人（双向、启动时刻错开）+ 6 个路边/绕桩障碍** |
| `narrow_passage` | 通道 + 2 墙体 | **8 个障碍**（入口前绕桩 4 + 出口后 2 + 原通道墙体 2） |
| `dead_end` | U 形袋 + 3 墙 | **10 个**（袋内两侧 4 + 袋口外 2 + 接近段 4 + 原 3 墙） |
| `mixed` | 4 障碍 + 2 行人 | **16 个障碍 + 3 个行人**（含一段挡墙，逼出绕行/穿缝） |

> 评分用的 `instances/*.yaml` **不受富化影响**，仍与算法侧 `5场景需求清单.md` 的数量一致
> （如静态障碍 4~6 个）；富化只用于可视化查看与压力观察。
窗口里能对照检查的东西：
| 场景 | 该看到什么 |
|---|---|
| `static_avoid` | 起点蓝环 → 4~6 个棕色圆柱障碍 → 终点绿环+绿柱，有可绕行空隙 |
| `dynamic_pedestrian` | 1~2 个红色行人圆柱沿 y 轴匀速横穿（按实例里的启动时刻开始动） |
| `narrow_passage` | 两段灰色墙体夹出一条 1.3~1.5 m 的通道（净宽越接近 1.3 越该"贴边"） |
| `dead_end` | U 形三面墙盲袋（袋口 1.4 m、深 2.0 m），目标在袋底 |
| `mixed` | 圆柱障碍 + 2 个行人同时出现，直线 12±1 m |
 > 注意：机器人只是摆着（不接 ROS、不驱动）；行人按真值轨迹动，供目视核对"信息是否公平"。

## 已导出的场景 USD（`assets/scenes/scenarios/`）

| 文件 | 内容（独立核对，见 `results/w3/scenarios_usd_check.txt`） |
|---|---|
| `static_avoid_01.usd` | 圆柱障碍 17，碰撞体 18，含机器人 + 目标标记 |
| `dynamic_pedestrian_01.usd` | 圆柱障碍 6 + 行人 4，碰撞体 11 |
| `narrow_passage_01.usd` | 圆柱障碍 6 + 墙体 2（净宽 1.46 m），碰撞体 9 |
| `dead_end_01.usd` | 圆柱障碍 7 + 墙体 3（U 形袋 1.40×2.00 m），碰撞体 11 |
| `mixed_01.usd` | 圆柱障碍 15 + 墙体 1 + 行人 3，碰撞体 20 |

打开方式：Isaac Sim `File > Open` 选文件 → 点 Play（行人会动）；或直接跑上面的 ①/②/④ 命令。

## 待补（不夸大）

- 场景 USD 目前是**运行时按实例搭建**（`scene_from_scenario.py`），并支持 `--save` 导出成
  `assets/scenes/scenarios/*_01.usd`；还没有"一套资产 + 参数化实例"的正式资产库（E15 前补）。
- `benchmark.run(scenario)` 还只在契约层（`contract.py`），执行入口 E15 实装。
