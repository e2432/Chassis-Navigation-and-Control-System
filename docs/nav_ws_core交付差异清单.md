# nav_ws_core 交付差异清单（算法侧核对）

> 2026-11-06，算法侧对工程侧 `nav_ws_core/` 交付（五类场景 + 100 冻结实例）的浏览核对结果。
> 核对基准：docs/接口契约.md v1.2、docs/5场景需求清单.md（W3 定稿）、方案书 W4 决策。
> 核对方式：静态阅读（文件格式 / 生成器与播放代码走查），**未做运行时验证**。
> 用途：交工程组逐项处理；处理状态请在各条目后追加标注。

---

## 结论摘要

5 个 scenario 键、100 冻结实例、种子规则、真值 TF 链、话题命名均与契约一致，交付主体成立。
差异集中在四块：**①实例表字段与契约 v1.2 schema 不对齐**（挡住"两侧直接加载同一份表"）、
**②评测记录/指标/基线整段缺失**、**③两场景起终点长度随机化超出需求清单授权**、**④行人速度符号疑似反向**（动态行人实际不动）。

优先级：**P0 = ④、①；P1 = ②；P2 = ③；P3 = 已知边界**。

---

## 2.0.0 重冻结交付收口（2026-11-06，随契约 v1.4）

工程组 **2.0.0 重冻结交付**（`generator_version: 2.0.0`、`asset_version: '2026-11-06'`，单表
`docs/scenarios/instances.yaml`）整体替换旧冻结表，本清单 P0~P3 状态随之收口：

| 条目 | 状态 | 说明 |
|---|---|---|
| P0-1 行人符号 | **关闭（被替代）** | 行人模型整体替换为常速横穿 `y = dir·speed·(t−t_meet)`（`dir` 为权威方向字段、无 `vy` 符号载体），`vy` 外科反号处方作废 |
| P0-2 字段不对齐 | **关闭（达成）** | 2.0.0 回归规范名十字段，`scenario_index`/`path_length_m` 转正；契约 v1.4 收编（v1.3 字段收编整包作废） |
| P1 评测记录/基线缺失 | **开放** | 2.0.0 仍无评分/落盘/基线；三件套已交付工程组待实装 |
| P2 起终点抽样越权 | **关闭（追认扩写）** | 沿「认领不回炉」先例，2.0.0 新随机化维度整体追认（窄通道入口位置/横向偏移/壁厚/结构 kind/通道长 2.6~4.0、死角袋深 1.9~3.2/四向袋口/位置/壁厚、`world_poses` 世界朝向摆放），需求清单授权行按 2.0.0 维度重写 |
| P3 已知边界 | 保留 | 无变化 |
| （新）表落位修正 | **已处理** | 交付误落 `nav_ws_core/scenarios/instances.yaml`（readme 与生成器 `default_manifest_path()` 均认 `docs/scenarios/instances.yaml`）；算法侧已 `git mv` 至契约路径、**零内容改动**，知会工程组后续上传按契约路径落位 |

随契约 v1.4 一并认领的语义变更：**dead_end 目标在陷阱外、任务允许规划绕行**（不宣称所有控制器必须入库倒车；`failure_class: dead_end_reverse` 保留给入袋被困/倒车失败运行），`recovery_required` 字段随 v1.3 字段收编作废。

---

## P0-1 行人速度符号疑似反向 → 动态行人瞬移后冻结

> **已关闭（2026-11-06 2.0.0 重冻结）**：行人轨迹模型整体替换为常速横穿 `y = dir·speed·(t−t_meet)`（无 `vy` 符号载体），下文「`vy` 外科反号」处方**作废、不再执行**；语义收编见契约 v1.4。

**位置**：
- 生成器：`nav_ws_core/src/robot_benchmark/robot_benchmark/scenario_gen.py:148`（`_gen_pedestrian`）
- 播放：`nav_ws_core/src/robot_sim/robot_sim/scripts/scene_from_scenario.py:298`（`animate`）
- 数据例：`nav_ws_core/scenarios/instances/dynamic_pedestrian_instances.yaml`（`dynamic_pedestrian_01` 首个行人）

**现状**：实例中 `start.y = -3.0, velocity.vy = -0.768, stop_at_y = +3.0`。
按 `y(t) = start.y + vy·dt` 行人向 y 负方向越走越远，永远到不了 `stop_at_y`；
而 `animate()` 的 clamp（`vy<0 → y = max(y, stop_at_y)`）会在**第一帧**把行人从 y=−3 瞬移到 y=+3，之后每帧都被 clamp 冻结在 +3。

**根因**：`_gen_pedestrian` 中 `y_start = -side·span`、`vy = -side·v`、`stop_at_y = +side·span`——
`vy` 方向与 `start → stop_at_y` 的行进方向相反（side=+1 时从 −3 应向 +3 走，vy 应为正）。

**影响**：Isaac 预览/回放里 `dynamic_pedestrian`、`mixed` 的行人实际不动（瞬移后静止），
动态应变（指标 7）主场场景退化成静态场景。

**建议动作**（2026-11-06 grill 定案，P0-1 表口径）：冻结表逐行人 **`vy := −vy` 外科反号**（几何一概不动、不重新生成），生成器符号同修保可复算，
补一条单测：行人 y(t) 从 start 单调走到 stop_at_y、恰在 t = start_time_s + |y_start|/v 过 y=0。
方向定义见契约 v1.3：**方向 = start → stop_at_y 推导、`vy` 仅为速率载体**——消费方不信任符号列，修复仅约束表值最终形态（修复落地前消费方按推导口径即可正确工作）。

**复核命令**：

```bash
"$ISAAC_PYTHON" src/robot_sim/robot_sim/scripts/scene_from_scenario.py \
  --scenario dynamic_pedestrian --run 1 --gui --duration 20
```

预期现象（修复前）：行人第一帧跳到对侧后静止不动。

---

## P0-2 实例表字段与契约 v1.2 schema 不对齐

> **已关闭（2026-11-06 2.0.0 重冻结）**：2.0.0 表回归规范名十字段（`run`/`time_limit_s`/`pedestrians`/`walls` 等，`scenario_index`/`path_length_m` 转正），对齐达成；下文对照保留为历史记录。

**位置**：`nav_ws_core/scenarios/instances/*.yaml`（5 分文件）
vs `docs/接口契约.md`「场景实例表（schema v1）」

**现状 → 应然对照**：

| 交付字段 | 契约 v1.2 字段 | 处理 |
|---|---|---|
| `scenario_id: "static_avoid_01"` | `scenario: 键名` + `scenario_index: 1` + `run: 1` | 拆为两字段（scenario 键本身一致 ✓） |
| `run_index` | `run` | 改名 |
| `seed` | `seed` | 一致 ✓ |
| `timeout_s` | `time_limit_s` | 改名（回填语义见契约：W4 基线后按校准公式回填） |
| （无） | `path_length_m` | 需补（起终点直线长度） |
| `robot_start {x,y,yaw}` | （无，约定固定 (0,0,0)） | 建议保留为扩展字段，不冲突 |
| `goal {x,y,yaw}` | `goal {x,y,yaw}` | 一致 ✓ |
| `obstacles: [{type: circle, x, y, r}]` | `obstacles: [{x, y, r}]` | 圆障碍等价，去掉 `type` |
| `obstacles: [{type: box, …}]`、`passage`、`dead_end` | `walls: [{x0, y0, x1, y1}]` | 轴对齐矩形需换算拆出 |
| `dynamic: [{start, velocity, start_time_s, stop_at_y}]` | `pedestrians: [{x_cross, speed, t_meet, dir, r}]` | 语义换算见下 |
| 路径 `scenarios/instances/<scenario>_instances.yaml`（5 分文件） | `docs/scenarios/instances.yaml`（单表） | 定其一：出合并导出，或契约升版认 5 分文件 |
| 多出：`asset_version` / `map_version` / `goal_tolerance` / `solvable` / `validity` | 无 | 有价值，建议契约升版收编 |

**行人语义换算**（字段名对齐后可机械转换，前提是 P0-1 符号先修好）：

```
x_cross = start.x
speed   = |vy|
dir     = sign(vy)            # +1 向 +y、−1 向 −y（契约口径）
t_meet  = start_time_s + |start.y| / speed
r       = radius
```

注意：契约轨迹是**全程匀速直线、不折返不停止**（无启动延迟、无到边停住）；
交付的 `start_time_s`/`stop_at_y` 是额外语义。二选一：
(a) 换算到契约语义、舍弃启动/停住语义；(b) 契约升 v1.3 认领这两个字段（走变更流程双方确认）。

> **已决（2026-11-06，算法侧拍板）**：走 (b)——`start_time_s`/`stop_at_y` 由契约 v1.3 收编，
> 语义按交付现状定义（延迟启动 + 行至 stop_at_y 停住），换算公式中的 `t_meet`/`dir` 一并入约。

**建议动作**：随 v1.3 完成字段 re-export / 转换脚本，
保证 `docs/scenarios/instances.yaml` 与 5 分文件数值同源。在对齐完成前，算法侧按需写临时适配层消费几何数值。

---

## P1 评测记录 / 指标 / 基线整段缺失

> **仍开放（2026-11-06 2.0.0 收口确认）**：2.0.0 交付仍无评分 / 落盘 / 基线（readme 自证仅最小桥接演示），下文各条待工程组实装。

**位置**：整个 `nav_ws_core/`（readme「当前边界」自述"不提供完整基准评分功能"）

**缺**（契约「评测记录 schema（v1）」与「基准套件 API」要求的套件侧载体）：

- `runs/<scenario>/<seed>/run.json` + `trajectory.npz` 落盘（run.json 各字段：ablation 快照 / metrics / qp_timing / failure_class）
- 指标键：`success` / `time_to_goal_s` / `path_length_m` / `smoothness` / `min_clearance_m` / `estop_count`，套件级 `dynamic_stress`
- `failure_class` 枚举判定（collision / timeout / solver_fail / dead_end_reverse / narrow_oscillation / dynamic_stall / other）
- `benchmark.run_suite()`（或等价 API）与基线控制器 DWA / TEB / MPPI

**影响**：M3/W6 正式对比（同环境 `benchmark.run()`、7 指标对照）前必须补齐；
在补齐前，算法侧原型自行按同 schema 落盘（契约允许各自输出根），不阻塞离线开发。

**建议动作**：按契约 v1.2「评测记录 schema」逐字段实现；键名勿另立（v1.1 对齐纪要已定"指标/场景键以契约为准"）。

（2026-11-06 更新：P1 三件套已交付工程组，等待其实现 PR，不另挂验收钩子。）

---

## P2 static_avoid / dynamic_pedestrian 起终点长度随机化超出需求清单授权

> **已关闭（2026-11-06 2.0.0 重冻结）**：沿下文「方向 1 认领」先例，2.0.0 新随机化维度一并追认（见「2.0.0 重冻结交付收口」），5场景需求清单授权行已按 2.0.0 维度扩写。

**位置**：`nav_ws_core/scenarios/static_avoid.yaml` 与 `dynamic_pedestrian.yaml`（均 `corridor_length_m: [9.5, 10.5]`），
波及两场景全部 40 个实例（goal.x = 10.081、9.648、…）。其余场景抽样均在授权内（mixed 11~13、
narrow_passage 通道长 3±0.5/净宽 1.3~1.5、dead_end 袋口 1.3~1.5 均为清单明文授权）。

**现状**：起终点直线长度逐实例抽样 9.5~10.5 m。

**应然**：5场景需求清单 static_avoid / dynamic_pedestrian「随机化范围」均仅授权障碍/行人类抽样；
「环境要素」定起终点直线 10 m。契约示例亦为 `path_length_m: 10.0` 固定（仅 mixed 为 11~13 抽样值）。

**两个处理方向（择一）**：

1. **认领**：需求清单/契约升版，把起终点长度 9.5~10.5 m 抽样补进授权范围（表已冻结、数值即事实，成本最低）；
2. **回炉**：按原授权范围重新生成两场景 40 实例并重新冻结（`--check` + 全量校验重跑，双方重新确认表）。

注：契约 v1.2 已定"表内数值即最终事实，重建只认表、不认抽样"——方向 2 属重建，需明确知会双方。
另注：同种子重生成会改变 RNG 流，40 实例的整套几何（障碍/行人参数）都会变，不是只改长度。

> **已决（2026-11-06，算法侧拍板）**：**方向 1 认领**，随契约 v1.3（收编行人语义的同一次升版）一并追认：
> 变更记录明写"追认 2026-11-03 冻结生成（asset_version）的路径长度抽样"；
> 5场景需求清单 static_avoid / dynamic_pedestrian「随机化范围」行补上"起终点直线 9.5~10.5 m"，
> 使生成器就地转为合规、此后重生成不出界。实例表不重建、不改数值。
> 契约/清单修改本体走 feature 分支 + PR，双方确认后 v1.3 生效。

---

## P3 已知边界（readme 已声明，列此备忘，无待办或仅确认项）

| 项 | 说明 | 建议 |
|---|---|---|
| best_effort QoS 运行验证未做 | readme：仍需原 Linux 环境复测 | 工程侧排期时知会算法侧一起盯 `/lidar/points`、`/imu` |
| 雷达仅输出 x/y/z，近场低矮障碍覆盖不足 | 影响 VoxelLayer 近场感知 | 记入评测局限；若补强度/线号走契约话题字段变更 |
| 基准锁 kinematic；IMU 不可用于惯性性能评估 | physics 平移偏差 | 与契约"闭环保真度维持完美运动学"一致，维持 |
| 空视觉 prim 引用警告 | 模型警告，无功能影响 | 择机清理 |
| `chassis.yaml` 命名 | 内容在 `robot_description/config/robot_geometry.yaml`（footprint 多边形/wheels/sensors 齐） | 算法侧按此路径消费即可，或契约注明别名，不强求改名 |

---

## 已具备确认（核对通过项，无需动作）

- 5 个 scenario 键命名与契约一致：`static_avoid / dynamic_pedestrian / narrow_passage / dead_end / mixed`
- 100 冻结实例：5 × 20，种子 = 场景序 × 100 + run 序（101…520），抽验 4 个实例文件全部正确
- `test_list.yaml` 五类 × 20 id 清单齐全
- 生成器 + `gen_scenarios --check` 校验 CLI + 12 项单测（readme 自证通过）
- `scene_from_scenario.py` 按实例搭建 Isaac 场景（含 box/cylinder 几何、起终点标记）
- 真值链路：`truth_odom_node` / `map_odom_static_node` / `truth_tf.launch.py`，TF 链 `map → odom → base_footprint → base_link → 传感器` 与契约一致
- 话题命名：`/lidar/points`、`/imu`、`/odom` 与契约一致
- 底盘描述：URDF/xacro + STL + `robot_geometry.yaml`（含契约 chassis.yaml schema 所需的 footprint 多边形、wheels、sensors）

---

## 附：契约 v1.3 待办摘要（已随 v1.3 落地，2026-11-06）

1. **行人语义收编**（P0-2 已决）✅：`dynamic` 字段（含 `start_time_s` 延迟启动、`stop_at_y` 行至对侧停住）
   分段语义入约，`t_meet`/`dir` 派生口径一并写明——落 docs/接口契约.md「场景实例表（schema v1.1）」。
2. **路径长度抽样追认**（P2 已决）✅：static_avoid / dynamic_pedestrian 起终点直线 9.5~10.5 m 抽样
   追认 2026-11-03 冻结生成；5场景需求清单「随机化范围」已补授权行。
3. **字段名对齐**（P0-2 随行）✅：**收编交付命名**（`run_index`/`timeout_s`/`dynamic`/`obstacles(type)`/
   `passage`/`dead_end` 等，v1.2 旧名作废、映射表入约），补 `path_length_m`/`scenario_index`（导出补写），
   路径定格 `nav_ws_core/scenarios/instances/<scenario>_instances.yaml` 5 分文件。

落地范围为文档侧（含 W4 工作手册 §1.3「实例加载器需求」——加载器**实现**另任务）；
P0-1 修复、生成器/冻结表修改仍归工程组 PR。文档改完后按契约「变更规则」走 feature 分支 + PR 双方确认生效。

---

## 变更记录

| 日期 | 变更 | 作者 |
|---|---|---|
| 2026-11-06 | 初版（浏览核对，未含运行时验证） | 算法侧 |
| 2026-11-06 | P0-2 行人语义定为契约 v1.3 收编；P2 定为认领追认（不回炉）；补 v1.3 待办摘要 | 算法侧 |
| 2026-11-06 | 契约 v1.3 文档侧落地（接口契约/5场景需求清单/W4手册同步，字段名收编交付命名）；待办摘要标 ✅；加载器以需求形式落 W4 手册 §1.3 | 算法侧 |
| 2026-11-06 | grill 收口定案：P0-1 表口径 = `vy` 外科反号 + 生成器同修 + 单测；行人方向 = start→stop_at_y 推导（vy 为速率载体）；「表即事实」例外边界（几何冻结 / 标注类字段一次性补录回填走 PR）；加载器选型定案（box 解析距离为主、行人按 t 求位 + VO 段速度）；P1 确认已交付工程组 | 算法侧 |
| 2026-11-06 | 2.0.0 重冻结交付收口（随契约 v1.4）：P0-1 关闭（轨迹模型替换，反号处方作废）、P0-2 关闭（规范名回归）、P2 关闭（新维度追认扩写）、P1 保持开放；表落位修正知会；dead_end 新语义（目标在陷阱外、允许绕行）认领 | 算法侧 |
