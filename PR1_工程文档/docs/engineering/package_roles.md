# 团队交付范围说明

本次最小交付为 robot_description、robot_sim、robot_bringup、robot_benchmark 四包；
下列八包表保留原工作空间的历史规划。当前 M1 已由 robot_sim 专用真值节点及统一桥接运行，
旧控制包与 Isaac 扩展未随本次交付。最新 TF 归属见 [tf_tree.md](tf_tree.md)。

# 七包职责表（E03 产出）

编制日期：2026-10-30。依据：路线书 E03 的建议七包划分 + 对 2026-10-23 既有包的核对结果。

## 1. 结论：实际采用 7 个新包 + 1 个既有包

路线书建议的七个包已全部建立；另外把 2026-10-23 包里的 `robot_a_sim_control`
作为**既有实现**纳入仓库（不重复重写，理由见 `src/robot_a_sim_control/VENDORED.md`）。
两者关系在第 3 节写清楚：谁在哪一层、什么时候迁移、迁移完成前谁是唯一可运行链路。

| # | 包 | 语言/构建 | 职责一句话 | 当前状态 | 主要任务 |
|---|---|---|---|---|---|
| 1 | `robot_description` | ament_cmake | 几何/惯量/关节/传感器外参/footprint 的唯一来源 | **E05 最小底盘模型已完成**（xacro + Isaac 验收）；整机 footprint 待补 | E05/E06 |
| 2 | `robot_sim` | ament_cmake | 仿真资产清单、Isaac 扩展、桥接与时钟约定 | 清单与约定已建，无节点 | E06/E14/E15 |
| 3 | `robot_bringup` | ament_cmake | 唯一顶层启动入口、控制器选择、模式切换 | 可拉起 ROS 2 侧；非一键启动 | E10/E17/E19 |
| 4 | `robot_navigation` | ament_cmake | 地图来源、Nav2 公共参数、基线与 MPC 配置分层 | 分层骨架 + 占位基线，参数故意留空 | E11/E12/E17 |
| 5 | `robot_benchmark` | ament_python | 场景编排、自动执行、七项指标采集、结果汇总 | **契约可用**，执行未实现（显式报错） | E14/E15/E16 |
| 6 | `robot_safety` | ament_cmake | `/cmd_vel` 唯一出口、独立急停、碰撞记录 | 规则草案，节点未实现 | E13 |
| 7 | `robot_mpc` | ament_cmake | 算法侧 MPC 插件落位（占位） | 占位包，等算法成员填充 | E18 |
| 8 | `robot_a_sim_control` | ament_python | **既有包**：仿真控制桥/轮速里程计/真值定位/Nav2 配置/巡航 | 原样纳入、可构建、可运行 | 逐步拆分（W4 起） |

## 2. 接口边界（哪个包能发布什么）

| 接口 | 唯一责任方 | 说明 |
|---|---|---|
| `/clock` | 仿真场景内的 Graph（`robot_sim` 管辖的场景） | 其它任何节点不得发布 |
| `/odom`、`odom→base_link` | 真值适配（现在：`robot_a_sim_control/state_bridge`；目标：`robot_sim`） | 同一变换只允许一个发布者 |
| `map→odom` | 避障真值模式下的专用节点（现在：`truth_localization`） | SLAM 模式下必须停用这一路 |
| `base_link→livox_frame` 等外参 | `robot_description`（经 `robot_state_publisher`） | 不允许各处各写一份外参 |
| `/cmd_vel` | `robot_safety`（趋势目标；当前由既有 `command_bridge` 直接消费） | 见 `docs/接口契约.md` 第 4 节 |
| 导航目标 | Nav2 Action（`navigate_to_pose`） | 不作为普通话题发送 |
| 场景/重置/目标注入 | `robot_benchmark` 调用 `robot_sim` | 接口在 E15 定稿 |

## 3. 与既有包的迁移计划（避免两套真相）

| 能力 | 现在在哪 | 目标在哪 | 迁移判定条件 |
|---|---|---|---|
| 底盘速度输入与超时 | `robot_a_sim_control/command_bridge` | `robot_sim` + `robot_safety` | E06 完成全向测试且 E13 安全层上线后 |
| 真值里程计与 TF | `robot_a_sim_control/state_bridge` + `truth_localization` | `robot_sim` | E09 完成唯一 TF 树并复测通过 |
| Nav2 参数 | `robot_a_sim_control/config/nav2.yaml` | `robot_navigation/config/nav2_common.yaml` + `baselines/*.yaml` | E11 完成地图对齐、E17 接第一种基线后 |
| 地图 | `robot_a_sim_control/maps/` | `robot_navigation/maps/` | E11 记录地图分辨率/原点并与 TF 核对后 |
| RViz 配置 | `robot_a_sim_control/rviz/` | `robot_bringup`（随启动加载） | E10 完成一键启动时 |

迁移期间规则：
1. 同一能力**只能有一处生效**，另一处必须显式标注“未启用/待迁移”；
2. 迁移提交必须同时更新本表，并在 `docs/接口契约.md` 的变更记录里登记；
3. 既有包在迁移完成前**不修改源码**，修补走新包或接口层。

## 4. 构建与可见性验收（已实测）

```text
$ ./tools/engineering/build_workspace.sh
Summary: 8 packages finished [1.78s]

$ ros2 pkg list | grep -E '^robot'
robot_a_sim_control  robot_benchmark  robot_bringup  robot_description
robot_mpc  robot_navigation  robot_safety  robot_sim
```

## 5. 待确认（需算法成员签字）

- [ ] 第 1 节的包划分与第 3 节的迁移计划是否同意（尤其是“既有包不重写、按表迁移”）；
- [ ] `robot_safety` 是否要和 `robot_mpc` 一样由算法侧参与参数定义（当前按工程侧主责）；
- [ ] 是否需要为算法侧单开一个只装控制器的工作空间（当前结论：不需要，避免两套测试环境）。
