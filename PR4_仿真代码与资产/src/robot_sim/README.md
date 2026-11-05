# 本次交付说明

默认 M1 使用本包桥接及真值节点，参见 [工程侧入口](../../docs/engineering/README.md)。
旧扩展与第三方资产不随本次交付，以下 W1 内容为历史记录。

# robot_sim

**状态：W1 骨架（E03 建立）。本包目前只承载“约定与资产清单”，没有 ROS 2 节点实现。**

## 职责

- 仿真资产、场景、Isaac Sim 运动学扩展的清单与版本记录；
- 桥接（`isaacsim.ros2.bridge`）、DDS 域、时钟与实时因子的约定来源；
- 后续 E06（全向速度输入）、E14（场景配置）、E15（自动执行的场景重置）的仿真侧实现落在这里。

## 当前内容

| 路径 | 说明 |
|---|---|
| `config/scenarios_assets.yaml` | 资产清单（指向仓库根 `assets/`），每个场景的用途、状态、来源 |
| `config/sim.yaml` | 桥接/DDS/时钟/实时因子的固化约定，全部来自 E01/E02 实测 |
| `isaac_exts/robot_a.kinematic_base/` | 2026-10-23 包的麦轮运动学扩展（0.4.0），原样纳入、未修改 |
| `docs/` | 预留（当前为空） |

## 使用方式（当前）

1. GUI 启动 Isaac Sim：`${ISAAC_SIM_ROOT}/isaac-sim.sh`（先 source `/opt/ros/humble`）；
2. Extension Manager 把本包的 `isaac_exts` 加入搜索路径，启用 `isaacsim.ros2.bridge` 与 `robot_a.kinematic_base`；
3. 打开 `assets/scene_robot_a_nav.usd`，按包内 README 的 `Play -> Capture Pose -> Stop -> Prepare Kinematic -> Play`。

## 明确的边界

- 本包**不做**物理接触动力学验证：`robot_a.kinematic_base` 是运动学位姿积分，不是轮地接触仿真（见路线书 E06 的提醒）；
- 本包**不发布** TF 或里程计：TF 发布责任在 `docs/接口契约.md` 第 5 节明确规定；
- 仿真世界的 `world`、地图原点与 `odom` 的固定关系在 E09 才落地，当前不要假定额定值。
