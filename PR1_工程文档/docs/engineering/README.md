# 工程侧入口（M1 / E14）

运行要求：Ubuntu 22.04、ROS 2 Humble（系统 Python 3.10）、Isaac Sim 5.1.0（Python 3.11）、
`isaacsim.ros2.bridge` 4.12.4，GPU 运行单实例。完整依赖见 [dependencies.md](dependencies.md)。

先激活安装 Isaac Sim 的 conda/venv 环境，再从仓库根目录运行以下两条命令；将 `ISAAC_PYTHON` 设置为你机器上可导入 `isaacsim` 的 Python 3.11：

```bash
export NAV_WS="$PWD" ISAAC_PYTHON="$(command -v python3.11)"; source tools/engineering/e09_sys_env.sh; ./tools/engineering/build_workspace.sh
source install/setup.bash; ros2 launch robot_bringup simulation.launch.py mode:=kinematic use_sim_time:=true
```

GUI 增加 `gui:=true`；外部控制器接管时加 `demo:=false`。headless 模式自动移除 `DISPLAY`。
基准统一使用 kinematic；该模式 IMU 不用于惯性性能评价。physics 的含重力 IMU 历史证据单独保留。
脚本位于 `tools/engineering/`，场景配置唯一位置是根目录 `scenarios/`。

```bash
./tools/engineering/verify_assets.sh
./tools/engineering/acceptance_check.sh  # Linux/ROS 完整验收，需另行复跑
PYTHONPATH=src/robot_benchmark python3 -m robot_benchmark.gen_scenarios --out scenarios --check
PYTHONPATH=src/robot_benchmark python3 -m pytest src/robot_benchmark/test/test_scenario_gen.py -q
```

| 内容 | 入口 |
|---|---|
| 环境、桥接、底盘 | [environment](environment.md)、[bridge_check](bridge_check.md)、[robot_model](robot_model.md) |
| 运动、传感器、TF | [motion_check](motion_check.md)、[sensors](sensors.md)、[tf_tree](tf_tree.md) |
| 包职责、资产、依赖 | [package_roles](package_roles.md)、[assets](assets.md)、[dependencies](dependencies.md) |
| 执行记录 | [W1–W3](records_w1_w3.md) |
| 契约及 Q1–Q7 | [v1.1 提案](../接口契约.md)、[对齐记录](../接口契约_工程侧对齐记录.md) |
| 场景与冻结实例 | [场景说明](../../scenarios/README.md)、[100 个实例清单](../../scenarios/test_list.yaml) |
| 证据摘要 | [证据索引](evidence_index.md)、[交付验证](../../results/upload_validation.md) |

原 Linux 完整验收历史为 `PASS=23 FAIL=0`，不代表本次 Windows 整理后已经复跑。
当前 `benchmark.run` / batch / summarize 仍为显式失败的契约骨架，E15/E16 未实现；Nav2 接入及安全层待后续开发。
QoS 本次显式改为 best_effort，需在 Isaac Sim + ROS 2 上复测后才能确认与旧实测频率一致。

环境盘点可另设 `ISAAC_SIM_ROOT`（Isaac Sim 安装目录）；它只供 e01_env_inventory.sh 检查文件，不用于覆盖运行时 Python。
