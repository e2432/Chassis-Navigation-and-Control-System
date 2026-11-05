# nav_ws_core

工程侧核心交付：底盘模型、Isaac Sim / ROS 2 桥接、真值里程计与 TF、五类场景和 100 个冻结实例。
整个 `nav_ws_core/` 放在团队仓库根目录即可，只提交这一目录，使用一个 PR。

## 内容

| 目录 | 核心内容 |
|---|---|
| `src/robot_description` | 底盘 xacro、STL、几何参数 |
| `src/robot_sim` | 仿真桥接、真值节点、演示指令、场景搭建 |
| `src/robot_bringup` | 仿真与 TF 启动入口 |
| `src/robot_benchmark` | 场景生成器、校验 CLI 和单元测试 |
| `assets` | 自建底盘 USD、必需组合层、最小场景 |
| `scenarios` | 五类 YAML 模板、每类 20 个实例及 `test_list.yaml` |
| `config` / `scripts` | RViz 配置及环境、构建脚本 |

## 构建与启动

运行环境：Ubuntu 22.04、ROS 2 Humble（系统 Python 3.10）、Isaac Sim 5.1.0（Python 3.11、bridge 4.12.4）。
ROS 侧需 colcon、ament_cmake/ament_python、robot_state_publisher、xacro、rviz2、rclpy、tf2_ros，
及本目录各 package.xml 中的消息依赖；场景校验需 PyYAML、pytest；Isaac Python 需 PyYAML。

```bash
cd Chassis-Navigation-and-Control-System/nav_ws_core
export ISAAC_PYTHON="/你的Isaac环境/bin/python3.11"  # 改成可 import isaacsim 的 Python 绝对路径
source scripts/env.sh
bash scripts/build.sh
source install/setup.bash
ros2 launch robot_bringup simulation.launch.py mode:=kinematic use_sim_time:=true
```

打开仿真窗口加 `gui:=true`；禁用 RViz 加 `rviz:=false`；外部控制器接管时加 `demo:=false`。
正式点云 `/lidar/points`，IMU `/imu`，里程计 `/odom`；默认真值链为
`map → odom → base_footprint → base_link → 传感器`。不要同时运行另一套相同 TF 发布者。

## 场景校验与预览

```bash
PYTHONPATH=src/robot_benchmark python3 -m robot_benchmark.gen_scenarios --out scenarios --check
PYTHONPATH=src/robot_benchmark python3 -m pytest src/robot_benchmark/test/test_scenario_gen.py -q
"$ISAAC_PYTHON" src/robot_sim/robot_sim/scripts/scene_from_scenario.py --scenario static_avoid --run 1 --gui
```

五类为 `static_avoid / dynamic_pedestrian / narrow_passage / dead_end / mixed`。
各控制器共用 `scenarios/test_list.yaml` 和实例文件的完整取值；种子为场景序号 ×100 + 运行序号。
`--rich` 仅生成查看/压力变体，不修改冻结评分实例。预览仅搭建场景，不启动导航控制器。

## 当前边界

精简版不包含历史记录、截图、ADR、接口文档、第三方资产和未实现的执行/汇总骨架；只保留本文件说明。
基准统一用 kinematic；physics 平移存在偏差，kinematic 的 IMU 不能用于惯性性能评估。
雷达只输出 x/y/z，近场低矮障碍覆盖不足；模型仍有空视觉 prim 引用警告。
本次精简后验证：12 项场景测试通过，100 个冻结实例校验通过；语法、安装数据与资产文件引用检查通过。
ROS/Isaac 构建启动及 best_effort 运行验证仍需原 Linux 环境。
本交付不提供 Nav2 自动导航、安全层或完整基准评分功能。
