# nav_ws_core

工程侧核心交付：底盘、Isaac/ROS 桥接、真值 TF，以及按接口契约 v1.2 场景 schema v1 冻结的 100 个实例。
实例生成器版本为 **2.0.0（2026-11-06）**。这是一批重新冻结的数据，不能与旧版结果按相同 seed 混合比较。

## 上传位置

将更新包中的两个目录放到团队仓库根目录，一起提交一个 PR：

```text
docs/scenarios/instances.yaml   # 唯一实例表；契约指定路径
nav_ws_core/                   # 核心代码、五份生成模板和本说明
```

本次只保留这一个 Markdown 说明。若仓库已有旧版，请在同一 PR 删除
`nav_ws_core/scenarios/instances/` 下的五份旧实例表及 `nav_ws_core/scenarios/test_list.yaml`，避免两份冻结数据并存。

## 实例内容与本次变化

每类 20 个，共 100 个；seed 仍为场景序号 ×100 + run 序号，五类顺序不变。
每条记录严格使用契约十个字段：scenario、scenario_index、run、seed、path_length_m、time_limit_s、goal、obstacles、pedestrians、walls。
墙体使用轴对齐的 x0/y0/x1/y1，圆障碍使用 x/y/r；起点隐含为 (0,0)，目标航向取起终点连线方向。

- 行人改为 `x_cross/speed/t_meet/dir/r`，全程使用 `y = dir * speed * (t - t_meet)`，没有等待、停止或边界钳位。
- 窄通道的 20 份布局均不同：改变入口位置、横向偏移、净宽、长度和壁厚；包含薄导向墙与较深门洞两种结构。
- 死角的 20 份布局均不同：西、东、南、北四向袋口各 5 份，并改变位置、宽度、深度和壁厚。目标位于陷阱外，任务允许规划绕行；不宣称所有控制器都必须先进入再倒车。
- 表内 `world_poses` 按 seed 冻结整个场景的平移和朝向。窄通道、死角各覆盖八种世界朝向；它们的相对布局也有变化，非仅旋转同一份几何。
- `layouts` 保存两类结构的尺寸和方位元数据；这些是表顶层的可选信息，不给契约实例记录增加字段。
- 静态绕行和混合场景的既有圆障碍位置、半径及目标保留；动态行人的轨迹模型已统一修复。

算法侧直接消费 `instances` 中的 path-frame 数值。Isaac 预览默认再应用 `world_poses`，用同一个刚体变换处理机器人、墙体、障碍和目标；
`--path-frame` 可查看未经世界摆放的契约坐标。世界摆放不会改变墙体在 path frame 中的轴对齐性质或几何可达性。

## 构建与启动

环境：Ubuntu 22.04、ROS 2 Humble（系统 Python 3.10）、Isaac Sim 5.1.0（Python 3.11、bridge 4.12.4）。
ROS 侧需 colcon、ament_cmake/ament_python、robot_state_publisher、xacro、rviz2、rclpy、tf2_ros 及 package.xml 中的依赖。
场景生成/校验需 PyYAML、pytest；Isaac Python 也需 PyYAML。

```bash
cd Chassis-Navigation-and-Control-System/nav_ws_core
export ISAAC_PYTHON="/你的Isaac环境/bin/python3.11"
source scripts/env.sh
bash scripts/build.sh
source install/setup.bash
ros2 launch robot_bringup simulation.launch.py mode:=kinematic use_sim_time:=true
```

GUI 加 `gui:=true`，无 RViz 加 `rviz:=false`，外部控制器接管加 `demo:=false`。
正式点云 /lidar/points、IMU /imu、里程计 /odom；默认真值链为 map→odom→base_footprint→base_link→传感器。
上面启动的是最小桥接演示，不是 100 个实例的自动运行器。

## 统一表校验、复算与预览

以下命令从 nav_ws_core 运行，默认自动定位仓库根目录的统一表：

```bash
PYTHONPATH=src/robot_benchmark python3 -m robot_benchmark.gen_scenarios --check
PYTHONPATH=src/robot_benchmark python3 -m pytest src/robot_benchmark/test/test_scenario_gen.py -q
"$ISAAC_PYTHON" src/robot_sim/robot_sim/scripts/scene_from_scenario.py --scenario narrow_passage --run 7 --gui
"$ISAAC_PYTHON" src/robot_sim/robot_sim/scripts/scene_from_scenario.py --scenario dead_end --run 4 --gui
```

只在有意重新冻结时运行下列命令，它会重写统一表：

```bash
PYTHONPATH=src/robot_benchmark python3 -m robot_benchmark.gen_scenarios
```

CLI 的 `--out` 是 YAML 文件路径；预览用 `--manifest` 指定另一份表。`--all --cycle 10` 可按五类轮播；
`--save` 导出相对引用的 USD 静态快照。行人连续动画由脚本驱动，重新单独打开快照不会自动恢复动画。
旧的 rich 富化变体退出本次冻结流程，预览直接读取与算法侧相同的实例几何。

## 验证与边界

本次 24 项测试全部通过，覆盖契约字段、种子、确定性、静态 A* 可达性、行人过线/连续速度、真实预览动画函数、布局唯一性和世界位姿。
另用 OpenUSD 在内存中构建全部 100 个场景，完成 340 次行人三维坐标检查和 5 份代表场景导出引用检查。
静态 A* 通过只表示几何可达，不代表动态避障任务一定成功。

本次对齐的是场景实例表，不包含 benchmark.run、评分与 run.json/trajectory.npz 自动记录。
ROS/Isaac 实机运行仍需 Linux 环境复测；原模型的空视觉 prim 警告仍在。
kinematic 的 IMU 不能用于惯性性能评估。上轮审计中的统一底盘配置、任务限速、odom 重置清零、cmd_vel depth 等不在本次场景修改范围。
