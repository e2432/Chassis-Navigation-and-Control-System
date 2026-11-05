# robot_bringup

**状态：W1 骨架（E03 建立）。可拉起 ROS 2 侧链路，但还不是 M1 的一键启动。**

## 职责

- 唯一的顶层启动入口：把模型/描述、仿真桥接、里程计与 TF、RViz、导航参数组合起来；
- 控制器选择（`controller:=dwb|mppi|teb|mpc`）的入口在此，**实现**在 E17/E18；
- 场景/模式切换（真值基准 vs SLAM 演示）的入口也在此，负责“只启用一套 TF 发布者”。

## 当前能做什么

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
# 先用 GUI 打开 assets/scene_robot_a_nav.usd 并 Play
ros2 launch robot_bringup simulation.launch.py use_sim_time:=true controller:=dwb
```

它会包含既有 `robot_a_sim_control/launch/robot_a_truth_navigation.launch.py`。

## 当前不能做什么（不要当成已完成）

| 路线书要求 | 现状 |
|---|---|
| 一条命令启动仿真 + 全部 ROS 2 节点（E10 / M1） | **未实现**：Isaac Sim 仍需手工启动 |
| `controller:=` 真正切换基线（E17） | **未实现**：参数只记录 |
| 场景重置与模式切换（E15/E19） | **未实现** |
| RViz 配置随启动加载（E10） | 沿用既有包内 `rviz/robot_a_nav2.rviz`，未重新整理 |
