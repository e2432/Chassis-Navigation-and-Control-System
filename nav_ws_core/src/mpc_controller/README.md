# mpc_controller（W5 算法线插件骨架）

Nav2 `nav2_core::Controller` 插件包。W5 只做「加载 + 直通跟随 + 动态障碍心跳保护」；
EDT、完整 MPC 与 OSQP 移植留 W6。消息 `ObstacleState(.Array)` 与语义见 `msg/README.md`。

## 构建

```bash
source /home/robot/miniconda3/etc/profile.d/conda.sh
conda activate pro
source nav_ws_core/scripts/env.sh    # 仓库约定：ROS 构建使用系统工具链
cd nav_ws_core
colcon build --packages-select mpc_controller --symlink-install
source install/setup.bash
```

## D3 最小 smoke

```bash
# 全向（直线路径，检查非零 /cmd_vel、限速、失联零速、恢复）
ros2 launch mpc_controller mpc_controller_smoke.launch.py \
  chassis_type:=omni base_yaw_deg:=0.0

# 差速（机器人初始朝向 +Y，路径 +X；检查 vy=0 且 ω 转向）
ros2 launch mpc_controller mpc_controller_smoke.launch.py \
  chassis_type:=diff base_yaw_deg:=90.0

# 非恒等 map->odom（平移 + 旋转；路径在 map，插件逐拍变换到 odom）
ros2 launch mpc_controller mpc_controller_smoke.launch.py \
  chassis_type:=omni base_yaw_deg:=0.0 \
  map_x:=10.0 map_y:=5.0 map_yaw_deg:=90.0

# TF 缺失：安全停止（controller_server 可能在调用插件前 abort，无 /cmd_vel）
ros2 launch mpc_controller mpc_controller_smoke.launch.py \
  chassis_type:=omni publish_map_odom:=false tf_failure_expected:=true

# TF 超时：动态 map->odom 4 s 后停止，插件必须零速
ros2 launch mpc_controller mpc_controller_smoke.launch.py \
  chassis_type:=omni publish_map_odom:=true \
  map_odom_dynamic_seconds:=4.0 tf_failure_expected:=true
```

`setPlan()` 保存原始全局路径与 `frame_id`；`computeVelocityCommands()` 每拍按当前 TF
把路径变换到局部 costmap 的 `odom` 帧后再做前视跟随。TF 查询失败/超时返回零速，不回退到
未变换路径。D3 回归覆盖：omni 恒等、omni 非恒等（平移+旋转）、diff 转向、TF 缺失安全停止、
TF 超时零速，以及原有心跳失联与限速。

launch 包含：`controller_server + lifecycle_manager`、静态 `odom→base_footprint`、
假 `/odom`、全 `inf` `/scan`（让空局部 costmap 就绪）、10 Hz 空数组 mock、2 m 假路径
检查器。检查器按序验证：

1. 首条有效心跳前零速；
2. 有效心跳下非零 `/cmd_vel`；
3. 差速模式 `vy=0` 且 `ω` 转向；
4. 停 mock >0.5 s 后告警 + 零速；
5. 恢复心跳后指令恢复；
6. 绝对限速（0.2 m/s）不被突破、`NO_SPEED_LIMIT` 恢复底盘限速。

通过时输出 `RESULT: PASS`，launch 自动退出。

## D4 OSQP C++ smoke

```bash
# 1000 次、N=40、160 变量稀疏 QP（逐轮更新线性代价 q）
ros2 run mpc_controller osqp_smoke

# 触发/检查 SOLVED_INACCURATE 的探针（max_iter=100）
ros2 run mpc_controller osqp_smoke --runs 1 --max-iter 100
```

vendored OSQP 0.6.3（commit `0dd00a5`，qdldl `7d16b70`），详见
`third_party/VENDORED.md`。D4 实测结果与边界见 `docs/W5_D4_OSQP_SMOKE.md`
（位于仓库 `docs/`）。p95 只是 OSQP 求解时间，不代表 W6 整控制周期满足 20 Hz。

## 边界

- D3 不证明导航成功、不要求避障；不启动 planner/BT navigator。
- 底盘参数按 W5 方案 A 直接作 `FollowPath` 子键（`FollowPath.type` /
  `FollowPath.footprint.radius` / `FollowPath.limits.*`）。`diff` 由参数读取后
  强制 `vy_max=0`，算法代码不按底盘类型分支。
- 正式导航禁止用 mock 心跳；W6 换真跟踪器。
