# mpc_controller（W6 算法线 MPC 插件）

Nav2 `nav2_core::Controller` 插件包。W5 完成加载 + 直通跟随 + 动态障碍心跳保护；
W6 已把直通跟随器替换为 `MpcCore`（C++ 参考路径生成 + SQP/OSQP），保留心跳/TF/限速安全逻辑。
消息 `ObstacleState(.Array)` 与语义见 `msg/README.md`。

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

# TEST ONLY：非空场景障碍（行人按契约轨迹；墙不发布）
ros2 launch mpc_controller mpc_controller_smoke.launch.py \
  chassis_type:=omni obstacle_source:=scenario \
  scenario_name:=dynamic_pedestrian scenario_run:=1
```

# TEST ONLY：静态距离场（假 /map，墙在 y=2.0，验证 field 构建与约束接入）
ros2 launch mpc_controller mpc_controller_smoke.launch.py \
  chassis_type:=omni publish_static_map:=true

# 组合：非空动态障碍 + 静态距离场
ros2 launch mpc_controller mpc_controller_smoke.launch.py \
  chassis_type:=omni obstacle_source:=scenario \
  scenario_name:=dynamic_pedestrian scenario_run:=1 publish_static_map:=true
```

> `obstacle_source:=scenario` 与 `publish_static_map:=true` 仅用于集成测试，
> **禁止用于正式成绩**。静态墙通过 `/map` 距离场进入 MPC，动态障碍通过
> `/mpc/obstacles`；两者都使用时 smoke 会断言 `static_field=True` 且障碍数 >0。

`setPlan()` 保存原始全局路径与 `frame_id`；`computeVelocityCommands()` 每拍按当前 TF
把路径变换到局部 costmap 的 `odom` 帧，生成时变参考并交给 `MpcCore` 求解。
TF 查询失败/超时返回零速，不回退到未变换路径。D3 回归覆盖：omni 恒等、omni 非恒等
（平移+旋转）、diff 转向、TF 缺失安全停止、TF 超时零速，以及原有心跳失联与限速。
MPC 受加速度限幅，动力学检查等待收敛后判定。

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

## W6 MPC core（QP Builder / SQP / 回退）

`mpc_core` 已移植 W4 Python MPC 的 QP 装配与 SQP 求解（圆障碍 + 墙体、slack、
零速回退），独立于 Nav2。Python/C++ 数学一致性测试：

```bash
conda activate pro
cd prototype/mpc_numpy
python check_mpc_cpp_qp.py    # 逐项 P/q/A/l/u + 求解结果
python check_mpc_cpp_sqp.py   # SQP 正常 / max_iter 回退
python check_mpc_cpp_ref.py   # C++ 参考路径生成 vs Python
python check_mpc_cpp_field.py # 静态距离场 QP vs Python 参考
python field_sim_demo.py      # 合成闭环：绕障/窄通道
# 或一键：./run_w6_checks.sh
```

插件现已消费 `MpcCore + ReferencePath`：每拍按 TF 把全局路径变换到局部 costmap 帧，
生成时变参考并求解；MPC 参数在 `config/nav2_mpc_params.yaml` 的 `FollowPath.mpc.*`
（默认值与 W4 定稿一致）。本包默认 `CMAKE_BUILD_TYPE=Release`——实测 -O0 单拍
80~100 ms，超过 20 Hz 预算；Release 后约 4~13 ms（N=40、n_sqp=2、无动态障碍）。

## W6 静态距离场与诊断

- 静态障碍：订阅 `/map`（transient_local），构建 `GridDistanceField`（chamfer EDT），
  MpcCore 以软约束 + slack 接入；预测位姿经 map→odom 逆变换查询。
  参数：`use_static_distance_field`（默认 true）、`static_map_topic`（`/map`）、
  `static_field_max_distance`（5.0 m）。
- 动态障碍：`/mpc/obstacles` 非空时逐拍转换并送入 MPC；空数组清空动态障碍。
- 诊断：`MpcDiagnostics` 逐拍发布 `/mpc/diagnostics`（`diagnostics_topic` 可配），
  含 `solve_ms`、`qp_iterations`、`status`、`fallback_streak`、`alarm`、
  `obstacle_count`、`static_field_used`。该接口需契约变更 PR 双方确认。
- 自测：`./build/mpc_controller/distance_field_selftest`；smoke 用
  `publish_static_map:=true` / `obstacle_source:=scenario` 验证。
- 诊断录制（系统 Python，零数学）：
  `ros2 run mpc_controller record_mpc_diagnostics.py --ros-args -p output_file:=/tmp/mpc_diagnostics.csv`
- 离线聚合（conda pro）：
  `python aggregate_qp_timing.py --csv /tmp/mpc_diagnostics.csv --json-out /tmp/qp_timing.json`
- 一键数值检查（conda pro）：`cd prototype/mpc_numpy && ./run_w6_checks.sh`；
  另有 `field_sim_scenarios.py`、`field_sim_sweep.py`、`bench_mpc_py.py`。

结果与边界见仓库 `docs/W6_MPC_CORE.md`。真实 tracker、完整 Nav2 bringup、
Isaac 闭环与同环境 benchmark 仍待后续。

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

- D3 smoke 不证明导航成功、不要求避障；不启动 planner/BT navigator。
- 底盘参数按 W5 方案 A 直接作 `FollowPath` 子键（`FollowPath.type` /
  `FollowPath.footprint.radius` / `FollowPath.limits.*`）。`diff` 由参数读取后
  强制 `vy_max=0`。
- 正式导航禁止用 mock 心跳；当前仅在测试 launch 中使用，真实跟踪器尚未接入。
- 任务级平面限速由共享执行层负责，插件只响应显式 Nav2 speed limit，不重复限速。
