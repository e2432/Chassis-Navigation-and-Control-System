# W6 — MPC Core C++ 移植（QP Builder / SQP / 回退）

> 2026-11-13 记。W6 第一切片：独立移植 W4 Python MPC 的 QP 装配与 SQP 求解，
> 不等 M2 Nav2 launch。本文记录实现、Python/C++ 数学一致性结果与边界。

## 1. 范围

已完成：

- `mpc_controller::MpcCore`：`buildQp()` 与 `solve()`；
  - 状态 LTV 递推/condense、七项代价、box/增量/障碍软约束/slack；
  - 圆障碍 + 轴对齐墙体解析 clearance；
  - SQP `n_sqp` 次；OSQP C 0.6.3 求解；
  - OSQP 失败 → 本拍零速回退、streak 计数、连续 5 拍 alarm。
- `mpc_qp_dump`：输出 P/q/A/l/u 与 OSQP 解，用于矩阵一致性测试。
- `mpc_sqp_dump`：输出 SQP 结果（success/fallback/streak/alarm/u₀），用于求解一致性测试。
- Python 对照脚本：
  - `prototype/mpc_numpy/check_mpc_cpp_qp.py`
  - `prototype/mpc_numpy/check_mpc_cpp_sqp.py`
- C++ 参考路径生成器 `mpc_controller::ReferencePath`（移植 `mpc_proto/reference.py`；
  时间律 / 折线插值 / 目标截断 / 角包裹）与 `mpc_ref_dump`。
- Python 对照脚本 `prototype/mpc_numpy/check_mpc_cpp_ref.py`。
- Nav2 插件 `MpcController` 已用 `MpcCore + ReferencePath` 替换 W5 直通跟随器
  （保留心跳失联零速、TF 失败零速、限速、diff→vy=0 等安全逻辑）；参数见
  `config/nav2_mpc_params.yaml` 的 `FollowPath.mpc.*`。
- 静态障碍：`GridDistanceField` 从冻结静态地图 `/map` 构建距离场（chamfer EDT），
  MpcCore 以软约束 + slack 接入；map→odom 变换后查询、梯度回转 odom。
- 诊断：`mpc_controller/MpcDiagnostics` 逐拍发布 `/mpc/diagnostics`
  （solve_ms / iterations / status / fallback_streak / alarm / obstacle_count /
  static_field_used）。

未做（按审查顺序）：

- 动态障碍真实跟踪器：当前可用 TEST ONLY 场景发布器验证非空路径，正式数据源待定。
- M2 launch/config 联调、完整 Nav2 bringup、Isaac 闭环、同环境 benchmark。
- `qp_timing` / `ablation` / `solver_fail` 写入 run.json 的采集器聚合
  （字段与接口见 `docs/W6_工程侧答复.md`，待双方确认）。

## 2. QP Builder 一致性

固定同一输入（N=6、dt=0.05、1 圆障碍 + 1 墙体、slack 开启）：

```text
PASS P: max abs diff=6.353e-22 (tol=1e-8)
PASS q: max abs diff=2.776e-17 (tol=1e-8)
PASS A: max abs diff=0.000e+00 (tol=1e-8)
PASS l: max abs diff=0.000e+00 (tol=1e-8)
PASS u: max abs diff=0.000e+00 (tol=1e-8)
PASS control_u: max abs diff=1.894e-04 (tol=1e-3)
PASS slack_s: max abs diff=0.000e+00 (tol=1e-3)
python OSQP full:  status=solved iter=300
python OSQP upper: status=solved iter=300
C++ OSQP:          status=solved
RESULT: PASS
```

说明：P/q/A/l/u 逐项一致；控制解差 1.894e-4（相对任务速度约 0.03%），在
1e-3 容差内。差异来源是 Python `osqp 0.6.7.post3` 与 C OSQP 0.6.3 的求解路径，
不是 QP 装配差异。Python 全对称 P 与上三角 P 的结果相同，C++ 按 OSQP C API 传上三角。

运行：

```bash
conda activate pro
cd prototype/mpc_numpy
python check_mpc_cpp_qp.py
```

## 3. SQP 与安全回退一致性

固定参考路径、1 圆障碍 + 1 墙体、`n_sqp=2`：

```text
--- normal ---
python: success=True  fallback=False status=solved u=[0.05, 0.02427978, 0.1]
C++:    success=True  fallback=False status=solved u=[0.05, 0.02444441, 0.1]
control diff: 1.646e-04 (tol=1e-3)
PASS normal

--- fallback (max_iter=1) ---
python: success=False fallback=True status=maximum iterations reached u=[0,0,0]
C++:    success=False fallback=True streak=1 u=[0,0,0]
PASS fallback
```

运行：

```bash
conda activate pro
cd prototype/mpc_numpy
python check_mpc_cpp_sqp.py
```

## 4. 代码/构建

- `nav_ws_core/src/mpc_controller/include/mpc_controller/mpc_core.hpp`
- `nav_ws_core/src/mpc_controller/src/mpc_core.cpp`
- `nav_ws_core/src/mpc_controller/src/mpc_qp_dump.cpp`
- `nav_ws_core/src/mpc_controller/src/mpc_sqp_dump.cpp`
- `CMakeLists.txt`：新增 `mpc_core`、`mpc_qp_dump`、`mpc_sqp_dump`；链接 vendored
  OSQP 0.6.3 与 Eigen3。

```bash
conda activate pro
source nav_ws_core/scripts/env.sh
cd nav_ws_core
colcon build --packages-select mpc_controller --symlink-install
```

## 5. 边界与下一步

- 数学核心 + C++ 参考生成 + 插件骨架 smoke 已验证；**未接真实 tracker、真实点云
  costmap、完整 Nav2 bringup 与 Isaac 闭环**。
- 求解解差在 1e-4 量级；矩阵级一致性已达到 1e-8 容差；参考序列逐点一致（0.0 差异）。
- 下一步：
  1. `/mpc/obstacles` 真实数据源（跟踪器或明确的测试场景发布器）与动态场景闭环；
  2. M2 launch/config 联调、真实 `/lidar/points` costmap、Isaac 闭环；
  3. `qp_timing` / `ablation` / `solver_fail` 落盘（见 `docs/W6_工程侧答复.md`）。

## 6. C++ 参考路径生成一致性（2026-11-18）

- 实现：`include/mpc_controller/reference.hpp`、`src/reference.cpp`，逐行移植
  `prototype/mpc_numpy/mpc_proto/reference.py` 的 `ReferencePath`：
  `s(t) = min(v_ref·t, L)`、`searchsorted` 折线插值（side="right"）、零长段保护、
  目标截断后前馈全零、角差包裹。
- 测试：`prototype/mpc_numpy/check_mpc_cpp_ref.py`（`mpc_ref_dump` 对拍）。
  5 个用例（普通折线、目标截断、零长段、转弯/角包裹、目标落在顶点）
  `z_ref` 与 `u_ref` 最大绝对差均为 **0.000e+00**（容差 1e-12），RESULT: PASS。

## 7. 插件接入与 smoke（2026-11-18）

- `MpcController` 每拍：TF 把全局路径变换到局部 costmap 帧 → `ReferencePath` 生成
  `z_ref/u_ref` → `MpcCore::solve(z0, z_ref, u_ref, obstacles)` → 输出 `u`。
  新路径在 `setPlan()` 重置参考锚点、warm start 与失败 streak。
- 安全逻辑保持不变：心跳失效零速、TF 缺失/超时零速、`diff` 强制 `vy=0`、
  显式 Nav2 speed limit 才在插件内限速（任务级 0.6 m/s 仍由共享执行层负责，
  不重复限速）。
- 参数：`FollowPath.mpc.{N,dt,v_ref,q_*,r_*,terminal_factor,rho_slack,d_safe,
  n_sqp,eps_abs,eps_rel,max_iter,slack,terminal}`；默认值与 W4 定稿一致。
- 构建：`mpc_controller` 默认 `CMAKE_BUILD_TYPE=Release`。**实测 -O0 时单拍求解
  约 80~100 ms，超过 20 Hz 预算；-O3 后约 4~13 ms**（N=40、n_sqp=2、
  `qp_iters` 75~175、无动态障碍）。后续接入真实障碍后需重测。
- D3 smoke 回归（`mpc_controller_smoke.launch.py`，Release）：
  omni 恒等、diff（初始 yaw 90°）、omni 非恒等 `map→odom` 三组均 `RESULT: PASS`；
  覆盖首心跳前零速、有效心跳非零、失联零速、恢复、绝对限速 0.2 m/s、
  `NO_SPEED_LIMIT` 恢复。MPC 受加速度限幅影响，smoke 检查器改为等待收敛后判定。
- 已知限制：真实跟踪器、完整 Nav2/Isaac 闭环与整机实时性仍未验证。

## 8. 静态距离场与诊断（2026-11-18）

### 8.1 设计

- **数据源**：`/map`（冻结静态清单几何，与三基线 `static_layer` 同源）；不把局部
  costmap 的 `ped_corridor_layer` 混入静态场，动态信息仍走 `/mpc/obstacles`，
  对应「同源真值、各自接口」。
- `GridDistanceField`：占用格（值 ≥50）→ chamfer 距离变换（1/√2 权重，最大 5 m）；
  双线性插值距离与梯度。
- `MpcCore`：每拍每阶段加一条软约束 `d(p_map) − r_robot ≥ d_safe − slack`；
  预测位姿经 map→odom 逆变换到 map 查询，梯度旋转回 odom；无 field 或越界时该行置空。
- 插件参数：`use_static_distance_field`、`static_map_topic`（默认 `/map`）、
  `static_field_max_distance`；地图 QoS 为 transient_local。

### 8.2 验证

- `distance_field_selftest`：左右距离/梯度方向、越界、slack 列数、约束行数、
  约束行非零均 PASS。
- smoke `publish_static_map:=true`：静态场构建成功，诊断 `static_field=True`，
  `RESULT: PASS`。
- smoke `obstacle_source:=scenario ... publish_static_map:=true`：`obstacles=2`、
  `field=1`、单拍 16.4~18.1 ms，`RESULT: PASS`。
- 仅 Test-only 场景发布器；墙不由 `/mpc/obstacles` 发布，静态墙全部来自 `/map` 距离场。

### 8.3 Python 参考与闭环验证（conda pro）

- Python 参考实现：`mpc_proto/distance_field.py`（与 C++ 同序 chamfer、双线性、
  梯度归一化）；`MPCSolver._solve_qp/solve` 新增可选静态场参数（默认 None，
  W4 行为不变）。
- `check_mpc_cpp_field.py`（pro）对拍 C++ `mpc_qp_dump` 的可选 `field` 块：
  `P` 差 6.8e-21、`q` 差 6.9e-18、`A/l/u` 差 **0**；控制解差 1.1e-5（tol 1e-3）；
  identity 与旋转 `map→odom` 均 **PASS**。
- `field_sim_scenarios.py`（pro）合成闭环，全部 PASS：
  - `bump`：10.60 s，最小间距 0.298 m，y≈-0.23（绕行）；
  - `narrow`：10.60 s，最小间距 0.200 m，max_slack 0.100（软约束）；
  - `dead_end`：11.75 s，最小间距 0.301 m（进出 U 形陷阱）；
  - `pedestrian`：10.60 s，行人最小间距 0.325 m（无穿透）。
- `field_sim_sweep.py`（pro）扫参 63 组 **全部 PASS**：
  - `d_safe` 0.2/0.3/0.4 × `rho_slack` 20/80/200 × `terminal_factor` 1/2/4（bump+narrow）；
    `d_safe=0.4` 在 narrow 下 slack≈0.20，但无穿透；`rho`/`terminal` 未见定性差异。
  - `v_ref` 0.4/0.5/0.6 在 bump/narrow/pedestrian 全 PASS，到达时间约
    13.20/10.60/8.85 s。
  - CSV：`prototype/mpc_numpy/out/w6_field_sweep/sweep.csv`。
- `bench_mpc_py.py`（pro）：N=40 + 静态场 + 1 动态障碍，200 拍
  **mean 27.5 ms / p95 34.0 ms / max 46.4 ms**，0 fallback；
  这是 Python 原型，C++ 插件实时性见 §7。
- 诊断链路：`record_mpc_diagnostics.py`（系统 Python，零数学）→ CSV；
  `aggregate_qp_timing.py`（pro）→ `qp_timing` JSON/MD。
  smoke 录制 47 拍：**mean 8.65 ms / p95 14.67 ms / max 16.05 ms**。
- 一键检查：`prototype/mpc_numpy/run_w6_checks.sh`（QP/SQP/ref/field/demo/scenarios/selftest）。

### 8.3 诊断

- `mpc_controller/MpcDiagnostics` 逐拍发布 `/mpc/diagnostics`；采集器可聚合为
  `qp_timing`，原始 `qp_ms` 写 `trajectory.npz`。
- 该消息/话题为新增接口，需契约变更 PR 双方确认；字段定义见 `docs/W6_工程侧答复.md` §3。
- 已知：`/map` 每收到一帧就重建距离场（fake map 1 Hz、40×40 无感）；
  实际 800×800 地图需观察重建频率，后续可按地图哈希/版本去重。
