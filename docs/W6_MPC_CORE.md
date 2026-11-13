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

未做（按审查顺序）：

- C++ 参考路径生成器（当前 SQP 测试的 `z_ref/u_ref` 由 Python 生成；插件接入前需 C++ 化）。
- Nav2 插件内替换直通跟随器、M2 launch、真实 `/lidar/points` costmap、真实跟踪器、Isaac 闭环。

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

- 当前只验证数学核心；未接入 Nav2、tracker、EDT、Isaac。
- 求解解差在 1e-4 量级；矩阵级一致性已达到 1e-8 容差。
- 下一步：
  1. C++ 参考路径生成（时间律、折线、目标截断），替换测试用 Python 输入；
  2. 插件内把直通跟随器切换到 `MpcCore`，保留心跳/限速/TF 安全逻辑；
  3. M2 launch/config 到位后接 Nav2、真实 `/lidar/points` costmap、真实跟踪器与 Isaac。
