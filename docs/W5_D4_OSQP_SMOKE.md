# W5 D4 — OSQP C++ standalone smoke 结果

> 2026-11-13 记。口径：`docs/W5执行计划.md` 决策 7、审查意见（逐轮更新 QP、两组计时、
> `SOLVED_INACCURATE` 单独计数、setup/求解失败路径、p95 预警不推广 W6 实时性）。
> 本文只记 D4 实测结果与边界，不改 W5 计划口径。

## 1. 版本与来源

| 项 | 值 |
|---|---|
| OSQP C 源码 | v0.6.3，commit `0dd00a578cf1c2691c5c379965d504c75bf6cfad` |
| qdldl | `7d16b70a10a152682204d745d814b6eb63dc5cd2`（`v0.1.3-40-g7d16b70`） |
| vendor 位置 | `nav_ws_core/src/mpc_controller/third_party/osqp/` |
| 版本/来源记录 | `third_party/VENDORED.md` |
| 执行二进制 | `nav_ws_core/install/mpc_controller/lib/mpc_controller/osqp_smoke` |
| 二进制 SHA-256 | `1a0e4e00cd04639ee50e5e8e3f21ad92c921067aed210e41944546533b0372d2` |
| 执行方式 | 普通用户、仓库目录、无 sudo、无联网、无额外第三方脚本；stdout/stderr/exit code 落 `out/w5_d4_osqp/` |

执行前已核对：克隆 HEAD 与 vendor 的 `include/`、`src/`、`lin_sys/`、`configure/`、
`CMakeLists.txt` 逐文件一致；vendor 无 `.git` 目录；`version.h` 为 `0.6.3`。

## 2. QP 规模与设置

- 变量 `n = 160 = 3×40（vx/vy/wz）+ 40 slack`；约束 `m = 320`
  （120 box + 120 rate + 40 slack≥0 + 40 软障碍）。
- `nnz(P)=160`（对角）、`nnz(A)=517`。
- OSQP 设置：`eps_abs=eps_rel=1e-3`、`max_iter=10000`、`warm_start=1`、`polish=1`、
  slack 正则 `1e-8`（与 W4 原型一致）。
- 1000 轮中每轮更新线性代价 `q`（正弦参考轨迹），**不是重复求解同一 QP**；
  固定 P/A/上下界，复用同一 workspace，保留热启动。

## 3. 实测结果（1000 次，exit code 0）

```text
OSQP C version: 0.6.3
vendored OSQP commit: 0dd00a578cf1c2691c5c379965d504c75bf6cfad
vendored qdldl commit: 7d16b70a10a152682204d745d814b6eb63dc5cd2
QP: n=160 m=320 nnz(P)=160 nnz(A)=517 N=40
settings: eps_abs=1.0e-03 eps_rel=1.0e-03 max_iter=10000 warm_start=1 polish=1 slack_reg=1.0e-08
runs=1000 hard_failures=0 solved_inaccurate=0
solver-only solve_ms mean=0.127 p95=0.131 max=3.705
update-only update_ms mean=0.000 p95=0.000 max=0.001
update+solve_ms mean=0.128 p95=0.132 max=3.705
iterations mean=25.57 max=600
last solve: status=1 (solved) pri_res=1.11e-16 dua_res=3.85e-14
SETUP FAILURE PATH: setup_status=2 (settings_validation), detected=1
FAILURE PATH: status=-3 (primal_infeasible), iterations=25
failure paths return status (caller must fall back to zero velocity)
RESULT: PASS
```

| 指标 | mean | p95 | max |
|---|---|---|---|
| solver-only `solve_ms` | 0.127 ms | 0.131 ms | 3.705 ms |
| update-only `update_ms` | ~0.000 ms | ~0.000 ms | 0.001 ms |
| update+solve `ms` | 0.128 ms | 0.132 ms | 3.705 ms |

- `p95=0.131 ms << 50 ms`，未触发 20 Hz 预算预警。
- `hard_failures=0`；`solved_inaccurate=0`；最后一次解 `pri_res≈1e-16`、`dua_res≈4e-14`。
- max 主要来自首次冷启动（600 iterations）；热启动后平均 25.57 iterations。

## 4. 失败与近似收敛路径

| 路径 | 构造 | 实际结果 | 处理 |
|---|---|---|---|
| setup 失败 | 合法数据 + `settings.scaling=-1` | `setup_status=2`（settings_validation），work 未创建 | 识别为 setup 失败，返回非零 |
| 求解不可行 | `x0+x1≥1` 且 `x0+x1≤0` | `status=-3`（primal_infeasible），25 iterations | 识别为失败，调用方须零速回退 |
| 近似收敛探针 | 同一 QP，`--runs 1 --max-iter 100` | `status=2`（solved_inaccurate），`pri_res=1.51e-2`、`dua_res=6.71e-1` | 单独计数并告警；结果为 `PASS_WITH_WARNINGS`，**不视为正常收敛** |

探针输出：

```text
settings: eps_abs=1.0e-03 eps_rel=1.0e-03 max_iter=100 warm_start=1 polish=1 slack_reg=1.0e-08
runs=1 hard_failures=0 solved_inaccurate=1
last solve: status=2 (solved inaccurate) pri_res=1.51e-02 dua_res=6.71e-01
WARN: OSQP_SOLVED_INACCURATE counted separately; W6 must judge residuals/safety
...
RESULT: PASS_WITH_WARNINGS
```

探针进程退出码为 0，但结果串明确为 `PASS_WITH_WARNINGS`，不与正常收敛的 `PASS` 混同。

## 5. 复现性

2026-11-13 从 `feat/w5-mpc-controller` 分支 fresh clone 到 `/tmp/w5_fresh_clone`，
执行：

```bash
cd nav_ws_core
colcon build --packages-select mpc_controller --symlink-install
```

构建 10.9 s 成功，OSQP/QDLDL 源码均来自版本库内 vendor 文件；`osqp_configure.h`、
`qdldl_types.h` 在构建时生成并由 `.gitignore` 忽略，克隆后 `git status` 干净。
未在克隆内执行 smoke 二进制（执行授权仅覆盖本次验证的构建产物）。

## 6. 边界

- 这是**独立 OSQP smoke**，QP 为按 W4 变量布局构造的代表性稀疏 QP；不包含
  SQP 线性化、QP 装配、EDT/障碍处理、ROS 插件与 Isaac 闭环。
- `p95=0.224 ms` 只说明该 QP 在 OSQP C 下的求解时间；**不能据此判定 W6 整控制周期
  满足 20 Hz**。50 ms 是整个控制循环预算，W6 必须计入完整链路耗时。
- 热启动与逐轮 q 更新降低了平均迭代数；首次冷启动 max 4.58 ms 应保留在视野内。
- `SOLVED_INACCURATE` 不应被无条件接受；W6 需结合残差与安全条件决定是否使用解，
  否则应按失败路径零速回退。
