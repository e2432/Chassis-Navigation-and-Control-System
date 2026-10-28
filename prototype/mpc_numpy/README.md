# mpc_numpy —— W2 numpy 开环模拟原型

自包含滚动推演（不接 ROS/Isaac，闭环于完美运动学模型，障碍为场景脚本数据）。
数学依据：[docs/MPC公式化.md](../../docs/MPC公式化.md)；W5~6 移植 C++ Nav2 插件（`mpc_controller`，见 [docs/Nav2插件教程笔记.md](../../docs/Nav2插件教程笔记.md)）。

- [启动指令教程](#启动指令教程)
- [验收四条](#验收四条每场景)
- [代码文件详解](#代码文件详解)
- [口径备忘](#口径备忘)

## 目录结构

```
prototype/mpc_numpy/
├── run_w2.sh            # W2 验收①：一条命令跑全部
├── run_sim.py           # 场景入口 CLI（跑模拟 + 验收表 + 诊断图）
├── sweep_horizon.py     # 预测时域 N 敏感性扫描
├── edt_sanity.py        # 非验收：栅格 EDT 梯度排雷
├── README.md
├── chassis/
│   ├── omni.yaml        # 全向轮底盘（契约 schema v1）
│   └── diff.yaml        # 差速轮（vy_max=0，迁移冒烟）
├── mpc_proto/           # 核心库（被上面的脚本 import）
│   ├── mpc.py           # QP 装配 + OSQP 求解 + 回退
│   ├── reference.py     # 折线参考 + 时间律推进 + 目标截断
│   ├── distance.py      # 解析距离场（静态 + 动态预测）
│   ├── scenarios.py     # S1/S2/S3 场景定义
│   ├── sim.py           # 滚动推演引擎 + 验收计算
│   ├── plotting.py      # 诊断图
│   └── chassis.py       # 底盘 YAML 载入
└── out/                 # 生成的诊断图（git 忽略）
```

## 启动指令教程

### 0. 环境准备（conda pro）

依赖都在 conda 环境 **`pro`**（numpy / scipy / osqp / matplotlib / pyyaml）。
两种等价用法，推荐 **A**（不依赖 shell 状态）：

```bash
# A. 直接用 pro 环境的 python 绝对路径（推荐，脚本与 run_w2.sh 均按此写）
PY=/home/robot/miniconda3/envs/pro/bin/python

# B. 激活 conda 环境后用 python（注意：base 的 python3 抢 PATH，必须先激活 pro）
source ~/miniconda3/etc/profile.d/conda.sh
conda activate pro
PY=python
```

⚠️ 陷阱：`conda activate base` 或未激活时，`python3` 指向 base（3.13），会报
`ModuleNotFoundError: osqp`。本原型不依赖 ROS 系统 Python，也不依赖 Isaac Sim。

进入目录并验证依赖：

```bash
cd /home/robot/project1/Chassis-Navigation-and-Control-System/prototype/mpc_numpy
$PY -c "import numpy, scipy, osqp, matplotlib, yaml; print('env ok')"
```

### 1. 一条命令跑通（W2 验收①）

```bash
./run_w2.sh
```

输出依次为：三场景验收表 → EDT sanity → `out/S{1,2,3}.png` 诊断图。
退出码 0 ⟺ 三场景全部到达且验收②③④通过（`run_w2.sh` 内部已固定用 pro 的 python）。

### 2. 分脚本指令

```bash
PY=/home/robot/miniconda3/envs/pro/bin/python   # 以下命令前先设好

# 2.1 三场景全跑（默认 N=40，出验收表 + 诊断图）
$PY run_sim.py --all

# 2.2 单场景 / 调参试验（N、底盘可换）
$PY run_sim.py --scenario S2                  # 只跑 S2（S1/S2/S3 三选一）
$PY run_sim.py --scenario S1 --N 20           # 换预测时域
$PY run_sim.py --all --chassis chassis/diff.yaml   # 迁移冒烟预演（vy_max=0，代码零改动）
$PY run_sim.py --all --no-plot                # 不出图（只看表）
$PY run_sim.py --all --quiet                  # 不打逐步日志（run_w2.sh 用这个）
$PY run_sim.py --all --out /tmp/mpc_out       # 诊断图换个输出目录

# 2.3 预测时域敏感性扫描（N ∈ {20,40,60} × 三场景，定 N 依据）
$PY sweep_horizon.py

# 2.4 EDT sanity（非验收：scipy 栅格 EDT + 有限差分梯度 vs 解析梯度）
$PY edt_sanity.py
```

**看输出**：验收表 10 列——`②逐步解算 / ③间隙>0 / ④Δu有界` 全 `PASS` 且「到达=是」才算过；
`min间隙` 是外缘间隙（真值几何），`TV/拍` 是归一化控制量总变差（界 0.30），
`max slack` 是障碍软约束松弛量（S3 窄通道 > 0 属软约束正常工作点）。
诊断图为三联图：轨迹俯视（参考折线/机器人轨迹/障碍）、全程间隙时序、控制量（线速度+角速度分面）。

### 3. 常见问题

| 现象 | 原因与处理 |
|---|---|
| `ModuleNotFoundError: osqp`（或 numpy/scipy） | 用错解释器——用 pro 的 python 绝对路径（上面 A 法） |
| `./run_w2.sh: Permission denied` | `chmod +x run_w2.sh` |
| 找不到 `mpc_proto` 模块 | 必须在 `prototype/mpc_numpy/` 目录下运行脚本（脚本已把自身目录插进 `sys.path`） |
| 验收② FAIL（解算 fail > 0） | OSQP 未收敛——看日志里的回退记录；连续 5 拍失败会报警（不切第二控制器） |
| 验收③ FAIL（间隙 ≤ 0） | 穿模——检查场景坐标/权重，见 docs/MPC公式化.md §4、§9 |

## 验收四条（每场景）

| # | 判据 | 口径 |
|---|---|---|
| ① | 一条命令跑通 | `./run_w2.sh` 退出码 0 |
| ② | 逐步求解成功 | 每控制拍 OSQP `solved`，零速回退 = 0 |
| ③ | 推演全程间隙 > 0 | 外缘间隙（真值几何）全程 > 0 |
| ④ | Δu 总变差有界 | TV/拍 < 0.30（归一化控制量总变差） |

到达判定复用指标定义 0.25 m / 0.25 rad。

## 代码文件详解

### `run_w2.sh` —— 验收入口

W2 验收①的「一条命令」。`set -euo pipefail` 保证任何一步失败即退出非 0；
固定调用 pro 的 python 绝对路径（找不到则报错退出）；依次执行
`run_sim.py --all --quiet`（验收表）与 `edt_sanity.py`（非验收 sanity）。

### `run_sim.py` —— 场景入口 CLI

解析命令行（`--all` / `--scenario` / `--N` / `--chassis` / `--out` / `--no-plot` / `--quiet`），
载入底盘与 MPC 配置，逐场景调 `sim.run()`，打印验收表并画诊断图。

| 函数 | 作用 |
|---|---|
| `main() -> int` | CLI 主流程；返回退出码 0 ⟺ 全部场景「到达 ∧ 验收②③④通过」 |
| `fmt_row(r: RunResult) -> dict` | 把一次推演结果压成验收表的一行（含三条 PASS/FAIL 判定） |
| `print_table(rows)` | 定宽打印验收表 |

### `sweep_horizon.py` —— 视界敏感性扫描

对 N ∈ {20, 40, 60} × S1/S2/S3 共 9 次完整推演，输出到达时间/间隙/TV/回退数对照表
与各 N 汇总（到达数、最小间隙下限、TV/拍上限）。结果是 docs/MPC公式化.md §10
「视界定案 N=40」的依据。

### `edt_sanity.py` —— EDT 梯度排雷（**非验收**）

为 W5~6 静态障碍改走 costmap 栅格 EDT 预演。S1 圆盘布局栅格化（2 cm），
`scipy.ndimage.distance_transform_edt` 求 EDT，对比：

- 间隙值：`EDT − r_robot` vs 解析 `‖p−c‖ − r_total`（自由区 max 误差应 < 2.5 格）
- 梯度方向：EDT 有限差分梯度 vs 解析径向（环带 cos∠ P5 > 0.9）

sanity 口径，不进 W2 验收四条。

### `mpc_proto/mpc.py` —— QP 装配与求解（核心）

七项代价栈 → 一个 condensed QP → OSQP；障碍软约束 + limits box/线性约束 + 回退策略。

| 符号 | 作用 |
|---|---|
| `MPCConfig` | 全部权重与开关：`N`、`dt`、`q_lat/q_long/q_theta`、`r_v/r_omega`、`r_dv/r_domega`、`terminal_factor`（≥2×）、`rho_slack`（L1 大权重）、`d_safe`、`n_sqp`（每拍 SQP 迭代数）、OSQP 容差。`Q_stage()/Q_terminal()/R()/Rd()` 产出对角权重阵 |
| `SolveResult` | 一次求解结果：`u`（本拍控制）、`success/status/fallback`、`max_slack`、`qp_iters`、`U`（全时域解，供 SQP 名义更新） |
| `rollout(z0, u_seq, dt) -> (N+1,3)` | 非线性完美运动学零阶保持积分（`ẋ=v_x cosθ−v_y sinθ` 等）；既做名义轨迹又做仿真真值 |
| `MPCSolver` | 求解器状态机：持 `u_prev`（增量约束用）、`fallback_streak`、`alarm` |
| `MPCSolver.solve(z0, obstacles, reference, t_now) -> SolveResult` | 每拍入口：生成参考 → 前馈名义 rollout → **SQP 迭代 n_sqp 次**（线性化→解 QP→用解更新名义）→ 取 `u[0]`；任何一次 OSQP 失败走**零速回退**（记日志；连续 5 拍置 `alarm`，不切第二控制器）。SQP 后续迭代失败沿用上一次成功解 |

`MPCSolver._solve_qp()` 内部流程（对应 docs/MPC公式化.md §7）：
① LTV 递推算 `Xi_k, xi_k` 使 `z_k = Xi_k·U + xi_k`；② 装配 H/f（误差二次项 + 控制能量 + 控制增量 + slack 线性罚）；
③ 装配约束（u box、Δu 界、障碍软约束 `∇dᵀ p_k + s ≥ d_safe − d̂ + ∇dᵀ p̂`、s ≥ 0）；④ OSQP 求解。

### `mpc_proto/reference.py` —— 时变参考

| 符号 | 作用 |
|---|---|
| `ReferencePath` | 折线参考几何（waypoints，NavFn 全局路径的 W2 替身）+ 时间律 `s(t) = min(v_ref·t, L)` |
| `ReferencePath.sequence(t_now) -> (z_ref, u_ref)` | 生成 k=0..N 的参考位姿（绝对时刻 t_now+k·dt）与前馈控制；**目标截断**：s 到终点后钉在目标位姿、前馈置零 |
| `ReferencePath.error_matrix(θ_ref) -> (3,3)` | 误差投影阵 M_k：世界系位置误差 → (e_lat, e_long, e_θ) |
| `wrap_angle(a)` | 角度 wrap 到 (−π, π] |
| `straight_path(start, goal, ...)` | 两点直线参考的便捷构造 |

口径：参考按时间律**不**逐拍锚定机器人投影——弱沿程权重（q_long ≤ 0.1 q_lat）下
重新锚定会让 e_long 只看一步以远，陷入「停在障碍前」的短视最优。

### `mpc_proto/distance.py` —— 解析距离场

| 符号 | 作用 |
|---|---|
| `Obstacle` | 单圆近似障碍（契约 `ObstacleState` 的脚本数据形态）：`p0` 当前位置、`v` 速度（静态=0）、`radius`；`center_at(tau)` = 匀速直线短期预测（VO 管道的预测中心） |
| `clearance(p, obs, tau, r_r) -> Clearance` | 解析外缘间隙 `d = ‖p−c‖ − (r_j+r_r)` 与解析梯度 `∇d = (p−c)/‖p−c‖`；`‖p−c‖<1e-9` 奇异保护取 (0,1) |
| `min_clearance(p, obstacles, tau, r_r)` | 多障碍取最小间隙（验收③与诊断图用） |

### `mpc_proto/scenarios.py` —— 场景定义

| 符号 | 作用 |
|---|---|
| `Scenario` | 场景数据包：`start/goal`（3 维位姿）、`waypoints`（参考几何）、`obstacles`、`t_max` 时限 |
| `s1_static_avoid()` | S1 静态绕障：静障 (4,0) r=0.6，参考折线从上方绕 |
| `s2_dynamic_crossing()` | S2 动态横穿：行人 (4,−4.9) 以 0.7 m/s 横穿，直参考，约 t≈7 s 相遇 |
| `s3_mixed_passage()` | S3 静+动混合（双障碍）：通道静障 (5,0.8) + 对向动障 0.7 m/s |
| `ALL` / `by_name(name)` | 场景清单与按名取用 |

改场景坐标/参考 waypoints 就改这里（W2 落地参数，与 docs/MPC公式化.md §9 一致）。

### `mpc_proto/sim.py` —— 滚动推演引擎

| 符号 | 作用 |
|---|---|
| `run(scenario, chassis, cfg, dt_sim=0.01) -> RunResult` | 主循环：每控制拍（0.05 s）取障碍状态 → `MPCSolver.solve()` → **细步积分**（dt_sim=0.01 s，5 细分步零阶保持）→ 记录细步真值间隙与轨迹；到达判定（0.25 m / 0.25 rad）即停。控制序列上算归一化总变差 TV |
| `RunResult` | 一次推演的全部数据：`arrived/t_arrival`、`solve_ok/solve_fail`、`min_clearance`、`tv/tv_per_tick`、`max_slack`、`alarm`、时序数组（`zs/us/t_ctrl/clearances`，供画图） |
| `RunResult.acceptance() -> dict` | 验收②③④的三条布尔判定（①由 `run_w2.sh` 保证） |
| `RunResult.acceptance_pass()` | 三条全过与否 |

常量：`ARRIVE_POS=0.25`、`ARRIVE_YAW=0.25`（指标定义口径）、`TV_PER_TICK_BOUND=0.30`（验收④界）。

### `mpc_proto/plotting.py` —— 诊断图

`plot_run(scenario, chassis, res, out_path)` 出单场景三合一 PNG（`out/<场景>.png`）：

1. **轨迹俯视**：参考折线（弱化灰虚线）+ 机器人轨迹（蓝）+ 障碍（静态青/动态黄，
   动态画 1 s 间隔拖尾，只画到推演结束）+ 起点/目标标注
2. **全程间隙时序**：外缘间隙（真值几何）+ 0 接触边界（状态红虚线）+ d_safe=0.20 参考线
3. **控制量**：线速度 v_x/v_y 与角速度 ω **分面**（单轴原则，不双轴）

配色用已校验默认色板分类槽位（固定顺序 蓝/青/黄），中文用 Noto Sans CJK。

### `mpc_proto/chassis.py` —— 底盘参数化描述

| 符号 | 作用 |
|---|---|
| `load_chassis(path) -> Chassis` | 读契约 schema v1 YAML；**`type: diff` 强制 `vy_max=0`**（迁移冒烟零改动的落点，算法不感知 type） |
| `Chassis` | `radius`（外接圆）、`v_min/v_max/vy_max/omega_max/acc_max/alpha_max` |
| `Chassis.u_min()/u_max()` | QP 控制 box 约束的上下界向量 |
| `Chassis.du_max(dt)` | 增量约束界：`acc_max·dt / alpha_max·dt` |

### `chassis/*.yaml` —— 底盘配置

与 docs/接口契约.md「底盘参数化描述」同 schema：`type`、`footprint.radius`、`limits` 六项。
`omni.yaml` 是验证底盘；`diff.yaml` 把 `vy_max` 置 0，同一算法直接跑（迁移冒烟预演）。

## 口径备忘

- 控制步长 dt = 0.05 s（契约 20 Hz）；仿真积分 dt_sim = 0.01 s（零阶保持）。
- 参考几何 = 场景 waypoints（NavFn 全局路径的 W2 替身）：静态绕行的几何由参考路径
  承担；MPC 负责跟踪 + 障碍软约束余量 + 动态避让（VO 管道 + 匀速直线预测）。
- 直参考穿障不采用：q_long ≤ 0.1·q_lat 下横向代价压过沿程收益（宁可迟到不可贴障），
  静态绕行几何归全局规划——与 ADR-0001 分工一致。
- OSQP 失败 → 零速 + 日志，连续 5 拍报警，不切第二控制器。
