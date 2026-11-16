# 9-1 / 9-2 / 9-3 交付包（工程侧）

> 本目录是 **上传 GitHub 用的最小集**：只含主要代码 + 必要数据 + 成绩汇总，
> 另附本说明。
> 采集时间：2026-11-15　　对应台账：`9-1工程执行台账.md`（在本仓库 `docs/` 下，未复制进本包）

---

## 0. 三张图看懂这个包

| 任务 | 本包提供的 |
|---|---|
| **9-1 基线调参方案**（算法组主导） | 三基线参数 YAML + 共享 costmap/规划器配置 + 平面限速层 + 安全层 + 基线启动 launch |
| **9-2 基线成绩入库**（工程组主导） | 采集/裁判/汇总/驱动代码 + 批量采集与时限冻结脚本 + **300 run 正式成绩汇总** |
| **9-3 时限回填**（工程组执行、算法组审核） | 时限冻结执行记录 + 结论（`instances.yaml` 0 行变更） |

---

## 1. 目录结构

```
code/
├── robot_bringup/                     # 9-1：基线参数与执行层
│   ├── config/nav2_common.yaml        # 共享 costmap / Smac 规划器 / footprint / 限速 / 安全层参数
│   ├── config/baselines/dwa.yaml      # DWA 基线（dwb_core::DWBLocalPlanner）
│   ├── config/baselines/mppi.yaml     # MPPI 基线（nav2_mppi_controller）
│   ├── config/baselines/teb.yaml      # TEB 基线（官方 teb_local_planner humble-devel）
│   ├── scripts/flat_speed_limiter.py  # 平面合速度限速（√(vx²+vy²)≤0.6、|wz|≤1.5、加速度限幅）
│   ├── scripts/safety_supervisor.py   # 安全层（/safety/stop、odom 看门狗、碰撞临近制动）
│   ├── launch/nav2_baseline.launch.py # 基线栈（支持多控制器同栈：controllers:=dwa,mppi,teb）
│   ├── launch/simulation.launch.py    # 一条命令：Isaac + TF/真值 + Nav2 + 限速
│   ├── launch/truth_tf.launch.py      # TF 树与真值里程计
│   └── CMakeLists.txt, package.xml
├── robot_benchmark/                   # 9-2：裁判 / 采集 / 汇总 / 场景生成
│   ├── robot_benchmark/judge.py       # 七项指标 + 公共判定（path frame，时间相对 run 起点）
│   ├── robot_benchmark/runner.py      # 采集入口（backend=nav2 / replay），落 run.json + trajectory.npz
│   ├── robot_benchmark/nav2_driver.py # 真实闭环驱动（单栈校验/复位/清图/规划重规划/场景身份校验）
│   ├── robot_benchmark/scenes.py      # 按清单生成场景 .usda + Nav2 地图 .pgm/.yaml + 静态占据栅格
│   ├── robot_benchmark/scenario_gen.py# 实例表读取/校验（契约 v1.4）
│   ├── robot_benchmark/summarize.py   # 每基线汇总表 + 异常清单
│   ├── robot_benchmark/gen_scenarios.py
│   └── test/                          # 40 条单测（judge / 场景 / 静态图回归）
├── robot_sim/isaac_sim_bridge.py      # Isaac 桥接：/clock /lidar/points /imu /odom /sim/ground_truth
│                                      #   + /sim/reset（每个 run 复位）+ /sim/episode_start（行人相位锚点）
└── scripts/
    ├── run_workset.py                 # 批量采集（一场景一次启动、三基线共用、基础设施失败自动重试）
    ├── freeze_time_limits.py          # 9-3：按公式算时限 + 回填 instances.yaml + 出执行记录
    ├── build_teb_ascii.sh             # TEB 官方源码在 ASCII 路径下的构建脚本
    └── build_mpc_ascii.sh             # mpc_controller 在 ASCII 路径下的构建脚本（MPC 本轮未评测）

data/
└── instances.yaml                     # 评分清单（100 实例：5 场景 × 20 种子，含冻结的 time_limit_s）

results/
├── 9-1参数定稿_2026-11-15.md           # 参数快照（sha256）、版本、采集口径
├── 9-2正式成绩_2026-11-15.md           # 300 run 正式成绩、失败构成、9-3 复核
├── 9-3时限冻结_执行记录.md             # 公式、逐场景结果、失败构成、给算法侧的审阅要点
└── summary_{dwa,mppi,teb}.{md,json}   # 每基线 20 seed 汇总表 + 异常清单（失败 run 逐条）
```

**没有包含**（按"只上传主要代码"处理）：

- 原始采集证据 `out/9-2_official/`（300 × `run.json` + `trajectory.npz`，共约 105 MB）——
  需要复核时按第 4 节命令重新采集即可；
- 构建产物（`build/ install/ log/`）、`__pycache__`、Isaac/ROS 环境内文件；
- 团队仓库里原有的其余文件（本包只是 9-1~9-3 相关部分）。

---

## 2. 正式成绩（20 种子）

| 场景 | DWA | MPPI | TEB |
|---|---|---|---|
| static_avoid | 80% | **95%** | 80% |
| dynamic_pedestrian | 40% | 60% | 40% |
| narrow_passage | 70% | 85% | 85% |
| dead_end | 90% | 90% | 90% |
| mixed | 55% | 55% | 55% |
| **合计（100 run/基线）** | **67.0%** | **77.0%** | **70.0%** |

- 全体 **214/300 = 71.3%**；套件级 `dynamic_stress`（dynamic_pedestrian + mixed）= **50.8%**
- 300/300 全部采集成功，**基础设施失败 0**，逐条终检 0 条不合规
- 失败构成：行人 `collision` 65 次（全部在含行人的两个场景，静态/墙类场景零碰撞）+
  `other`（progress_checker 判无进展）21 次（窄走廊停滞为主）

**9-3 结论**：`T_s = max(T_min , 1.5·max_j t̃)` 全部由下限决定 →
`static_avoid / dynamic_pedestrian / narrow_passage / dead_end = 60 s`、`mixed = 75 s`，
**`instances.yaml` 0 行变更**；用 20 种子正式数据交叉复核一致。

---

## 3. 复现步骤

环境：Isaac Sim 5.1（conda，Python 3.11）+ ROS 2 Humble（系统，Python 3.10），
`rmw_fastrtps_cpp`，`ROS_DOMAIN_ID=42`，单卡 RTX 5070 Laptop，运动学 `kinematic`。

```bash
# 0) 放进团队仓库后（本包 code/ 覆盖到 nav_ws_core/src/ 与 nav_ws_core/scripts/）
cd <repo>/nav_ws_core && source scripts/env.sh
export ISAAC_PYTHON=$HOME/miniconda3/envs/env_isaaclab/bin/python3

# 1) 单测（不需要仿真）：40 条
cd src/robot_benchmark && python3 -m pytest test -q

# 2) 构建依赖（两个包因"工作区路径含中文"必须构建到 ASCII 工作区；TEB 另需 libg2o）
source ~/teb_prebuilt/env.sh && source ~/teb_build_ws/install/setup.bash   # 若已用脚本构建过

# 3) 按清单生成场景 + 采集（一场景一次启动、三基线共用）
cd <repo>/nav_ws_core
python3 scripts/run_workset.py --out out/9-2_official --baselines dwa,mppi,teb \
    --runs 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16 17 18 19 20 --retries 2

# 4) 每基线汇总（生成 summary.md 与异常清单）
cd src/robot_benchmark
python3 -m robot_benchmark.summarize --runs-root ../../../out/9-2_official/dwa \
    --out ../../../out/9-2_official/dwa/summary.md --json ../../../out/9-2_official/dwa/summary.json

# 5) 9-3 时限冻结（算 + 回填 + 出记录）
python3 scripts/freeze_time_limits.py --workset <工作集目录> --write \
    --report docs/9-3时限冻结_执行记录.md
```

---

## 4. 两条必须知道的环境限制（否则无法构建）

1. **工作区路径不能含中文**。本机原路径含 `桌面`，rosidl 生成接口时会把中文前缀截断，
   报 `list index: 1 out of range` / `Target dependency '/Chassis-…/ObstacleState.idl' does not exist`。
   符号链接无效（colcon 会 realpath 回中文路径）。
   → `teb_local_planner` / `mpc_controller` 用 `scripts/build_*_ascii.sh` 构建到 `~/teb_build_ws`、`~/mpc_build_ws`。
   **根治建议：整个工作区放到 ASCII 路径。**
2. **`libg2o` 只有 apt 包且 sudo 需要密码** → 用 `apt-get download` + `dpkg -x` 解包到 `~/teb_prebuilt`
   （无需 root），脚本已自动化。启动 Nav2 前必须 `source ~/teb_prebuilt/env.sh`。

---

## 5. 口径与纪律（判读成绩时必须遵守）

1. **工作集（前 5 种子）与正式集（20 种子）重合**，这是 9-1/9-2 既定设计；
   正式成绩一律以本次重新采集的 `run.json` 为准，**不引用调参期任何数字**。
2. **失败 run 一律如实保留**（`metrics.success=false` + `failure_class`），不做剔除、不重跑刷分。
3. **共享层（costmap / 规划器 / 限速 / 安全层）改动后必须停调参、三基线重跑**。
4. `T_s` 对四个控制器完全相同；**MPC 后续接入沿用同一 `T_s`**，不得单独调时限。
5. 本包参数已定稿（见 `results/9-1参数定稿_2026-11-15.md` 的 sha256）；
   任何一项变动都要停采、重算 9-3、重跑受影响成绩。

---

## 6. 已知缺口（如实记录，未解决）

| 项 | 现状 | 说明 |
|---|---|---|
| 行人碰撞 | 动态场景成功率 40~60%，穿透最深 −0.30 m | 三基线不预测行人轨迹、只能反应式避让；按决定**不处理**（它们是自研 MPC 的参照系，口径一致即可） |
| 窄走廊停滞 | `other` 21 次，`narrow_passage` 最多 | 判断为 DWB/MPPI/TEB 在 1.46 m 净宽 + 全向底盘下的固有局限；按 9-1 §4.1 记录缺口、不再在固定预算内调参 |
| `narrow_passage` 急停 | DWA 3、MPPI 2、TEB 3（目标 ≤1） | 安全层 `contact_imminent` 触发（seed 306/311/314），是"真的救过场"，判读时需与控制器失败区分 |
| `ablation` 字段 | 三基线写 `{"note": ...}` | 契约要求结构化字典，待算法侧给结构（MPC 接入时才真正需要） |
| MPC | 未纳入本次成绩 | 仓库里的 `mpc_controller` 仍是 D2 直通跟随器；待 W6 版本 + `/mpc/obstacles` 真实数据源 |
| Git 提交 | 未提交 | 本仓当前不是 git 仓库，需提交者按 `提交指南.md` 收口 |
