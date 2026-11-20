# nav_ws_core 源码同步说明（2026-11-20，v3 道口栏杆版 + W6 接驳）

本包是**可直接 `colcon build` 的完整工作树**（`robot_bringup` / `robot_benchmark` /
`robot_sim` / `robot_description` / `mpc_controller` 五个包 + 场景模板 + 工具脚本 +
仿真资产），用于替代**只含源码子集**的交付包 `code/`。
上传包里 `code/` 只是**给评审看的抽样**，缺 `setup.py` / `resource/` / 构建元数据，
**不能**直接合入工作树编译——请以本包为准。

## 0. 本版（2026-11-20）相对 11-18 版的增量

针对算法侧 2026-11-20 答复里的 G1–G7 缺口，本版新增/修改：

| 缺口 | 落点 | 内容 |
|---|---|---|
| **G2 QoS 定案** | `scripts/obstacle_feeder.py` | `/mpc/obstacles` 统一 **reliable, depth 10**（与插件 `rclcpp::QoS(10)` 对齐，**插件不用改**）；`--qos best_effort` 可选 |
| **G3 承载范围** | `scripts/obstacle_feeder.py` | **只发栏杆矩形（圆链近似，速度 0）**；静态圆/墙体/行人本体**永不进消息**；id = `100*band_index + k`，带子存在期间稳定 |
| **G4 feeder + 存活校验** | `scripts/obstacle_feeder.py` + `scripts/run_workset.py` | feeder 20 Hz、`frame=odom`、失效即移除（不留 ghost）；`--mpc-obstacles feeder` 时由 workset 拉起并校验存活（提前退出 → 该场景 mpc 判脏） |
| **G5 mock 心跳开关** | `launch/nav2_baseline.launch.py` | 新增 `mock_obstacles:=true|false`（默认 true=开发；feeder 在场时必须 false）；`nav2_driver` 在 mpc 时校验 `/mpc/obstacles` **恰好 1 个发布者** |
| **G6 未膨胀栅格** | `scripts/static_local_grid.py`（`robot_bringup` 节点，已进 CMake `install(PROGRAMS)`） | 新增 `/debug/static_local_grid`：局部 6×6 m @0.05 的**纯静态**栅格，**无 inflation、无道口栏杆 band**，10 Hz，`frame=map`，与局部 costmap 同几何 |
| **G7 solver_fail** | `runner.py`（judge.py **保持冻结不改**） | 读 `diag.mpc_solver_fail_streak_max`，连续 ≥5 拍且 run 最终未成功 → `failure_class=solver_fail`；优先级 `collision > timeout > solver_fail > other` |
| **E-P0-2 参数段** | `config/baselines/mpc.yaml` | `FollowPath.mpc.*`：`N=40, dt=0.05, v_ref=0.6, d_safe=0.30, n_sqp=2, eps=1e-3, max_iter=10000, slack/terminal=true, q_long=0.4, q_lat=4.0, static_field_source=local_costmap` |
| **E-P0-4/6 采集** | `runner.py` + `nav2_driver.py` + `run_workset.py` | `--baseline mpc` 可用（launch 控制器 id=`mpc`，落盘 `controller=mpc_cpp`）；订阅 `/mpc/diagnostics` → `run.json.qp_timing={solves,mean_ms,p95_ms,max_ms}`；`ablation` 写 A 口径快照 `{vo:false, esdf:true, slack:true, terminal:true, N:40, q_ratio:0.1}` |

### 0.1 顺手修掉的一个"包里跑不起来"问题

交付包**不含 `docs/`**，而实例表（100 条冻结实例）原先只按 `<repo>/docs/scenarios/instances.yaml`
查找 → 干净解压后 `pytest` 与采集都会 `FileNotFoundError`。现在
`scenario_gen.default_manifest_path()` 的查找顺序改为：

1. `$NAV_WS/scenarios/instances.yaml`（工作树规范位置，包里就有）
2. `<repo>/docs/scenarios/instances.yaml`（文档镜像）
3. 安装后的 `share/robot_benchmark/scenarios/instances.yaml`

`test/test_scenario_gen.py` 与 `setup.py` 同步改为走同一入口。实测（干净解压 + `colcon build`）
`pytest -q` → **40 passed**（算法侧预期的 38 是 9-3 之前的口径）。

## 1. 内容

```
nav_ws_core/
├── src/robot_bringup/       共享执行层：Nav2 launch、代价地图/安全层配置、四份控制器参数、
│                            道口栏杆层 ped_corridor_layer.cpp(+plugins.xml/CMakeLists)、
│                            三级速度链（flat_speed_limiter / safety_supervisor）、
│                            static_local_grid.py（/debug/static_local_grid）
├── src/robot_benchmark/     场景生成(scenes/scenario_gen/gen_scenarios)、判分(judge)、
│                            采集(nav2_driver/runner)、汇总(summarize)
├── src/robot_sim/           Isaac 桥(isaac_sim_bridge)、真值 odom、map→odom 静态 TF、场景导出
├── src/robot_description/   机器人 xacro/URDF/meshes
├── src/mpc_controller/      W5 骨架（ObstacleState(.Array) + MpcDiagnostics + 直通插件）
├── scenarios/               5 个场景模板 + instances.yaml（100 条冻结实例，含 9-3 时限）
├── scripts/                 run_workset / run_300 / run_mixed / demo_gui / dev_* /
│                            9-3 时限冻结 / MPC、TEB 的 ASCII 路径构建 / obstacle_feeder.py
├── config/ + assets/        RViz 配置 + 机器人 USD 与场景资产（92 KB）
├── SYNC.md                  本文件
└── SHA256SUMS               逐文件校验（`sha256sum -c SHA256SUMS`）
```

## 2. 构建

```bash
cd nav_ws_core
source scripts/env.sh                 # 系统 ROS 2 Humble（不要用 conda 的 python 跑 ROS）
export ISAAC_PYTHON=$HOME/miniconda3/envs/env_isaaclab/bin/python3   # Isaac Sim 5.1.0
colcon build && source install/setup.bash

# TEB（第三方，需单独编）：见 scripts/build_teb_ascii.sh
# mpc_controller（含 rosidl 消息）：工作区路径**含中文**时必须走 scripts/build_mpc_ascii.sh
#   bash scripts/build_mpc_ascii.sh && source ~/mpc_build_ws/install/setup.bash
```

自测基线（本包实测）：`cd src/robot_benchmark && python3 -m pytest -q` → **40 passed**。

## 3. W6 起栈命令（A 栅格口径）

```bash
# 1) 空场景冒烟（launch 自带空数组心跳）
ros2 launch robot_bringup nav2_baseline.launch.py baseline:=mpc mock_obstacles:=true

# 2) 正式 A 口径（feeder 发真实栏杆矩形 + 纯静态栅格给 EDT）
ros2 launch robot_bringup nav2_baseline.launch.py baseline:=mpc \
    mock_obstacles:=false static_local_grid:=true
python3 scripts/obstacle_feeder.py --mode gate --qos reliable --rate 20 --frame odom

# 3) 批量（自动起 feeder、关 mock、校验 /mpc/obstacles 只有 1 个发布者）
python3 scripts/run_workset.py --out out/w6_mpc --scenarios static_avoid mixed \
    --runs 1 2 3 4 5 --baselines mpc --mpc-obstacles feeder
```

## 4. 冻结点（与正式 300 run 完全一致，采集后复核 12/12 未漂移）

| 文件 | sha256 |
|---|---|
| `src/robot_bringup/config/baselines/dwa.yaml` | `f5afde6f754dd8e6dc0341f1086871a5a277eedaa18d694c0dc3364a0b0f07c1` |
| `src/robot_bringup/config/baselines/mppi.yaml` | `6b3ff7f58d9a1d653ff7e6dbb3e8e6cbd034cee61422f970de7e698ea6ddf77f` |
| `src/robot_bringup/config/baselines/teb.yaml` | `819d28097ae0e5b69560d2ea28f6cc255dd74005c00bb73a8b604ed0bfb008e1` |
| `src/robot_benchmark/robot_benchmark/judge.py` | `9cf63a257f00fbc53d840f5cc6ec94171ddecc15ced2b19b6eb64a9b296a070c` |
| `scenarios/instances.yaml` | `0edf6029bfae4b4f6ec6a322063e3b1e589e6e4b665a27bac63cb2ad76f9fa2c` |
| `libteb_local_planner.so`（第三方，不在本包内） | `a10be1f00f4a19e48f690c0504a587b59d0123cb47a86fb7bb13a3c4133e82d5` |

**W6 接驳改动后的新哈希**（这三份 11-18 后动过，正式 300 run 用的是 11-18 版）：

| 文件 | sha256（2026-11-20） |
|---|---|
| `src/robot_bringup/config/baselines/mpc.yaml` | `5cb876893158e00e76d92cbc155df8b0e7010f9e618f64017525ebed13341ac4` |
| `src/robot_bringup/launch/nav2_baseline.launch.py` | `975c9c786bb0bd8915196eaab4ddb0924876929615f8e82587971c3fbb3178fc` |
| `src/robot_benchmark/robot_benchmark/nav2_driver.py` | `43ca6d891c6cee941dc7cc5c767259fe45d72043098561728f493838f2578b83` |
| `src/robot_benchmark/robot_benchmark/runner.py` | `81d235bfe36a9a4249e38edb6e42d9e47ad37cff2785036a78579305cf3785a3` |
| `src/robot_bringup/scripts/static_local_grid.py` | `14b24d0938836998abf2b65e11f1f00611c65221c1fbb33818d20232d87d9309` |
| `scripts/obstacle_feeder.py` | `8caaad96dd7ecce55f3238ccf942995fd2922ab6056121d8cf9d41174d12a864` |
| `scripts/run_workset.py` | `2210d9a1166c9cf793a381d58eab28c66fee514ab19bc1e65f3ddfeae9985cdd` |

> 注：`nav2_common.yaml`、`ped_corridor_layer.cpp`、四份基线里的 dwa/mppi/teb 均**未改**，
> 与正式 300 run（258/300 = 86.0%）保持一致；本次改动只涉及 W6 接驳面。

## 5. 仍未闭合（需要算法侧提供）

1. **G9 `mpc_controller` 的 W6 代码**：本包内只有 W5 骨架（直通跟随 + 消息定义），
   请把 `feat/w6-mpc-integration@f78982a` 的整目录覆盖到 `src/mpc_controller/` 再按 §2 构建。
2. **A-P0-1 参数段核对**：`config/baselines/mpc.yaml` 的 `mpc.*` 是按答复文字写的，
   合入他们的 `nav2_mpc_params.yaml` 后请逐项 diff（不一致以他们那份为准）。
3. **G6 话题内容联调**：`/debug/static_local_grid` 已按"纯静态、无膨胀、无 band"实现，
   但 `static_field_source: local_costmap` 的读取路径在他们插件里，需要他们接一次并回报。
