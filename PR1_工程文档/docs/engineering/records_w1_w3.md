# W1–W3 工程执行记录

> 历史记录；路径已按团队仓库布局整理。发布状态与当前入口见 [README](README.md)。

## W1：W1 执行记录（E01 / E02）

按路线书第 14 节要求，每完成一个 E 编号记录：日期、代码提交、产出路径、执行步骤、验收证据、失败原因、下一步。

## E01 核对真实环境，锁定兼容版本

| 项目 | 内容 |
|---|---|
| 完成日期 | 2026-10-30 |
| 代码提交 | 无（本地目录，尚未建 Git 仓库；E03 建仓后回填 commit） |
| 执行机器 | `robot`（Ubuntu 22.04.5，Ryzen 9 8945HX，30 GiB RAM，RTX 5070 Laptop 8 GiB） |
| 产出路径 | `docs/engineering/environment.md`；证据 `results/w1/e01/` |
| 执行步骤 | 1) `tools/engineering/e01_env_inventory.sh` 采清单；2) `tools/engineering/e01_run_sim_startup.sh` headless 启动空场景并启用 `isaacsim.ros2.bridge`；3) 读取 GUI 方式启动的 Kit 日志摘录关键行；4) 比对并冻结版本 |
| 验收证据 | `results/w1/e01/env_inventory.txt`（428 行命令输出）；`results/w1/e01/sim_startup_stdout.log`（含 `bridge_enabled_after=True`、`bridge_ext_version=4.12.4`、`bridge_ogn_node_count=30`）；`results/w1/e01/sim_startup_stdout_no_ros_source.log`（不 source ROS 2 时的失败对照）；`results/w1/e01/gui_startup_evidence.txt`（App 5.1.0 / Kit 107.3.3 / GPU / `Isaac Sim Full App is loaded`） |
| 失败与排查 | 1) 首次探针剥离了 ROS 环境，Bridge 报 `RMW was not loaded / AMENT_PREFIX_PATH is not set`；确认必须 source 系统 ROS 2 后才能启用桥接（已写入 `environment.md` 第 6 节，失败日志保留为回归对照）；2) 桌面快捷方式指向不存在的 `isaac-sim.sh`，改用 `bin/isaacsim`；3) 探针里两处 Kit 内部 API 名称随版本变化（`get_extension_dict`、`get_enabled_extension_id_list`），改为读磁盘 `extension.toml`，避免再次因 API 差异中断 |
| 验收判定 | 通过：重启后可按第 9 节步骤再次启动；`ros2` 命令可用；版本组合已冻结 |
| 下一步 | E02 双向通信与仿真时间；E03 建仓 |

## E02 验证桥接和仿真时间

| 项目 | 内容 |
|---|---|
| 完成日期 | 2026-10-30 |
| 代码提交 | 无（同上） |
| 产出路径 | `docs/engineering/bridge_check.md`；证据 `results/w1/e02/run_monotonic/`、`results/w1/e02/run_reset/`、`results/w1/e02/use_sim_time_probe.txt` |
| 执行步骤 | 1) `tools/engineering/e02_run_all.sh ... false 45`：仿真侧 headless + 动作图发 `/clock` `/tf`、订阅 `/w1/ext_to_sim`；外部侧收 CLI 证据并记录 45 s；2) 同一脚本换 `--reset-on-stop true` 再跑一遍；3) `tools/engineering/e02_use_sim_time_probe.py` 做 true/false 对照；4) `tools/engineering/e02_analyze.py` 汇总 |
| 验收证据 | 双向各成功（`/clock` 发出 1382/1375 条被外部收到；外部 149 条 Twist 中 123/124 条序号被仿真收到）；`/clock` 实测 59.9 Hz、RTF 0.997；暂停 5.02 s 时钟冻结于 26.400 s；重置后 `resetOnStop=false` 单调、`true` 回跳 −32.8 s；`use_sim_time=True` 节点时间与 `/clock` 中位偏差 0.000 s |
| 失败与排查 | 1) 外部记录器比仿真晚结束，导致“发送数≠接收数”；改为按 `wall_epoch` 只统计仿真在线窗口，并把成因（暂停/停止/退出后发送）写入文档；2) 分析脚本最初用“时间必须递增”判定 `use_sim_time`，在回跳场景误报 FAIL；改为与同刻 `/clock` 逐点比对后两次运行均 PASS |
| 验收判定 | 通过：双向各成功一次；暂停/恢复/重置的时间行为均可解释；仿真时间与墙上时间已明确区分 |
| 下一步 | E03 建仓；E04 接口契约 v1（把本文件第 8 节的约束写进去） |

## E03 建立 Ubuntu 工作空间和仓库

| 项目 | 内容 |
|---|---|
| 完成日期 | 2026-10-30 |
| 代码提交 | 分支 `feat/w1-e03-workspace`；提交哈希见文末“提交记录” |
| 产出路径 | `src/`（7 个新包 + 1 个既有包）、`assets/`、`tools/engineering/`、`docs/engineering/package_roles.md`、`docs/engineering/assets.md`、`docs/engineering/dependencies.md`、`docs/adr/0007-engineering-package-layout.md`、`CONTRIBUTING.md`、`.gitignore` |
| 执行步骤 | 1) 核对 2026-10-23 既有包与资产，决定“纳入而非重写”；2) 建七包骨架（每个包 README 写明状态与未实现项）；3) 资产纳入仓库并生成 `MANIFEST.sha256`；4) 写构建/依赖/验收脚本；5) `git init` + 分支流程；6) 跑验收 |
| 验收证据 | `./tools/engineering/build_workspace.sh` → `Summary: 8 packages finished`；`ros2 pkg list` 可见 8 个包；`./tools/engineering/acceptance_check.sh` → `PASS=9 FAIL=0`；`./tools/engineering/verify_assets.sh` → 20 个文件校验一致 |
| 失败与排查 | 1) `colcon build` 报 `ModuleNotFoundError: No module named 'catkin_pkg'`——根因是 PATH 里的 conda base Python 3.13 被 `ament_package_xml` 调用；解决：`tools/engineering/build_workspace.sh` 把 PATH 收敛到系统路径，并写入 `docs/engineering/dependencies.md`；2) 新包 `package.xml` 用了 `engineering@todo` 被判 `Invalid email`，整个包不被 colcon 识别；改为 `member3@example.com`；3) CSV 证据含 CRLF 会被 Git 归一化改写字节，`.gitattributes` 里对 `*.csv` 关闭文本处理；4) 合并到 `main` 后复跑验收出现“随机某个包缺失”：根因是验收脚本里 `ros2 pkg list | grep -qx` 在 `set -o pipefail` 下让 grep 早退触发 SIGPIPE（退出码 141）被判失败；改为先把包列表落盘再比对，`tools/engineering/e02_run_all.sh` 里同样的写法一并修正，修完连续三次 `PASS=9 FAIL=0` |
| 验收判定 | 通过（可获取、可构建、可加载）：算法成员按 README 能获取同一目录并构建 |
| 遗留 | **远端托管与 `main` 保护规则未配置**（无托管账号）；本地 Git 的分支规范只是约定，不是技术强制 |
| 下一步 | E04 接口契约 v1；W2 起按职责表迁移既有能力 |

## E04 与算法成员签定接口契约 v1

| 项目 | 内容 |
|---|---|
| 完成日期 | 2026-10-30（**草案完成，互审未完成**） |
| 代码提交 | 同上分支；提交哈希见文末“提交记录” |
| 产出路径 | `docs/接口契约.md`（v1.0-draft）、`docs/指标定义.md`（指标草案 v0）、`src/robot_benchmark/robot_benchmark/contract.py`（可 import 的数据契约） |
| 执行步骤 | 1) 从既有实现里抽取真实话题/帧/参数（`command_bridge`、`state_bridge`、`truth_localization`、`nav2.yaml`）；2) 结合 E01/E02 实测确定时间、QoS、重置语义；3) 写话题总表、运动符号、速度出口、TF 责任表、场景/目标/控制器切换接口、`benchmark.run` 语义；4) 把契约里可执行的部分落成 `robot_benchmark.contract` 与七项指标键；5) 列 9 项待确认问题与签认区 |
| 验收证据 | `python3 -c "import robot_benchmark ..."` 通过（七项指标键 + `Scenario`/`RunResult` 序列化）；`python3 -m robot_benchmark.run ...` 退出码 2 并明确报未实现（不伪造结果）；`acceptance_check.sh` 检查契约含“待确认/签认区”均 PASS |
| 失败与排查 | 1) 契约最初想直接抄 `nav2.yaml` 的参数，核对后发现既有配置未经验证——改为“只记录来源 + 标注待复核”，避免把未验证值当基准；2) 镜像里没有 IMU 帧（资产只有 `base_link`/`livox_frame`），契约里显式写 `imu_link` 待建，不留隐性假设 |
| 验收判定 | **部分完成**：文档与可执行契约已交付；M0 要求的“两人互审 + 记录确认日期/待办/责任人”尚未闭环，第 11 节签认区为空 |
| 遗留 | 第 10 节 9 项待确认（速度符号、限速与加速度、目标容差与超时、重置时钟口径、点云 QoS、安全层锁存、指标公式、真值裁判、是否允许热切换）；指标公式由算法侧 W3 前定稿 |
| 下一步 | 与算法成员逐项确认 → 契约升 v1.0-final → M0 关闭；W2 起按契约实现 E05/E06 |

## 未完成 / 需要注意

- **E03 遗留**：远端仓库与 `main` 保护规则未配置（无托管账号，保护目前只是文字约定）；
- **E04 遗留**：契约 v1.0-draft 未签认，M0 未关闭；指标公式仍是草案；
- 本轮所有桥接结果都是**空场景 + 测试用话题**（`/w1/ext_to_sim`），尚不代表底盘、传感器或导航链路可用；
- 各新包都是“骨架 + 明确未实现”，不要因为 `colcon build` 通过就认为功能已完成；
- 每次运行 Isaac Sim 前确认没有第二个实例占用显存（8 GiB 是硬约束）。

## 提交记录

| 提交 | 分支 | 说明 |
|---|---|---|
| `7c3e0e5` | `main` | E01/E02：环境核对与桥接验证的脚本与证据 |
| `763d114` | `feat/w1-e03-workspace` | E03：七包骨架、资产分发、仓库规范与验收脚本 |
| `014ba42` | `feat/w1-e03-workspace` | E04：接口契约 v1.0-draft 与七项指标草案 |
| `8dba50a` | `main`（合并提交） | `git merge --no-ff`，一次验收对应一条合并提交 |
| 本文档最后更新的提交 | `main` | 修复验收脚本 SIGPIPE 误判 + 回填本表 + 补 `results/w1/e03_e04/` 证据 |

复现提交历史：`git log --graph --oneline --decorate`（快照见 `results/w1/e03_e04/git_history.txt`）。


---

## W2：W2 执行记录（E05 起）

按路线书第 14 节要求记录：日期、代码提交、产出路径、执行步骤、验收证据、失败原因、下一步。
本文件记录 W2（2026-11-02 ~ 11-08）的实际执行情况；W1 见 `docs/engineering/records_w1_w3.md`。

## E05 建立最小底盘模型和简单场景

**状态：最小底盘模型已完成并验证。**

> **E05 最终产出物（2026-10-31 确定）：`assets/scenes/e05_min_scene.usd`**
> —— 可直接用 Isaac Sim 打开并 Play 的场景（底盘 + 地面 + 墙 + 箱体 + 灯光）。
> USB/xacro 源：`src/robot_description/urdf/agl_chs_03_f01.urdf.xacro`；
> 验收证据：`results/w2/e05/`。

| 项目 | 内容 |
|---|---|
| 完成日期 | 2026-10-31（提前于 W2 起始日 11-02） |
| 底盘型号 | AGL-CHS-03-F01（桌面 PDF 规格书 V1.2，2026-08-22） |
| 产出路径 | `src/robot_description/urdf/agl_chs_03_f01.urdf.xacro`、`src/robot_description/meshes/agl_chassis_octagon.stl`（+`tools/engineering/gen_chassis_mesh.py`）、`src/robot_description/config/robot_geometry.yaml`、`docs/engineering/robot_model.md`、`assets/robot_a_chassis_agl_chs03_f01.usd`（+`assets/configuration/*.usd`）、`tools/engineering/e05_import_check.py` |
| 参考模型留档 | `assets/reference/robot_a_description/`（上一代整机 URDF + 37 个 STL） |

### 执行步骤

1. 读 PDF 规格书，逐项抄出底盘参数（尺寸/质量/离地/轮组/速度/传感器）；
2. 在本机查找“原本的 URDF”：VSCode 本地历史 → 原始路径 `<your-ws>/robot_ws/robot_a_description`（已删除）
   → 微信文件缓存里找到完整包（`urdf/robot_a.urdf` + 37 个 STL）；
3. 量 STL 包围盒与 URDF 关节原点，与 PDF 对照 → **判定不是同一套参数**（6 寸轮 vs 8 寸、122.48 kg vs 65 kg）；
4. **读 PDF 第 3 页三视图修正形状**（2026-10-31 补做）：
   俯视图显示车体是「旋转 45° 的方形 + 四角截平」的八边形，四个 45° 斜面各装一个全向轮，
   轮轴水平 45°；前视图给出离地总高 251、车体高 226。据此改为：
   八边形棱柱网格车体（`gen_chassis_mesh.py` 生成）+ 四角 45° 全向轮；
5. 澄清 726 的语义：图纸 726 尺寸线与轮外缘相切，量的是「跨轮、垂直 45° 斜面」距离，
   不是 X/Y 外接方盒 → 轮心 |x|=|y| = 235.46 mm、外接包络 662 mm；
6. 按 PDF 建立参数化 xacro（网格车体 + 解析惯量，复用上一代的坐标约定与关节命名）；
5. `xacro` + `check_urdf` 校验；
6. Isaac Sim 5.1 headless 导入 → 静态 2 s 采样 → 落体试验 → 导出 USD → 回读校验。

### 验收证据（`results/w2/e05/`）

| 检查 | 结果 |
|---|---|
| 导入成功、铰接根 `/robot_a/base_footprint` | PASS |
| 跨轮距离 726.0 mm（图纸 726） | PASS |
| X/Y 外接包络 661.8 mm | PASS |
| 4 个旋转关节 / DOF=4 / body=9 | PASS |
| base_link 静止高度 0.1050 m（= 轮半径） | PASS |
| 底盘底面离地 0.0250 m（= PDF 25 mm） | PASS |
| 2 s 内水平漂移 0、z 抖动 0、倾角 0° | PASS |
| 落体（0.4023 m → 0.1050 m，全程无穿透） | PASS |
| 导出 USD 可回读（9/9 link、8 关节、铰接根正确） | PASS |

完整输出：`results/w2/e05/import_check.txt`；逐帧位姿：`results/w2/e05/settle_samples.csv`。

### 失败与排查（都记下来，避免重复踩）

1. **URDF 导入器不认 xacro**：必须先 `xacro` 展开成纯 URDF；脚本里现已自动展开并保留展开结果作为证据；
2. **`omni.kit.commands.execute("URDFCreateImportConfig")` 返回 `(status, config)`**，
   直接取 `[0]` 会拿到 `bool`，随后 `parse_urdf` 报 `incompatible function arguments`；
   正确写法是取 `[-1]`；
3. **只调用 `Articulation.initialize()` 会崩**（`simulation_view` 还是 `None`）：
   必须先用 `World.reset()` 建立物理视图，并且 `timeline.play()`，否则 `world.step()` 不推进物理；
4. **位姿要从 `base_link` 读，不能读铰接根**：`base_footprint` 是静态帧，读出来永远是 0；
   读取时用 `world.step(update_fabric=True)` 同步物理到 fabric；
5. 一次 `carb::cpp::bad_optional_access` 崩溃发生在 Isaac Sim 启动阶段（此前有多个实例反复启停），
   清掉 `/tmp/carb.*` 与残留实例后正常；以后连续跑批前先确认无残留实例；
6. **URDF 导入器不写 extents**，`UsdGeom.BBoxCache` 会返回空包围盒（FLT_MAX）——量包络要改用
   各轮连杆的世界位姿 + 已知轮径/轮宽解析计算；
7. **车体“消失”事故**：URDF 里车体是 mesh（`package://robot_description/meshes/...`），
   导入进程没有 source 工作空间 → 导入器只打一条 Warning
   （`Failed to resolve mesh 'package://...'`）就跳过网格，结果只剩基本体的轮子和悬空的雷达。
   修复：展开 xacro 时用 `mesh_dir:=file://<绝对路径>` 覆盖；并新增回归检查
   「车体网格已加载 + 网格实测 643 × 656 × 226 mm」；`e01_run_sim_startup.sh` 也会顺带 source
   `install/setup.bash`（若已构建），让 `package://` 路径同样可解析。
   另外 USD 里的网格是**实例代理**，`stage.Traverse()` 走不到，必须用 `TraverseAll()` 才能查到。
8. 我第一次按「726 = X/Y 外接方盒」建模，导致车体八边形切角过小、轮心偏外 78 mm；
   放大看图纸上那条 45° 细线与轮外缘**相切**后才发现语义不同（详见 `docs/engineering/robot_model.md` 3.2）。

### 追加：最小场景（2026-10-31）

按 E05 要求补了「地面 + 墙 + 单个箱体」的最小场景，产物 `assets/scenes/e05_min_scene.usd`
（生成脚本 `tools/engineering/make_min_scene.py`，渲染截图 `results/w2/e05/scene_preview.png`）。

回读校验：prim 60 个，`/World` 下含 `GroundPlane / Wall / Box / Sun / Dome / PhysicsScene / robot_a`；
铰接根 `/World/robot_a/base_footprint`、9 个关节、底盘 9 个 link 全部命中。

### 未完成（E05 剩余项）

- [ ] 整机成员确认「726 是跨轮距离还是外接方盒」（当前按图纸语义建模，见 `docs/engineering/robot_model.md` 5.2 冲突表）；
- [ ] 整机成员提供真实尺寸、轮心间距、轮宽、雷达/IMU 安装位姿、上半身固定姿态（7 项假设见 `docs/engineering/robot_model.md` 5.1）；
- [ ] 整机 footprint（当前只覆盖底盘 0.726 × 0.726）；
- [x] 最小场景（地面 + 墙 + 单个箱体）与截图 —— 已完成（`assets/scenes/e05_min_scene.usd`）；
- [ ] GUI 内目视检查与 RViz 中模型/安装关系一致性；
- [ ] 接触与摩擦参数（PhysicsMaterial，E06）；
- [ ] 同步既有 `robot_a_sim_control` 的轮径（0.0762→0.105）等默认值（E06）。

### 与算法侧需要确认的接口变化

- 帧名：`livox_frame` → `lidar_front_link` / `lidar_rear_link` / `imu_link`（契约待确认项 #10）；
- 限速口径：契约当前沿用上一代 0.25/0.20/0.60，而 PDF 底盘可到 0–2 m/s（待确认项 #11）。

## E06 实现全向速度输入

| 项目 | 内容 |
|---|---|
| 完成日期 | 2026-10-31 |
| 产出路径 | `tools/engineering/e06_omni_drive.py`（仿真侧驱动）、`tools/engineering/e06_motion_test.py`（外部指令）、`tools/engineering/e06_run_all.sh`（一键）、`tools/engineering/e06_analyze.py`（判定）、`docs/engineering/motion_check.md`、证据 `results/w2/e06/run_kinematic/`、`results/w2/e06/run_physics/` |
| 执行步骤 | 1) 按 4×45° 全向轮布局写逆解（`u_i=(sinφ,−cosφ)`），启动时做 IK→FK 自检；2) 仿真侧用 OG 节点订阅 `/cmd_vel` + 心跳，按 0.5 s 超时、限速后驱动轮子，并发布 `/odom`；3) 外部按仿真时间跑 10 阶段序列（前后/左右/正负旋转/组合/零速/停发）；4) 分析器用**阶段内机体系位移**判定方向与串扰 |
| 验收证据 | kinematic：**9/9 PASS** —— 前进 0.297 m、后退 −0.297 m、左移 +0.297 m、右移 −0.297 m、逆时针 +0.895 rad、顺时针 −0.895 rad、组合 (0.238, 0.226) m/+0.597 rad、零指令残余 0.0000 m/s、超时停发后最大速度 0.0000 m/s |
| 失败与排查 | 1) 两侧 `ROS_DOMAIN_ID` 不一致导致外部看不到 `/clock`（统一为 42）；2) `timeline.get_current_time()` 只回绕在 0~0.033 s，改用 `IsaacReadSimulationTime` 的输出作为时间基准；3) `ROS2PublishOdometry` 的 quaternion 属性必须传序列（IJKR），传 `Gf.Quatd` 报 `list of size 1`；4) URDF 导入的关节默认驱动不足以做速度控制，必须先 `switch_dof_control_mode("velocity")` + `set_gains(kps=0,kds=1e4)` + `set_max_efforts(80)` |
| 补充实验 | **接触驱动对照（physics 模式）4/9**：旋转能跟到 89%，但平移只有指令的 ~30% 且串扰与主轴同量级 —— 圆柱轮没有滚子，侧向被摩擦锁死。已在 `docs/engineering/motion_check.md` 明确标记 kinematic 是**早期接口模型**，不作为轮地接触动力学验证 |
| 下一步 | 与算法侧确认运动模型口径（kinematic 推进 vs 补滚子的接触驱动）、`ω` 上限与加速度限制；W3 起把该驱动并入 `robot_sim` 并由 E09 接管 TF/里程计 |

## E08 接入雷达和 IMU

| 项目 | 内容 |
|---|---|
| 完成日期 | 2026-11-01 |
| 产出路径 | `src/robot_sim/config/sensors.yaml`（传感器配置）、`docs/engineering/sensors.md`（说明与实测）、`tools/engineering/e08_sensor_pipeline.py`（仿真侧雷达+IMU+发布）、`tools/engineering/e08_ros_verify.py`（订阅端实测）、`tools/engineering/e08_bag_analyze.py`（bag 去重分析）、`tools/engineering/e08_make_figures.py`、`tools/engineering/e08_rviz_check.sh` + `tools/engineering/e08_tf_publish.sh` + `tools/engineering/e08_shoot_window.py`、`config/e08_rviz.rviz`、证据 `results/w2/e08/` |
| 执行步骤 | 1) 查证官方激光雷达资产：18 个 RTX USD / 47 个 JSON 里没有 Mid-360，且 `/Isaac/Sensors` 资产包不可用 → 用 PhysX 射线投射自建通用 3D 雷达（360×8、−7°~52°、40 m）；2) IMU 用 `isaacsim.sensors.physics.IMUSensor`（200 Hz、含重力、与雷达同位置）；3) 用 `OnCustomEvent` 边沿触发发布 `/points_raw`、`/imu`、`/clock`；4) 剧本让底盘原地转 90°/前进/反转/侧移，外部订阅端 + rosbag 双路实测频率与丢帧；5) RViz（俯视正交 + URDF TF 链）确认环境结构 |
| 验收证据 | 点云 **10.02 Hz**（178 条 / 17.77 s 仿真时间，重复发布 1.00×、缺口 0 帧）；IMU **60.11 Hz**（= 物理步率，标称 200 Hz，缺口 69.9% 属发布路径上限）；`/clock` 仿真步长中位与 max 都是 16.67 ms（**没有掉物理步**）；方向校验六个阶段全部满足 `方位角 ≈ −偏航 + 3.3°`，IMU `wz` 转正 +0.463 / 转负 −0.463；RViz `Global Status: Ok`（`results/w2/e08/rviz_points.png`） |
| 失败与排查 | 1) IMU 全 0：早期为保证位姿用"瞬移+关重力"（kinematic），物理传感器看不到动态 → 加 `--mode physics`（保留重力+轮子速度驱动），静止 az=9.810、转向 wz≈0.46；2) **发布节点接 `OnPlaybackTick` 会每帧重发当前值**，实测点云与 IMU 都变成 ~84 Hz，且时间戳每帧都变所以"按 stamp 去重"也识别不出；3) 换 `omni.graph.action.Branch` 电平门控仍 1.4× 重复（`app.update()` 一次可能跑 1~2 个 tick，电平被消费多次）→ 最终用 `OnCustomEvent` + carb 事件，Counter 验证 100 次事件=100 次发布；4) RViz 一开始无 `/tf` 导致 Fixed Frame 告警 → 补 `robot_state_publisher`(URDF) + 静态 `odom`；5) 无 ImageMagick/Xvfb 时截图：自己写 xwd 解析（注意 `bytes_per_line` 有 4 字节对齐填充） |
| 已知限制 | 2.7 m 内看不到地面点（−7° 最下沿射线落地距离）；8 条垂直束是规则栅格、非真实花瓣扫描；无噪声/反射率/运动畸变；只建了 1 台雷达（规格书 2 台对角）；IMU 发布到不了 200 Hz；physics 模式平移只到指令 ~30%（E06） |
| 下一步 | E04 契约 #10（`livox_frame` → 新帧名）在仿真侧已由 RViz + TF 验证，等算法侧确认下游配置；第二台雷达与超声留 E09/E10；`/points_raw` → costmap 的 `pointcloud_to_laserscan` 参数要在 E10/E11 里按"近场盲区"重设 |


---

## W3：W3 执行记录（E08–E10）

W3（2026-11-09 ~ 11-15）：建立完整的感知与状态管线。
E08（雷达 + IMU）的记录写在了 `docs/engineering/records_w1_w3.md`（当时按 W2 排期做完），本文件记录 E09、E10。

## E09 发布真值里程计，建立唯一 TF 树

| 项目 | 内容 |
|---|---|
| 完成日期 | 2026-11-02 |
| 产出路径 | `docs/engineering/tf_tree.md`（交付文档）、`src/robot_sim/config/tf_tree.yaml`、`src/robot_sim/robot_sim/nodes/{truth_odom_node.py,map_odom_static_node.py}`、`src/robot_bringup/launch/truth_tf.launch.py`、`src/robot_sim/robot_sim/scripts/isaac_sim_bridge.py`（仿真侧统一桥接）、`tools/engineering/e09_{unit_test_nodes.py,node_test.sh,verify.py,run_all.sh,sys_env.sh}`、证据 `results/w3/e09/` |
| 执行步骤 | 1) 定架构：仿真只发**原始真值** `/sim/ground_truth`（`world` 系、twist 世界系、不发 TF）；2) `truth_odom_node` 做世界系→机体系转换并发 `/odom` + 唯一一条 `odom→base_footprint`；3) `map_odom_static_node` 发静态 `map→odom`（真值模式 identity）；4) `robot_state_publisher` 按 URDF 发 `base_footprint→base_link→传感器`；5) 先写节点级单测（合成真值），再写集成验证（Isaac 真值 + TF + 订阅端核对） |
| 验收证据 | 节点单测：twist 转机体误差 **2.8e-17**、`/tf_static` 有 `map→odom`、TF 与 `/odom` 一致。集成（kinematic）：`/odom` **60.00 Hz**、无重复时间戳、前进 Δ=(+0.448, 0.000) m、左移 Δ=(0.000, +0.448) m、原地逆时针 Δyaw=+1.567 rad、静止漂移 0.00000 m/s、TF 全部外参与 URDF 一致、`base_footprint` 父帧唯一、重置后 0.2 s 内 `/odom` 与 TF 对齐且 `map→odom` 未变 |
| 失败与排查 | 1) 节点起不来：本机 conda base（Python 3.13）在 PATH 里，`ros2 run` 的 rclpy 直接崩 → 写 `tools/engineering/e09_sys_env.sh` 收敛 PATH（与 `build_workspace.sh` 同口径）；2) `ros2 topic pub --once` 在 DDS 发现完成前就发出，测试拿不到数据 → 改成 Python 以 20 Hz 持续发布；3) 残留节点（`ros2 run` 只是包装器，杀它不杀子进程）造成"两条 `/odom` 重复发布"→ 测试脚本按脚本名兜底清理；4) 重置瞬间（真值跳变）用位姿差分算出的 twist 会飙到 27 m/s → 适配节点把超限 twist 置 0；5) 验证脚本把仿真开始前的时间当成段边界，导致运动段判断错 → 改成**从数据自动识别运动段**；6) 物理模式下方向检查 FAIL，原因是圆柱轮无辊子（E06 已知）→ 方向类验收改用 kinematic 模式，物理模式留作诚实对照 |
| 已知限制 | 物理模式平移 ~30% 且方向被"打偏"45°；`odom→base_footprint`（不是 `base_link`，见 `docs/engineering/tf_tree.md` 第 2 节）；轮子无 joint_states |
| 下一步 | E10 把仿真、TF、RViz 收进一条命令；E12 接 `/joint_states`；E19 若启动 SLAM 支线，用 `truth_tf.launch.py truth:=false` 关掉真值 TF |

## E10 统一启动与 RViz 配置，验收 M1

| 项目 | 内容 |
|---|---|
| 完成日期 | 2026-11-02 |
| 产出路径 | `src/robot_bringup/launch/simulation.launch.py`（一条命令，`gui:=true` 可开 Isaac 窗口）、`config/e10_robot.rviz`、`src/robot_sim/robot_sim/scripts/{isaac_sim_bridge.py,e10_demo_driver.py}`、`tools/engineering/e10_{m1_check.py,run_all.sh}`、`tools/engineering/tile_windows.py`（左右平铺两窗口）、证据 `results/w3/e10/` |
| 执行步骤 | 1) 把 E06/E08/E09 三件事合进一个桥接脚本（`--cmd-vel/--sensors/--truth/--demo` 开关）；2) launch 一条命令起 Isaac(headless)+桥接+TF 树+RViz+演示指令源；3) `e10_m1_check.py` 订阅端核对 M1 四项；4) 抓 RViz 窗口截图 |
| 验收证据 | 一条命令：`ros2 launch robot_bringup simulation.launch.py use_sim_time:=true`（记录在 `results/w3/e10/launch_command.txt`）；`/odom` **1227 条 / 20.43 s = 60.00 Hz**，最大间隔 16.7 ms（无断流）；发 0.1 m/s 后机器人移动 **0.198 m**；`/points_raw` 205 帧、每帧 494~496 点、`frame_id=lidar_front_link`；`TF map→base_link` 可解析；RViz 截图 `results/w3/e10/rviz_e10.png`（Global Status: Ok，点云里地面环/墙/箱体清晰） |
| 失败与排查 | 1) 截图 `X_GetImage BadMatch`（抓 root 失败）→ 改成按窗口 id 抓（`xwd -id`）；2) launch 里 Isaac/RViz/截图用了不同 `DISPLAY`（`:0` vs `:1`）→ 脚本统一 `export DISPLAY`；3) 上一轮被杀进程留下 fastrtps 共享内存段，导致某一轮 `/odom`、`/points_raw`、`/clock` 全 0 条 → 每轮开跑前清 `/dev/shm/fastrtps_*`，并在检查脚本里先热身等 `/clock`；4) M1 检查脚本自身 bug（`30%` 被当成格式化占位符）；5) 清理 trap 曾把自己的父进程杀掉 → 排除 `$$`/`$PPID` |
| GUI 模式（2026-11-03 补） | `gui:=true` 打开 Isaac Sim 窗口，并在 Isaac 视口里把点云画成绿色点（`/World/LidarDebug/points`），可与 RViz 同时看；`tools/engineering/tile_windows.py` 自动把两个窗口左右平铺（本机没有 wmctrl/xdotool，用 libX11 的 XMoveResizeWindow）；证据 `results/w3/e10/{isaac_e10.png,rviz_e10_gui.png}` |
| 手动发指令不动（2026-11-03 收到反馈后修） | 两个原因：① 发指令的终端只 `source /opt/ros/humble/setup.bash`，`ROS_DOMAIN_ID` 与 launch（=42）不一致 → 完全收不到；② 默认 `demo:=true` 的内置演示指令源跑完序列后仍以 20 Hz 发零速，把手动指令盖掉 → 已改成**跑完发一次零速就退出**。诊断：发指令终端 `ros2 topic info /cmd_vel -v` 里 `Subscription count` 必须 ≥1（0 = 域不对） | 手动控制时建议 `demo:=false`；发指令终端用 `source tools/engineering/e09_sys_env.sh` |
| 平移指令"看起来不动"（2026-11-03 实测定位） | 用 `e10_motion_probe.py` 量化：**physics 模式**下 0.2 m/s 指令 4 s 只走 **0.252 m（43%）且方向偏 45°**（实测世界 Δ=(+0.252,+0.238)），旋转却有 87%；**kinematic 模式**下同样指令走 **0.740 m（93%），方向精确 +x**。所以"平移不动、旋转正常"是圆柱轮无辊子导致的物理限制（E06 已记录），不是链路问题。给用户的建议：要按指令走就用 `mode:=kinematic`；并且用探针看 `/odom` 位移而不是靠肉眼 |
| "Isaac Sim 闪退"（2026-11-03 澄清） | 日志里出现 `Simulation App Shutting Down` + `process has finished cleanly`，且**前面有完整的统计块**（点云/IMU 计数、最终位姿、射线耗时）= 跑到 `--duration` 后**正常收尾**，不是崩溃：本机 1199 帧点云 ≈ 120 s×10 Hz、7192 条 IMU ≈ 120 s×60 Hz，正好是默认 `duration:=120`。真崩溃的特征是 `process has died`、exit code ≠ 0（如 -9/-11）、或 `too many errors, simulation has stopped`。已加 `duration:=0 = 不限时长`，并在收尾时打印提示 |
| 点云话题改名（2026-11-04，对齐契约 v1.1） | `/points_raw` → **`/lidar/points`**：桥接话题参数、`sensors.yaml`、两套 RViz 配置、`e08_*`/`e09`/`e10` 脚本与文档全部切换；新增 `--cloud-alias` 可选兼容旧名。回归实测：`/lidar/points` 10 Hz（CLI 9.4 Hz）、206 帧/20.57 s、每帧 551 点、`frame_id=lidar_front_link`；E10 M1 全项 PASS |
| E14 五类场景（2026-11-04） | 新增 `robot_benchmark/scenario_gen.py`（确定性生成器 + 连通性/净空/可解性校验）、`gen_scenarios.py`（CLI，`--check` 可校验）、五份模板 `scenarios/*.yaml`、**5×20=100 个冻结实例** `scenarios/instances/*.yaml` + `scenarios/test_list.yaml`；种子规则 = 场景序×100+run 序（与算法侧一致）。校验与单测：`--check` 全通过、pytest 9 项通过 |
| 顺带修的坑 | headless 模式带 `DISPLAY` 时 Kit 会卡在 `carb.windowing-glfw.plugin` 初始化（实测卡 80 s 无进展）→ 桥接在 headless 时主动清 `DISPLAY`；`demo:=true` 的演示指令源跑完即退出（不再用零速盖手动指令） |
| 五类场景"可视化查看"（2026-11-04） | 新增 `scene_from_scenario.py`：按已冻结实例搭 Isaac 场景（地面/机器人/圆柱与墙体障碍/行人代理/起点目标标记/灯光，相机自动摆位），支持 `--gui` 开窗口、`--all --cycle N` 巡览、`--save --headless` 导出 `assets/scenes/scenarios/*_01.usd`；`--rich` 富化（按 E14 要素加障碍/行人，每加一组都重跑可达性校验，不可达即回退）。已导出 5 份富化 USD 并独立核对：static_avoid 17 圆柱、dynamic_pedestrian 6 圆柱+4 行人、narrow_passage 6 圆柱+2 墙（净宽 1.46 m）、dead_end 7 圆柱+3 墙（袋口 1.40×2.00 m）、mixed 15 圆柱+1 墙+3 行人（`results/w3/scenarios_usd_check.txt`）。行人按真值轨迹匀速横穿、到 stop_at_y 停住 |
| 已知限制 | RViz 里 `RobotModel` 仍显示 `Status: Error`（底盘 STL 没渲染出来，点云/TF/里程计轨迹正常）——列入 E11/E12 处理；physics 模式下 0.1 m/s 指令实测速度峰值 0.048 m/s（= 指令的 ~48%），要精确运动验证用 `mode:=kinematic` |
| 下一步 | E11/E12：地图来源 + VoxelLayer + `pointcloud_to_laserscan`（注意 E08 的近场盲区）+ `/joint_states`；RViz 配置补机器人模型显示 |
