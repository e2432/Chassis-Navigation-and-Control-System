# E01 环境核对与兼容版本冻结

- 任务：W1 / E01「核对真实环境，锁定兼容版本」
- 采集日期：2026-10-30（起算周 W1 = 2026-10-26 ~ 11-01）
- 采集方式：全部在本机 Ubuntu 上实际执行命令，不依赖 Windows 侧文件或历史记录推测
- 证据目录：`results/w1/e01/`
  - `env_inventory.txt`：系统/GPU/ROS 2/DDS/Isaac Sim 全量命令输出（由 `tools/engineering/e01_env_inventory.sh` 生成）
  - `sim_startup_stdout.log`：headless 空场景启动 + ROS 2 Bridge **成功**启用的完整日志（`tools/engineering/e01_sim_startup_probe.py`）
  - `sim_startup_stdout_no_ros_source.log`：同一探针在**未 source ROS 2** 时的失败日志（第 6 节“关键坑”的原始证据）
  - `gui_startup_evidence.txt`：GUI 方式启动 5.1 的关键日志行摘录（完整日志在 `~/.nvidia-omniverse/logs/Kit/Isaac-Sim Full/5.1/kit_20261030_104911.log`）

> 本文只记录**已实际验证**的内容。未验证项在文末单列，不当作已完成。

## 1. 结论：本项目冻结的兼容组合

| 组件 | 冻结版本 / 取值 | 依据 |
|---|---|---|
| 操作系统 | Ubuntu 22.04.5 LTS (jammy), kernel 6.8.0-136-generic | `lsb_release -a`, `uname -a` |
| GPU | NVIDIA GeForce RTX 5070 Laptop GPU, 8151 MiB 显存, compute capability 12.0 (sm_120) | `nvidia-smi --query-gpu=...` |
| GPU 驱动 | 580.178.04（open kernel module） | `nvidia-smi`, `/proc/driver/nvidia/version` |
| CUDA（驱动侧） | 13.0 | `nvidia-smi` 头部 |
| ROS 2 | Humble（`/opt/ros/humble`，387 个 `ros-humble-*` 包） | `printenv ROS_DISTRO`, `dpkg -l` |
| DDS/RMW | `rmw_fastrtps_cpp`（fastrtps 2.6.11 / rmw-fastrtps-cpp 6.2.10） | `dpkg-query`, 仿真侧显式 `RMW_IMPLEMENTATION=rmw_fastrtps_cpp` |
| `ROS_DOMAIN_ID` | 默认未设置（=0）；本项目检查统一显式设为 42 | `printenv`, 见 `bridge_check.md` |
| Isaac Sim | **5.1.0**（pip 包 `isaacsim 5.1.0.0`，Kit 107.3.3+production） | `isaacsim/VERSION`, `pip list`, Kit 日志 |
| 仿真 Python | 3.11.15（conda env `env_isaaclab`） | `python -V` |
| 外部 ROS Python | 3.10.12（`/usr/bin/python3`，系统 rclpy） | `python3 -V`, `import rclpy` |
| Isaac Sim 启动入口 | `${ISAAC_SIM_ROOT}/isaac-sim.sh` | 实际运行 + Kit 日志 `Cmd:` 行 |
| ROS 2 Bridge 扩展 | `isaacsim.ros2.bridge`（扩展版本 **4.12.4**，随 Isaac Sim 5.1 自带） | `config/extension.toml`, Kit 日志启动行 |
| 桥接 ROS 来源 | `system_default`（使用已 source 的系统 ROS 2 Humble） | `extension.toml` 设置项 |

**冻结规则**：除非出现阻塞性问题（见第 8 节），本项目中途不升级 Isaac Sim、不换 ROS 发行版、不改 DDS 实现。
任何版本变动都必须在本文件登记变更人和日期，并重跑 E01/E02 的检查脚本。

## 2. 主机与系统

| 项目 | 值 |
|---|---|
| 主机名 / 用户 | `robot` / `wxw` |
| 系统 | Ubuntu 22.04.5 LTS (Jammy Jellyfish) |
| 内核 | 6.8.0-136-generic（x86_64） |
| CPU | AMD Ryzen 9 8945HX，32 逻辑核 |
| 内存 | 30 GiB（无 swap） |
| 根分区 | `/dev/nvme0n1p6`，288 G，已用 58 G，**可用 215 G**（22%） |
| 编译器 / 构建 | gcc 11.4.0，cmake 3.22.1，colcon（`/usr/bin/colcon`），rosdep（`/usr/bin/rosdep`），git 2.34.1 |

容量提示：`/tmp` 是 16 G 的 tmpfs，落在内存里。仿真资产、rosbag 与批量结果不要写到 `/tmp`，否则会挤占内存并随重启丢失。

## 3. GPU

- 型号：NVIDIA GeForce RTX 5070 Laptop GPU，8151 MiB 显存（另有 AMD 核显 `05:00.0`，未参与仿真）
- 驱动：580.178.04（`nvidia-driver-580-open`，open kernel module）；`/proc/driver/nvidia/gpus/0000:01:00.0`
- 当前状态（采集时）：空载 49–55 °C，显存占用 15–600 MiB，Xorg 与 Isaac Sim 在跑

驱动兼容性注意（来自 Isaac Sim 启动日志 `[gpu.foundation.plugin]`）：

```text
The list of unsupported driver ranges being verified with known bugs (RTX):
[0.0, 535.129)
The recommended drivers: 535.161.07 (GameReady, Studio, RTX/Quadro, Grid/vGPU)
The latest driver version may also be used but is not fully tested.
```

即：本机 580.178.04 **不在**“有已知缺陷的不支持区间”内，但也不在 NVIDIA 推荐的 535.161.07 上。
实际验证结果（本文件第 6 节、`bridge_check.md`）表明 5.1 在本驱动上可以正常启动和出图；
因此按“可用就冻结”的原则保留 580.178.04，并把“驱动不在推荐列表”登记为**已知风险**，不主动降级。

显存是硬约束：8 GiB 需要同时容纳 Isaac Sim 渲染、物理、相机/雷达渲染目标、RViz2 以及算法成员的 MPC 进程。
运行测得：
- GUI 全量 5.1 空场景启动后约 0.5–3.4 GiB 占用（叠两个实例时实测 3420 MiB / 8151 MiB）；
- 结论：正式批量实验只允许**一个** Isaac Sim 实例，跑批前先关闭其它实例并确认 `nvidia-smi` 空闲。

## 4. ROS 2 Humble

```text
$ ls -d /opt/ros/*
/opt/ros/humble
$ source /opt/ros/humble/setup.bash && printenv ROS_DISTRO ROS_VERSION
humble
2
$ dpkg -l | grep -c "^ii  ros-humble"
387
```

关键包（版本以 `dpkg-query` 为准，见 `env_inventory.txt` 第 5 节）：

| 包 | 版本 | 用途 |
|---|---|---|
| `ros-humble-desktop` | 0.10.0-1jammy | 基础桌面栈（含 RViz2、rqt） |
| `ros-humble-ros-base` | 0.10.0-1jammy | 基础节点 |
| `ros-humble-nav2-bringup` | 1.1.20-1jammy | W4 起接入 Nav2 |
| `ros-humble-rviz2` | 11.2.26-1jammy | W3 起可视化 |
| `ros-humble-tf2-tools` | 0.25.20-2jammy | `tf2_echo` 核对 TF |
| `ros-humble-rmw-fastrtps-cpp` | 6.2.10-1jammy | 默认 RMW |
| `ros-humble-fastrtps` | 2.6.11-1jammy | DDS 实现 |

`ros2 doctor --report` 提示部分包“local 低于 latest”（例如 `ros2cli 0.18.18 < 0.18.21`）。
这是 apt 源里已有更新版本，**不需要处理**：本项目冻结当前已装版本，不批量升级 Humble 包。

入口：`.bashrc` 里已有 fishros 安装脚本写入的 `source /opt/ros/humble/setup.bash`。
非交互式 shell（脚本、systemd、DSH 这类工具调用）**不会**读 `.bashrc`，因此本仓库所有脚本都显式 source，见第 6 节。

## 5. Isaac Sim 5.1（实际安装形态）

本机不是“下载安装器 + `~/isaacsim` 目录”那种布局，而是 **pip 安装进 conda 环境**：

```text
conda env     : env_isaaclab  (Python 3.11.15)
site-packages : ${ISAAC_SIM_ROOT}
pip 版本      : isaacsim 5.1.0.0（含 isaacsim-app/core/robot/sensor/ros2/... 等 27 个分包子包）
VERSION 文件  : 5.1.0-rc.19+release.26219.9c81211b.glapps
Kit 版本      : 107.3.3+production.229672.69cbf6ad.gl
应用配置      : <site-packages>/isaacsim/apps/isaacsim.exp.full.kit（App Name: 'Isaac-Sim Full'）
日志目录      : ~/.nvidia-omniverse/logs/Kit/Isaac-Sim Full/5.1/kit_<timestamp>.log
同环境另装    : IsaacLab 0.54.4（editable，来自 ~/IsaacLab/IsaacLab）——本期路线不使用 IsaacLab
```

### 启动方式（唯一可用入口）

```bash
# GUI
${ISAAC_SIM_ROOT}/isaac-sim.sh

# headless（脚本化，本项目 E02 及后续自动执行使用）
${ISAAC_PYTHON} your_script.py   # 脚本内 SimulationApp({"headless": True})
```

**已知问题 1：桌面快捷方式不可用。**
`~/.local/share/applications/IsaacSim.desktop` 的 `Exec=` 指向
`.../site-packages/isaacsim/isaac-sim.sh`，而该文件在 pip 版 5.1 中**不存在**（实测 `ls` 为空）。
解决办法：不依赖桌面图标；使用上面的 `bin/isaacsim`。若要在桌面放图标，改为指向 `bin/isaacsim`。

**已知问题 2：首次/冷启动慢。**
同一个 RTX 着色器缓存尚未编译时，日志出现 `Waiting for RtPso async group async compilation`。
实测三种情况：
- GUI 全量 app 空场景冷启动到 `Isaac Sim Full App is loaded`：**111 s**；
- headless 脚本、缓存已热：`SimulationApp ready` **7.8–8.0 s**（E02 两次运行均为 8 s 量级）；
- headless 脚本、与 GUI 同时启动（着色器在编译）：**89.6 s**。

自动化脚本必须给足超时（建议 ≥ 240 s），不要把“启动慢”误判为“启动失败”。

**已知问题 3：GUI 与 headless 同时运行会抢显存。** 见第 3 节末尾。

## 6. ROS 2 Bridge 名称与启用方式（按 5.1 实际安装核对）

- 扩展名：**`isaacsim.ros2.bridge`**（不要沿用旧名 `omni.isaac.ros2_bridge`；后者在 5.1 里只作为
  `extsDeprecated/` 下的废弃扩展存在，日志中会被注册但不应使用）
- 扩展版本：4.12.4（`<site-packages>/isaacsim/exts/isaacsim.ros2.bridge/config/extension.toml`）
- 关键设置项：
  - `exts."isaacsim.ros2.bridge".ros_distro = "system_default"` —— 用系统已 source 的 ROS 2
  - `exts."isaacsim.ros2.bridge".publish_without_verification = false` —— 没有订阅者时默认不发布（见 `bridge_check.md`）
  - `exts."isaacsim.ros2.bridge".enable_nitros_bridge = false`
- Bridge 自带的动作图节点（供 E02 及后续使用）：
  `ROS2Context`、`ROS2PublishClock`、`ROS2PublishTransformTree`、`ROS2PublishOdometry`、
  `ROS2PublishImu`、`ROS2PublishPointCloud`、`ROS2PublishLaserScan`、`ROS2SubscribeTwist`、
  `ROS2Subscriber`、`ROS2Publisher`、`ROS2Service*` 等（完整清单见扩展 `docs/ogn/`）。
- 官方参考：[Isaac Sim 5.1 ROS 2 安装与桥接](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/installation/install_ros.html)、
  [Isaac Sim 5.1 ROS 2 Clock](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/ros2_tutorials/tutorial_ros2_clock.html)

### 启用方式（两种，均以“先 source 系统 ROS 2”为前提）

1. GUI：`Window > Extensions`→ 搜索 `isaacsim.ros2.bridge` → `ENABLED`。5.1 全量 app 默认会随启动加载该扩展
   （实测 GUI 日志有 `[ext: isaacsim.ros2.bridge-4.12.4] startup`）。
2. 脚本：

```python
import omni.kit.app
omni.kit.app.get_app().get_extension_manager().set_extension_enabled_immediate("isaacsim.ros2.bridge", True)
```

### 关键坑（实测）：不 source ROS 2 时 Bridge 启动失败

如果按“干净环境”启动仿真（剥离了 `AMENT_PREFIX_PATH` 等变量），Bridge 会报：

```text
[Error] [isaacsim.ros2.bridge.impl.extension] ROS2 Bridge startup failed
Checking to see if RMW can be loaded:
failed to get symbol 'rmw_init_options_init' due to Environment variable 'AMENT_PREFIX_PATH' is not set or empty
RMW was not loaded
```

原因：扩展自带的轻量件（`exts/isaacsim.ros2.bridge/humble/{lib,rclpy}`）只包含库和 python 包，
**不含 ament index**，因此 `rmw_implementation` 找不到候选 RMW。
`extension.toml` 里 `ros_distro = "system_default"` 也说明本机预期走系统 ROS 2。

复现命令（保留为回归检查，不要删）：

```bash
E01_NO_ROS_SOURCE=1 ./tools/engineering/e01_run_sim_startup.sh results/w1/e01 tools/engineering/e01_sim_startup_probe.py
# 预期：日志出现 ROS2 Bridge startup failed + bridge_enabled_after=False
```

**结论（本项目规则）**：启动 Isaac Sim 前必须 source `/opt/ros/humble`。
脚本化启动统一走 `tools/engineering/e02_run_harness.sh` / `tools/engineering/e01_run_sim_startup.sh`，不要手工剥离 ROS 环境。

### 关键坑（实测）：仿真内置 `rclpy` 不可用

source 系统 ROS 2 后，Bridge 的 C++ 动作图节点正常，但 python 侧：

```text
[Warning] [humble.rclpy] Could not import rclpy
Attempting to load system rclpy: No module named 'rclpy._rclpy_pybind11'
  (_rclpy_pybind11.cpython-311-x86_64-linux-gnu.so 在 /opt/ros/humble/...python3.10... 下不存在)
```

原因：系统 rclpy 是 Python 3.10 的绑定，仿真进程是 Python 3.11。
**结论**：仿真侧只用 **动作图（C++）节点**发消息；不要在仿真进程里 `import rclpy`。
外部 ROS 2 侧照常用 `/usr/bin/python3`（3.10）+ 系统 rclpy 写节点，两边互不污染。

（附注：Bridge 自带的 `humble/rclpy` 里有 3.11 绑定，`PYTHONPATH`+`LD_LIBRARY_PATH` 指过去后
在独立进程里 `import rclpy` 实测可用；但它绕开系统 ROS 2，本期路线**不使用**，只在需要时另行验证。）

## 7. DDS / 环境隔离

| 项 | 仿真侧 | 外部 ROS 2 侧 |
|---|---|---|
| Python | 3.11.15（conda `env_isaaclab`） | 3.10.12（`/usr/bin/python3`） |
| ROS 2 库来源 | Bridge 扩展链接系统 `/opt/ros/humble/lib`（`ros_distro=system_default`） | `/opt/ros/humble` |
| RMW | `rmw_fastrtps_cpp` | `rmw_fastrtps_cpp` |
| `ROS_DOMAIN_ID` | 显式 `42`（检查用） | 显式 `42` |
| 通信方式 | 本机（`lo` / `wlo1` / `enp4s0` 均可见） | 同机 |

规则：
1. 两侧 `RMW_IMPLEMENTATION` 必须一致（都 fastrtps）。本机**只装了 fastrtps**：
   `dpkg-query` 显示 `ros-humble-rmw-fastrtps-cpp` 已安装，
   `ros-humble-rmw-cyclonedds-cpp` 为 `not-installed`（未安装 cyclonedds）。
   若以后装了 cyclonedds 并在一侧切过去，会出现“`ros2 topic list` 看得到话题但收不到数据”。
2. 两侧 `ROS_DOMAIN_ID` 必须一致。本机默认均为 0；批量实验建议固定一个非 0 的值，
   避免与同网段其他人的 ROS 2 混淆。
3. `ROS_LOCALHOST_ONLY` 未设置（=0）。仅本机通信时可设为 `1` 减少发现范围。
4. `PYTHONPATH` / `LD_LIBRARY_PATH` 不要在两侧之间复制：仿真侧吃 conda + 系统 ROS 2，
   外部侧吃 `/opt/ros/humble`，混起来就是上面第 6 节的两种失败。

## 8. 什么时候才算“需要改动冻结版本”（阻塞判据）

同时满足以下任一条才允许调整，且必须先与算法成员书面确认、登记到本文件：

- Isaac Sim 在空场景（无自定义资产）反复启动失败，且日志指向驱动/Kit 层缺陷，换驱动或换 5.x 小版本可解决；
- Bridge 无法完成 E02 的双向收发，且确认不是 QoS/域名/环境变量问题；
- 显存不足以跑“机器人 + 雷达 + 相机”的最低组合，且关闭其它实例后仍 OOM。

不构成改动理由：`ros2 doctor` 报告新版可用、看到别人用 Rolling/Jazzy 文档、桌面图标不工作（有替代入口）。

## 9. 重启后的自检（E01 验收步骤）

重启 Ubuntu 后按顺序执行，全部通过才算“环境可重复启动”：

```bash
cd <your-ws>

# 1) 重新采集清单，和本文件第 2–5 节比对（重点看显存、可用磁盘、ROS_DISTRO）
./tools/engineering/e01_env_inventory.sh

# 2) headless 空场景 + 启用 ROS 2 Bridge，确认退出码 0 且出现 bridge_enabled_after=True
./tools/engineering/e01_run_sim_startup.sh results/w1/e01 tools/engineering/e01_sim_startup_probe.py \
    > results/w1/e01/sim_startup_stdout.log 2>&1
grep -E "^\[E01\]" results/w1/e01/sim_startup_stdout.log

# 3) 外部 ROS 2 可用
source /opt/ros/humble/setup.bash && ros2 doctor --report | head -5
```

判定标准（对照 2026-10-30 的实测输出）：
- `ros2` 命令可用、`ROS_DISTRO=humble`；
- 探针输出 `app_version=5.1.0`、`kit_version=107.3.3+production.229672.69cbf6ad.gl`、`bridge_enabled_after=True`；
- 探针输出 `bridge_ext_version=4.12.4`、`bridge_ros_distro_setting=system_default`、
  `bridge_publish_without_verification=False`、`bridge_ogn_node_count=30`；
- 探针输出 `sim_rclpy_import=FAIL` 属**预期**（Python 3.11 无系统 rclpy 绑定，见第 6 节），不是故障；
- 日志无 `ROS2 Bridge startup failed`；
- `simapp_ready_wall_s` 在缓存已热时应在 10 s 量级；
- GUI 方式启动一次（`bin/isaacsim`）能看到空场景，且日志出现 `Isaac Sim Full App is loaded`。

2026-10-30 实测：以上全部通过；GUI 与 headless 各启动成功一次，E02 双向通信通过（见 `bridge_check.md`）。

## 10. 未验证 / 待确认项（不得当作已完成）

- **RAID/快照与备份策略**未确认；215 G 可用空间能否支撑 W7 的 400 次 run + rosbag 尚未实测估算；
- **Mid-360 仿真资产**是否存在、与真机点云字段是否一致——W3（E08）才核对；
- **第二块 GPU / 多实例并行**未验证；本期按“单实例串行”设计；
- **远程托管与分支保护**属于 E03 范围，尚未建立（本机只有本地目录）；
- **驱动 580.178.04 的长期稳定性**未验证（不在 NVIDIA 推荐列表）；出现渲染异常时优先怀疑此项；
- IsaacLab 0.54.4 与 Isaac Sim 5.1 的版本对应关系未核对（本期不使用，不阻塞）。

## 11. 变更记录

| 日期 | 变更 | 变更人 |
|---|---|---|
| 2026-10-30 | 首次采集并冻结：Ubuntu 22.04.5 / ROS 2 Humble / Isaac Sim 5.1.0 / RTX 5070 Laptop 8 GiB / 驱动 580.178.04 / fastrtps | 工程侧（成员三） |
