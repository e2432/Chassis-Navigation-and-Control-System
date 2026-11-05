# E02 桥接与仿真时间检查

- 任务：W1 / E02「验证桥接和仿真时间」
- 执行日期：2026-10-30（同一台机器连续两次完整运行）
- 环境前提：见 `docs/engineering/environment.md`（Ubuntu 22.04.5 / ROS 2 Humble / Isaac Sim 5.1.0 / fastrtps / `ROS_DOMAIN_ID=42`）
- 证据目录：`results/w1/e02/run_monotonic/`（`resetOnStop=false`）与 `results/w1/e02/run_reset/`（`resetOnStop=true`）
  每次运行包含：`summary.txt`、`summary.json`、`ext_cli_evidence.txt`、`sim_stdout.log`、
  `ext_clock_trace.csv`、`sim_clock_samples.csv`、`sim_sub_rx.csv`、`ext_pub_log.csv`、
  `ext_tf_trace.csv`、`ext_simtime_probe.csv`、`sim_events.csv`
- 另有独立对照证据：`results/w1/e02/use_sim_time_probe.txt`（由 `tools/engineering/e02_use_sim_time_probe.py` 生成）

> 说明：本文件只记录本次实际测到的行为。所有结论都可由上面的 CSV 与脚本重新算出，
> 汇总命令：`/usr/bin/python3 tools/engineering/e02_analyze.py --out results/w1/e02/<run>`。

## 1. 结论摘要

| 检查项 | 结果 | 证据 |
|---|---|---|
| 仿真 → 外部：`/clock` 被外部 ROS 2 收到 | **通过** | 外部收到 1382 / 1375 条 `rosgraph_msgs/msg/Clock` |
| 仿真 → 外部：`/tf` 被外部 ROS 2 收到 | **通过** | `world -> W1TestFrame`，translation (1.0, 2.0, 0.5) |
| 外部 → 仿真：外部发的 `Twist` 被仿真侧收到 | **通过** | 仿真在线窗口内 149 条，收到 124/125 个序号，匹配 123/124 条（83%） |
| `/clock` 随仿真推进而增加 | **通过** | 分段内单调，RTF≈0.997，实测 59.9–60.0 Hz |
| 暂停时仿真时间停止推进 | **通过** | 暂停 5.02 s 墙上时间，`/clock` 停在 26.400 s 不动，恢复后从 26.417 s 继续 |
| 停止并重播后的时钟行为 | **已记录，两种配置都测** | `resetOnStop=false`：保持单调（32.333 → 32.450 s）；`resetOnStop=true`：回跳（32.867 → 0.067 s） |
| 参与仿真的节点使用 `use_sim_time=true` | **通过** | 探针节点 ROS 时间与同刻 `/clock` 完全一致（中位差 0.000 s），跨 45 s 墙上时间只走 23 s，重置时跟着回跳；反向对照见 `use_sim_time_probe.txt` |
| DDS 域统一（`ROS_DOMAIN_ID=42` 双侧显式设置） | **通过** | `ext_cli_evidence.txt` 第 2 行 |

两次运行 `summary.json` 均为 `"overall_pass": true`。

## 2. 被测链路与拓扑

```text
Isaac Sim 5.1 (headless, Python 3.11, conda env_isaaclab)
  └─ isaacsim.ros2.bridge 4.12.4（ros_distro=system_default，链接系统 /opt/ros/humble）
        └─ Action Graph /W1BridgeCheck
             ├─ OnPlaybackTick + IsaacReadSimulationTime + ROS2PublishClock   → /clock
             ├─ ROS2PublishTransformTree                                      → /tf
             └─ ROS2SubscribeTwist                                            ← /w1/ext_to_sim
                                ⇅  DDS (rmw_fastrtps_cpp, ROS_DOMAIN_ID=42)
外部 ROS 2 Humble 终端（/usr/bin/python3 3.10）
  ├─ ros2 topic list / info -v / echo --once / hz        （CLI 证据）
  ├─ e02_ext_recorder.py：订阅 /clock、/tf，发布 /w1/ext_to_sim（5 Hz）
  └─ 同进程内 use_sim_time=True 的节点，用来核对 ROS 时间
```

为什么这样拆：系统 rclpy 是 Python 3.10 绑定，仿真进程是 Python 3.11，
**仿真侧不能用 python rclpy**（`docs/engineering/environment.md` 第 6 节）。
因此仿真侧发消息一律用 C++ 动作图节点，外部侧才用 python 节点。

## 3. 方向一：仿真 → 外部 ROS 2（`/clock`、`/tf`）

外部终端实测（`results/w1/e02/run_monotonic/ext_cli_evidence.txt`）：

```text
$ ros2 topic list
/clock
/parameter_events
/rosout
/tf
/w1/ext_to_sim

$ ros2 topic info /clock -v
Type: rosgraph_msgs/msg/Clock
Publisher count: 1
Node name: _W1BridgeCheck_PublishClock
QoS profile:
  Reliability: RELIABLE
  Durability: VOLATILE
  History (Depth): UNKNOWN

$ ros2 topic echo /clock --once
clock:
  sec: 2
  nanosec: 483333462
---
```

```text
$ ros2 topic echo /tf --once
transforms:
- header:
    stamp: {sec: 13, nanosec: 733334049}
    frame_id: world
  child_frame_id: W1TestFrame
  transform:
    translation: {x: 1.0, y: 2.0, z: 0.5}
    rotation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
```

注意点（写进接口契约前必须知道）：

1. **Bridge 发布者不在 `ros2 node list` 里**，`ros2 node list` 输出为空，但 `ros2 topic info -v`
   能看到 `_W1BridgeCheck_PublishClock` 这类节点名。排查时用 `topic info`，不要因为 `node list` 空就判“桥接没起来”。
2. Bridge 节点的 QoS 是 `RELIABLE / VOLATILE / KEEP_LAST`，`History (Depth)` 显示 `UNKNOWN`。
   订阅端不要用 `BEST_EFFORT`，否则会静默收不到数据。
3. `publish_without_verification = false`（扩展默认）：**没有订阅者时不发布**。
   所以必须先起订阅（`echo`/`hz`/记录器），再判断“有没有数据”。
4. `/clock` 是**每个渲染帧发一次**，本机实测 59.9–60.0 Hz，不是固定的 30/50 Hz。
   下游不要假定 `/clock` 频率，也不要用它的频率反推控制频率。

## 4. 方向二：外部 ROS 2 → 仿真（`/w1/ext_to_sim`）

外部以 5 Hz 发布 `geometry_msgs/msg/Twist`，把序号写进字段（`angular.z = seq`，`linear.x = 0.001*seq`），
仿真侧 `ROS2SubscribeTwist` 收到后逐帧读取节点输出写入 `sim_sub_rx.csv`。

| 运行 | 仿真在线窗口内发出 | 仿真收到序号 | 匹配 |
|---|---|---|---|
| run_monotonic | 149 | 124 | 123（83%） |
| run_reset | 149 | 125 | 124（83%） |

未匹配的约 17% 不是“桥接丢包”，逐条核对后确认成因：

- 暂停/停止窗口内发布的指令到不了节点（Bridge 的发布订阅只在播放时活动），
  该窗口约 7 s × 5 Hz ≈ 35 条；
- 少量消息与暂停恢复瞬间的帧边界重合，仿真在该帧之前未处理即被下一条覆盖（队列深度 10）。

结论：**双向通信成立，内容可逐条比对（序号一致）**；但“发送条数 = 接收条数”不成立，
后续 E06（`/cmd_vel` 超时停车）与 E13（安全层）必须按“可能丢帧”设计超时逻辑，而不是假设必达。

## 5. 仿真时间行为（本文件最需要精确记录的部分）

### 5.1 `/clock` 与墙上时间的关系

| 运行 | 段 | 墙上 | 仿真 | RTF | 实测频率 |
|---|---|---|---|---|---|
| run_monotonic | 段0（触发后第一段播放） | 12.16 s | 12.12 s | 0.997 | 59.9 Hz |
| run_monotonic | 段1（恢复播放） | 5.99 s | 5.97 s | 0.996 | 59.9 Hz |
| run_monotonic | 段2（重置后播放） | 4.92 s | 4.90 s | 0.996 | 60.0 Hz |
| run_reset | 段0 | 12.04 s | 12.00 s | 0.997 | 59.9 Hz |
| run_reset | 段1 | 5.99 s | 5.97 s | 0.996 | 59.9 Hz |
| run_reset | 段2（重置后从 0 开始） | 4.92 s | 4.90 s | 0.996 | 60.0 Hz |

RTF ≈ 0.997 是脚本限速的结果（`--target-rtf 1.0`）。**不限速时 headless 空场景实测可到 5–6 倍实时**
（早期 smoke 试跑：4 s 墙上 → 23.6 s 仿真）。正式批量实验必须显式限制 RTF 或固定步长，
否则“仿真时间”与“墙上时间”比较毫无意义。

### 5.2 暂停

两次运行都在暂停窗口留下同样的证据：`/clock` 连续 5.02 s 没有新消息，
最后一条停在 26.400 s（`run_monotonic`）/ 26.933 s（`run_reset`），恢复后从 26.417 / 26.950 s 继续。
即：**暂停时仿真时间不推进，恢复后接着走，不补齐暂停期间的时间**。

同时注意：暂停期间仿真侧的“墙上时间”仍在走（`sim_events.csv` 的 `wall_s` 一直在增），
所以任何统计必须区分“仿真时间”和“墙上时间”，不能混用。

### 5.3 停止并重新播放（重置场景）

两个配置的差别就在这里，这是后续每次 `benchmark.run()` 重置场景都必须处理的问题：

| `IsaacReadSimulationTime.inputs:resetOnStop` | `/clock` 停止前最后一条 | 重播后第一条 | 结论 |
|---|---|---|---|
| `false`（默认，run_monotonic） | 32.333 s | 32.450 s | **单调，不回跳**，累积计数 |
| `true`（run_reset） | 32.867 s | 0.067 s | **回跳 −32.8 s**，重置后从 0 开始 |

关键细节（易误判，必须记下来）：

- 两种配置下，`omni.timeline.get_current_time()` 在 `stop()` 之后**都**读回 0.0；
  但 `false` 时 `/clock` 并不回跳。也就是说 **`timeline` 的当前时间不能代表 `/clock`**，
  记录时钟一律以 `/clock` 消息为准。
- 因此接口契约里必须明确选哪种语义：
  - 基准实验要求“每次 run 从 0 开始、时间可比较”→ 选 `resetOnStop=true`，
    并且 `benchmark.run()` 在重播后等第一个 `/clock` 消息再启动记录（否则会记到上一轮的尾巴）；
  - 需要跨任务连续累计→ 选 `false`，但要在结果里写明“仿真时间为累积值”。

### 5.4 `use_sim_time=true`

外部侧起了一个 `use_sim_time=True` 的节点，每 0.5 s 记录它自己的 `get_clock().now()`：

- run_monotonic：ROS 时间 14.283 → 37.350 s（+23.07 s），同期墙上时间跨 44.94 s；
  与同刻 `/clock` 消息相比中位偏差 **0.000 s**，最大 0.067 s（一帧）；
- run_reset：ROS 时间 14.933 → 4.967 s，**跟着时钟回跳**，与 `/clock` 偏差中位 0.000 s。

即：`use_sim_time=true` 生效，节点时间来源确实是 `/clock`，而不是墙上时钟。

反向对照（不需要仿真在跑，`results/w1/e02/use_sim_time_probe.txt`）：

```text
use_sim_time=False  node_ros_time=1793329276.323050  wall_epoch=1793329276.323071  diff=-0.000  (Unix 墙上时间)
use_sim_time=True   node_ros_time=0.000000          wall_epoch=1793329278.338593  diff=-1793329278.339  (非墙上时间（来自 /clock）)
```

两点结论：
1. `use_sim_time=false` 时节点报的是 Unix 时间，与仿真时间里量级差 1.79e9 秒——所以“时间看起来不对”先查这个参数；
2. `use_sim_time=true` 且**没有** `/clock` 发布者时，节点时间是 0 而不是墙上时间。
   这既是参数生效的反证，也提醒：仿真没播放时下游节点的 `now()` 是 0，`tf2` 之类会因此报错，
   启动顺序必须先有 `/clock` 再起下游节点。

## 6. DDS 与环境设置（统一域）

| 设置 | 仿真侧 | 外部侧 | 说明 |
|---|---|---|---|
| `ROS_DOMAIN_ID` | 42（`e02_run_harness.sh` 导出） | 42（`e02_ext_check.sh` 导出） | 双侧显式设置，避免依赖默认 0 |
| `RMW_IMPLEMENTATION` | `rmw_fastrtps_cpp` | `rmw_fastrtps_cpp` | 必须一致 |
| ROS 2 库 | 系统 `/opt/ros/humble/lib` | `/opt/ros/humble` | 仿真侧必须先 source ROS 2，否则 Bridge 启动失败 |
| Python | 3.11.15 | 3.10.12 | 不要互相 import |

### 6.1 “能看到话题却收不到数据”的排查顺序

按代价从低到高，逐项排除，不要同时改：

1. `ros2 topic list` 能看到、`ros2 topic info -v` 的 **Publisher/Subscription count 与类型**对不对？
   类型不一致时 DDS 不会报错，只是没人收。
2. **QoS**：Bridge 是 `RELIABLE + VOLATILE`。订阅端用了 `BEST_EFFORT` 会静默收不到。
3. **域名**：两侧 `echo $ROS_DOMAIN_ID` 是否相同；`ROS_LOCALHOST_ONLY` 是否一侧为 1。
4. **是否在播放**：Bridge 的节点只在 `Play` 时活动，暂停/停止期间发来的消息会丢。
5. **是否有人订阅**：`publish_without_verification=false` 时无订阅者不发布，先起订阅再看。
6. **看桥接日志**：`~/.nvidia-omniverse/logs/Kit/Isaac-Sim Full/5.1/kit_<ts>.log`，
   搜 `isaacsim.ros2.bridge`、`ROS2 Bridge startup failed`、`Could not import`。
7. **环境混用**：外部终端里是否被污染了 conda/仿真路径；仿真终端里是否缺 `AMENT_PREFIX_PATH`。

本次两轮运行中没有出现“话题可见但收不到”的情况；上述顺序来自本轮实际踩到的两个坑
（不 source ROS 2 → Bridge 启动失败；`node list` 为空但桥接正常）。

## 7. 一键复现

```bash
cd <your-ws>

# 配置 A：停止后时钟单调（默认 resetOnStop=false）
./tools/engineering/e02_run_all.sh results/w1/e02/run_monotonic false 45

# 配置 B：停止后时钟回跳（resetOnStop=true，基准实验推荐语义）
./tools/engineering/e02_run_all.sh results/w1/e02/run_reset true 45
```

脚本做的事（约 3 分钟/次，其中 Isaac Sim 启动约 60–120 s）：

1. `e02_run_harness.sh`：source 系统 ROS 2、导出 `ROS_DOMAIN_ID=42`、以 headless 启动 Isaac Sim，
   建动作图（`/clock`、`/tf`、订阅 `/w1/ext_to_sim`）；
2. 等 `/clock` 在外部可见后，`e02_ext_check.sh` 收 CLI 证据（topic list/info/echo/hz），
   再跑 45 s 记录器（订阅 + 5 Hz 发布 + `use_sim_time` 探针）；
3. 仿真侧收到第一条 Twist 后开始剧本：播放 → 暂停 5 s → 恢复 → 停止重置 → 再播放；
4. `e02_analyze.py` 用 `wall_epoch` 对齐两个进程的记录，输出 `summary.txt` / `summary.json`。

## 8. 对后续任务的约束（写进接口契约 v1 的内容）

1. `/clock` 由仿真侧唯一发布；其它节点一律 `use_sim_time=true`，禁止自建时钟。
2. 场景重置语义二选一并写死：基准实验用 `resetOnStop=true`（每次从 0 开始），
   重播后**等第一个 `/clock`** 再开始记录。
3. 采样与统计一律用仿真时间；墙上时间只用于诊断实时因子与计算耗时。
4. 从外部向仿真下发的指令可能丢帧（本轮实测约 17% 落在暂停/停止窗口），
   因此 E06 的“指令超时停车”和 E13 的安全层必须按超时设计，不能假设必达。
5. Bridge 节点不出现在 `ros2 node list` 中，健康检查以 `topic info -v` 与 `/clock` 频率为准。
6. 仿真进程不做 `import rclpy`；仿真内发消息只能走动作图节点。

## 9. 未验证 / 遗留项

- **重置后 Nav2/tf2 缓存行为**未验证（E09、E15 才涉及）：时钟回跳会让 `tf2` 抛出
  extrapolation 错误的可能，需要届时专门测；
- **`/clock` 在 GPU 负载升高时的抖动**未测：本轮 RTF 人为限到 1.0，且场景为空；
  W7 批量实验要重新记录实际频率与丢帧；
- **跨机器 DDS（多机共享仿真）**未验证，本期只做单机；
- **相机/雷达类话题吞吐**（点云、图像 QoS 与带宽）未测，W3（E08）再用同一套脚本扩展。
