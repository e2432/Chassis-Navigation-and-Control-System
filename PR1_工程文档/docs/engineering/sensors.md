# E08 传感器接入：3D 雷达 + IMU

W2 / E08。目标（见 `工程侧详细任务路线.md`）：

> 接入激光雷达与 IMU；输出 `docs/engineering/sensors.md`、传感器配置文件、点云与 IMU 示例记录。
> 验收：RViz 里能看到合理的环境结构；机器人转动时数据变化方向正确；记录实际频率与丢帧，
> **不先假定能达到某个频率**。

> **2026-11-04 话题改名（对齐算法侧契约 v1.1）**：`/points_raw` → **`/lidar/points`**。
> 本文 E08 实测数据是在旧名下采集的（数据本身不受影响）；代码/配置/RViz 已全部切到新名，
> 需要兼容旧脚本时给桥接加 `--cloud-alias /points_raw` 可同时发一份。

本文件里的所有频率、点数、耗时都是**本机实测值**（Linux + RTX 5070 Laptop，headless，
`ROS_DOMAIN_ID=42`），原始证据在 `results/w2/e08/`。

---

## 1. 结论摘要

| 项目 | 结果 | 证据 |
| --- | --- | --- |
| 雷达话题 | `/points_raw`，`sensor_msgs/PointCloud2`（x/y/z float32），`frame_id=lidar_front_link` | `results/w2/e08/ros/verify_summary.txt` |
| 雷达实测频率 | **10.02 Hz**（178 条 / 17.77 s 仿真时间，重复发布 1.00×，缺口 0 帧） | `results/w2/e08/bag_analysis.txt` |
| IMU 话题 | `/imu`，`sensor_msgs/Imu`，`frame_id=imu_link`，含重力 | 同上 |
| IMU 实测频率 | **60.11 Hz**（= 物理步率；标称 200 Hz，缺口 69.9%，属发布路径限制） | 同上 |
| RViz 结构 | 地面环 + 墙体 + 箱体 + 雷达坐标系清晰可见，`Global Status: Ok` | `results/w2/e08/rviz_points.png` |
| 转动方向 | 机器人逆时针转 90° 后，世界 `x≈+3 m` 的墙从机体 `(+3.23, +0.18)`（方位 +3.3°）转到 `(+0.56, −3.12)`（方位 −79.8°），且六个阶段都满足 `方位角 ≈ −偏航 + 3.3°`；IMU `wz` 同步由 0 变 +0.463，反向旋转变 −0.463 | `cloud_frames.png`、`imu_series.png`、`sensors_log.csv` |
| 点云耗时 | 单帧 2880 条射线：均值 **16.7 ms**、最大 20.4 ms（占 100 ms 预算的 17%，不限制 10 Hz） | `results/w2/e08/sensor_summary.txt` |

一句话：**雷达可以按 10 Hz 稳定发布真实几何、IMU 能反映真实动态，但 IMU 发布频率受物理步率
限制只能到 ~60 Hz，达不到规格书 200 Hz；雷达是自建射线投射模型，不是 Livox Mid-360 的真实
扫描模型。**

---

## 2. 规格 / 仿真模型 / 实测 三者对照

| 项目 | 规格书（AGL-CHS-03-F01 V1.2） | 仿真模型（本仓库） | 实测 |
| --- | --- | --- | --- |
| 雷达数量 | 2 台 3D 固态雷达，对角安装 | 1 台（前向） | 1 台 |
| 水平 FOV | 360° | 360°（1°/束，360 束） | 360° |
| 垂直 FOV | −7° ~ +52° | −7° ~ +52°（8 束均分） | 同 |
| 量程 | 40 m @10% 反射率，截止 100 m | 40 m | 40 m |
| 点频/帧点数 | 未给出 | 2880 点/帧（360×8） | 488~502 点/帧（见 §5 说明） |
| 频率 | 未给出 | 10 Hz | **10.02 Hz** |
| 数据字段 | 未给出 | x、y、z（无 intensity/ring/time） | 同 |
| IMU 采样率 | 200 Hz（集成在雷达内） | IMUSensor 配置 200 Hz | 发布 **60.11 Hz** |
| IMU 输出 | 未给出 | 姿态 + 角速度 + 线加速度（含重力） | 静止 az≈9.81，水平 ax/ay≈0 |
| 超声 | 4 个（前后左右） | 未建（URDF 里默认关闭） | — |

> 帧点数 2880 是**发射**的射线数；实测每帧只有 488~502 点，是因为大部分射线打到 40 m 外
> （地面只有 50×50 m，射线飞出后被裁掉）或打到天空返回空。这一点必须写清楚：**不能用点数
> 反推分辨率**。（详见 §5）

配置放在 `src/robot_sim/config/sensors.yaml`，脚本读它（命令行参数可覆盖）。

---

## 3. 雷达：为什么是自建模型，而不是官方 RTX 雷达资产

**查证结果**（`isaacsim.sensors.rtx` / `isaacsim.ros2.bridge` 4.12.4）：

- 本机 Isaac Sim 5.1 自带 18 个 RTX 雷达 USD 配置、47 个 JSON profile，覆盖
  NVIDIA / SLAMTEC / SICK / ZVISION / Velodyne / HESAI / Ouster，**没有 Livox Mid-360**
  （规格书的 FOV −7°~52°、40 m@10% 与 Mid-360 同量级，但规格书本身没写型号）；
- 且 `/Isaac/Sensors/...` 传感器资产包在本机不可用（Nucleus/资产包不可达），
  即使用其它型号的 profile 也取不到 USD 资产。

所以 E08 采取的做法是：用 PhysX 射线投射实现一个**参数可配的通用 3D 固态雷达模型**，
参数对齐规格书（360°×(−7°~52°)、40 m、10 Hz），并**明确记录它与真实雷达的差异**（§8）。

实现要点（`tools/engineering/e08_sensor_pipeline.py`）：

```python
q = omni.physx.get_physx_scene_query_interface()
q.raycast_all(origin, direction, max_range, callback)   # callback 收集所有命中
```

- 射线方向在**雷达坐标系**里固定生成（`DIR_LOCAL`），逐帧乘上机体航向旋转到世界系；
- 命中点再转回雷达坐标系写进 `PointCloud2`（所以 `frame_id=lidar_front_link` 是真实的）；
- **必须排除自身命中**：−7° 的下俯射线会先打到车体顶面（`/World/robot_a/...`），
  这些命中被丢弃，否则点云里会出现一圈"跟着车转"的假障碍物。

---

## 4. IMU

- 用 `isaacsim.sensors.physics.IMUSensor`，挂在 `/World/robot_a/base_link/ImuE08`
  （与雷达同位置，符合"IMU 集成在雷达内"的规格），`frequency=200`；
- `read_gravity=True`：线性加速度**含重力**，水平静止时 `az ≈ +9.81 m/s²`——与真实 IMU 一致
  （下游用 `imu_filter` / EKF 时要按此处理，不能误以为是"去重力"数据）；
- 踩坑记录：**IMU 只有在机器人真的被物理仿真时才有效**。E08 早期版本为保证位姿精确而
  用"瞬移 + 关重力"（运动学模式）驱动底盘，结果 IMU 三个轴全是 0。改为 physics 模式
  （保留重力 + 轮子速度驱动）后 `az=9.810`、转向时 `wz≈±0.46` 都正常。
  → 因此脚本保留 `--mode kinematic|physics`，**要验证 IMU 必须用 physics 模式**。

---

## 5. 频率、抖动与丢帧（不假定目标频率，全部实测）

### 5.1 测量方法

1. **仿真内**：脚本自己记录每次发布的仿真时间（`results/w2/e08/sensors_log.csv`，采集时的旧话题名 `/points_raw`）；
2. **订阅端**：独立进程 `tools/engineering/e08_ros_verify.py` 订阅话题，用 `/clock` 的仿真时间做基准，
   同时统计 `header.stamp` 相同（重复发布）与间隔分布；
3. **录制回放**：`ros2 bag record` 录 `/points_raw /imu /clock`，再用
   `tools/engineering/e08_bag_analyze.py` 按 `header.stamp` 去重后统计**真实更新频率**。

第 3 步是必要的：**消息条数 ≠ 数据更新次数**，见 §6。

### 5.2 实测结果（`results/w2/e08/bag_analysis.txt`）

```
bag 原始消息数: /clock=1494, /imu=1068, /points_raw=178
仿真时间跨度 17.77 s / 录音墙钟 20.65 s（实时因子 0.86）
/clock: 1494 条，仿真步长中位 16.67 ms / max 16.67 ms

/points_raw   原始 178 条 / 去重后 178 帧 → 原始 10.02 Hz，真实更新 10.02 Hz，
              重复倍数 1.00×（标称 10 Hz，缺口 0 帧 / 0.0%）
/imu          原始 1068 条 / 去重后 1068 帧 → 原始 60.11 Hz，真实更新 60.11 Hz，
              重复倍数 1.00×（标称 200 Hz，缺口 2485 帧 / 69.9%）
```

| 话题 | 标称 | 实测（仿真时间基准） | 抖动（相邻间隔） | 重复发布 | 缺口 |
| --- | --- | --- | --- | --- | --- |
| `/points_raw` | 10 Hz | **10.02 Hz** | 中位 ~99.7 ms，P95 116 ms | 1.00× | 0 帧 |
| `/imu` | 200 Hz | **60.11 Hz** | 中位 16.7 ms，P95 32 ms | 1.00× | 69.9% |
| `/clock` | —（按渲染 tick） | 84 条/秒（墙钟） | 仿真步长恒为 16.67 ms | — | — |

**丢帧结论**：

- **物理步没有丢**：`/clock` 的仿真时间步长中位与最大值都是 16.67 ms，说明没有出现 33 ms
  的跳变（跳变=掉物理步）；
- **点云没有丢**：按 10 Hz 应收 178 帧，实收 178 帧；
- **IMU 的 70% 缺口不是丢包**，而是**发布路径上限**：IMU 数据只在物理步里更新
  （60 Hz 仿真 = 实测 60.11 Hz），ROS 发布走同一节奏，不可能超过它。要 200 Hz 需要
  另一条链路（见 §9）。

---

## 6. 踩过的坑：Ros2Bridge 发布节点会"每帧重发当前值"

这是 E08 里最花时间的一个问题，记录下来避免后续再踩。

1. 直接把 `ROS2PublishPointCloud` / `ROS2PublishImu` 的 `execIn` 接到
   `omni.graph.action.OnPlaybackTick` 上时，**每个渲染 tick 都会把当前输入值再发一次**。
   实测：点云与 IMU 都是 **~84 Hz**（= 渲染 tick 率），而数据其实 10 Hz / 60 Hz 才更新。
   而且**时间戳每帧都在变**（`timeStamp` 接的是仿真时间），所以"按 stamp 去重"也识别不出
   重复——下游 Nav2/SLAM 会当作 84 Hz 的传感器在跑。
2. 试过用 `omni.graph.action.Branch` 做电平门控：`app.update()` 一次可能跑 1~2 个 tick，
   电平会被消费多次，实测仍有 **1.4× 重复**（Branch 本身没问题，用 Counter 验证过
   200 tick → 34 次触发，符合预期；问题在于"电平不是边沿"）。
3. 最终方案：`omni.graph.action.OnCustomEvent` 做**边沿触发**。Python 侧每到该发新数据时
   推一个 carb 事件：

   ```python
   bus = omni.kit.app.get_app().get_message_bus_event_stream()
   EV = carb.events.type_from_string("omni.graph.action.e08_pub_cloud")
   bus.push(EV, payload={})        # 一次事件 = 一次发布
   ```

   图里 `OnCustomEvent.outputs:execOut → PubCloud.inputs:execIn`。用
   `omni.graph.action.Counter` 验证：推 100 次事件 = 100 次发布（与 tick 数无关）。
   于是才有 §5 的 1.00× 重复倍数。

   > 细节：`OnCustomEvent` 是 pop 订阅，发布发生在**下一次图求值**（约 1 个 tick，≤20 ms），
   > 时间戳取发布时刻的仿真时间，因此时间戳与点云内容的偏差 ≤1 帧。

4. 触发节奏：点云按仿真时间每 0.1 s 一次；IMU **每物理步**一次（`dt > 0`），
   所以不会出现"同一物理帧被发两次"。

---

## 7. 数据变化方向正确吗

### 7.1 点云：墙的机体坐标随航向变化

场景 `assets/scenes/e05_min_scene.usd` 里有一面墙在世界 `x≈3 m`、一辆小箱体在 `x≈1.5 m`。
记录每个阶段"最近的墙体命中点"在**机体坐标系**下的位置（`results/w2/e08/sensors_log.csv`）：

| 阶段 | 机体偏航 | 墙在机体系 | 方位角 | 距离 | −偏航 |
| --- | --- | --- | --- | --- | --- |
| still（初始） | −1.1° | (+3.23, +0.18) | +3.3° | 3.24 m | +1.1° |
| rot_ccw_90 | 41.1° | (+2.46, −1.87) | −37.2° | 3.09 m | −41.1° |
| forward | 82.9° | (+0.56, −3.12) | −79.8° | 3.17 m | −82.9° |
| rot_cw_90 | 52.9° | (+2.08, −2.45) | −49.7° | 3.22 m | −52.9° |
| lateral_left | −1.0° | (+3.15, +0.18) | +3.3° | 3.16 m | +1.0° |
| still2 | −1.1° | (+3.23, +0.18) | +3.3° | 3.24 m | +1.1° |

六个阶段都满足 **方位角 ≈ −偏航角 + 3.3°**（常数项来自机器人在世界 y 方向的小偏移），
也就是"墙始终在它的世界方向上，只是机体坐标系在转"。趋势与预期一致：
**航向 +90° → 原本在前方（+x）的墙跑到机体右侧（−y）**；转回来又回到 +x。
`results/w2/e08/cloud_frames.png` 里四帧俯视图能直接看出墙从"右侧竖线"变成"下方横线"再变回来。

### 7.2 IMU：转向符号与角速度

`results/w2/e08/imu_series.png`：

- 线加速度：静止段 `az=9.810`（重力），`ax≈ay≈0`；启动/停止瞬间有 ±1~4 m/s² 的尖峰（正常）；
- 角速度：逆时针段 `wz` 稳定在 **+0.463 rad/s**，顺时针段 **−0.463 rad/s**（指令 ±0.5，
  轮子打滑/驱动带宽使其到 0.46，与 E06 的力学位移结论一致）；
- 侧移段 `wz≈0`（正确：横移不转向）。

点云侧向、IMU 侧向、指令符号三者互相印证。

---

## 8. RViz 里能看到什么

`tools/engineering/e08_rviz_check.sh`（bag 回放 + `robot_state_publisher` 发 URDF TF + `rviz2` +
窗口截图）产出 `results/w2/e08/rviz_points.png`：

- **地面环**：`r≈2.7 m` 的圆——这不是场景里的物体，而是 **−7° 最下沿射线打到地面的交线**；
- **墙**：右侧一条竖线（z 高，按 Z 值着色呈紫色），距离 ≈2.9 m；
- **箱体**：`x≈1.5 m` 附近的一小段；
- **雷达坐标系**：红/绿/蓝三轴在原点；
- `Global Status: Ok`、`PointCloud: Status: Ok`、`Fixed Frame: odom`（URDF 帧链已接通）。

`Fixed Frame` 用 `odom` 是因为 `tools/engineering/e08_tf_publish.sh` 起了 `robot_state_publisher`
（URDF）+ 静态 `odom→base_footprint`。**这一步顺带验证了 E08 用的帧名与 URDF 完全一致**
（`lidar_front_link` / `imu_link`），也就是 E04 接口契约第 10 条里那个待确认项的技术前提。

### 8.1 在 Isaac Sim GUI 里亲手验证（`tools/engineering/e08_gui_sensors.py`）

不用命令行跑 headless，直接在 GUI 里看：

1. Isaac Sim 里 `File > Open` → `assets/scenes/e05_min_scene.usd`，点 **Play**；
2. `Window > Script Editor` → `File > Open` 选 `tools/engineering/e08_gui_sensors.py` → **Run**
   —— 底盘开始按剧本循环（静止 3 s → 原地逆时针 90° → 前进 → 顺时针 90° → 侧移 → 静止），
   **雷达点云实时画在视口里**（绿色的 `/World/E08Debug/points`），同时发
   `/points_raw`、`/imu`、`/clock`；
3. 视口里应当能对上三样东西：**地面环**（r≈2.6 m 的一圈点，−7° 射线的落地交线）、
   **墙**（`x≈3 m` 的一条竖排点）、**箱体**（`x≈1.5 m` 附近一小簇）；
   机器人在原地逆时针转 90° 时，墙会从"正前方"移动到"右侧"——这就是方向正确性的直观版；
4. 另一个终端（`ROS_DOMAIN_ID` 必须与 GUI 一致，GUI 里脚本会打印出来，未设置即默认 0）：

   ```bash
   export ROS_DOMAIN_ID=42            # 与启动 Isaac Sim 时的值一致
   ros2 topic hz /points_raw          # 期望 ≈10 Hz
   ros2 topic hz /imu                 # 期望 ≈60 Hz（= 物理步率，不是 200 Hz）
   ros2 topic echo /imu --once        # az≈9.81，转向时 wz 在 ±0.46 附近
   ros2 topic echo /points_raw --once --field header.frame_id   # lidar_front_link
   ```

5. 想在 RViz 里看（不依赖 Isaac 窗口）：`rviz2 -d config/e08_rviz.rviz`，
   Fixed Frame 选 `lidar_front_link` 也能看（有 TF 时选 `odom`）。

其它开关（同一次会话里直接再执行一行）：

```python
e08_gui_start(motion=False)   # 只发传感器，不动底盘（想自己摆位/用别的驱动时）
e08_gui_start(draw=False)     # 关掉视口里的点云，只看话题
e08_gui_stop()                # 停（也会打印本次点云/IMU 条数）
```

> 注意：**必须 physics 模式**（脚本默认）。IMU 是物理传感器，若用瞬移（kinematic）驱动，
> 它看到的角速度/加速度会退化为 0 —— 这是 E08 排查中最容易踩的坑。

**排障：机器人"原地起飞"怎么办？**（2026-11-01 实际踩到）

- 原因：`UsdPhysics.Scene` 的 `gravityMagnitude` 是**正的标量大小**，重力方向由
  `gravityDirection`（默认 `(0,0,−1)`）决定。GUI 脚本早期版本在"恢复重力"时写了
  `Set(-9.81)`，等于**重力朝上**，车就会被顶起来。
- 现在脚本会自检：发现 `gravityMagnitude ≤ 0` 就纠正为 `9.81` 并打印原值；
  停止时还原原值。每次运行结束还会打印 `|base_footprint.z|` 的最大值（正常 0.000 m），
  高度超过 0.3 m 会立刻告警。
- 如果车已经被甩飞：同一次会话里执行 `e08_gui_start(reset_pose=True)`（复位到原点），
  或者菜单 `File > Revert` 重载场景后点 Play 再来一次。
- 自测证据：`results/w2/e08/gui_smoke.txt` 里第二段就是"故意写负重力 → 自动纠正 →
  `|z|` 最大 0.000 m"的对照记录。

### 8.2 验收判定清单（对着这张表打勾就行）

| # | 要确认的事 | 怎么查 | 期望 | E08 实测 |
| --- | --- | --- | --- | --- |
| 1 | 雷达在发且频率对 | `ros2 topic hz /points_raw` | ≈10 Hz | **10.02 Hz** |
| 2 | 帧名与 URDF 一致 | `ros2 topic echo /points_raw --once --field header.frame_id` | `lidar_front_link` | 一致 |
| 3 | 点云是真实几何 | 视口/RViz 看地面环、墙、箱体 | 结构可辨 | 见 `rviz_points.png` |
| 4 | 转动时数据方向正确 | GUI 里看墙的相对方位 / `cloud_frames.png` | 方位角 ≈ −偏航 | 六阶段全符 |
| 5 | IMU 有重力 | `ros2 topic echo /imu --once` | `az≈9.81`、`ax≈ay≈0` | 9.810 |
| 6 | IMU 角速度符号正确 | 逆/顺时针各转一次 | `wz` +0.46 / −0.46 | +0.463 / −0.463 |
| 7 | IMU 频率（不假定 200 Hz） | `ros2 topic hz /imu` | = 物理步率 | **60.11 Hz** |
| 8 | 有没有丢帧/重复 | `tools/engineering/e08_bag_analyze.py` | 重复 1.00×、点云缺口 0 | 1.00× / 0 帧 |
| 9 | 物理步有没有掉 | `/clock` 相邻步长 | 恒 16.67 ms | 中位=max=16.67 ms |

---

## 9. 已知限制与风险（必须一起交付）

| 限制 | 影响 | 后续 |
| --- | --- | --- |
| 近场盲区：−7° 最下沿射线要 2.7 m 才落地 | **2.7 m 内看不到地面级障碍物**（矮于 ~0.3 m 的物体在近处不可见） | E10 起 Nav2 用 `pointcloud_to_laserscan` 时要把 `min_height`/`range_min` 设对；必要时加超声（规格书 4 个）补近场 |
| 只有 8 条垂直束（规则栅格） | 与真实非重复花瓣扫描不同，同一物体相邻帧落点数会跳变；远处结构会"稀疏" | 提高 `v_rays` 或换 RTX 雷达资产（需资产包） |
| 无噪声/无反射率/无运动畸变 | 点云"太干净"，SLAM/配准算法在真机上的鲁棒性无法在这里验证 | E13 起如需要再补噪声模型 |
| 只有 1 台雷达（规格书 2 台对角） | 后方视野缺 360°，实际只覆盖前向半球以外的部分仍靠单台 360°（本模型是单台 360°，所以没有后方盲区，但**没有冗余**） | E09/E10 补第二台，验证双雷达拼接 |
| IMU 发布 60 Hz（标称 200 Hz） | 高动态场景（急停/撞击）采样不足 | 需要时改用 RTX IMU 或独立发布线程；先把结论写进契约 |
| IMU 无噪声/偏置 | EKF 参数整定结果不可信 | 同上 |
| physics 模式下轮子是圆柱（无辊子） | 平移只到指令的 ~30%（E06 结论），但**转速/角速度接近指令（~92%）**，所以 E08 的旋转/IMU 验证不受影响 | E09 起做接触建模或继续用运动学模式验证算法 |
| 没有 `/tf` 的全链路 | 现在只有 URDF 静态树 + `odom`，没有 `map`/里程计 | E12 由 launch 统一拉起；E11 里程计 |

---

## 10. 怎么复现

```bash
cd ${NAV_WS}
source /opt/ros/humble/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp ROS_DOMAIN_ID=42

# 1) 仿真侧：发 /points_raw、/imu、/clock，并按剧本转/走（physics 模式才有真实 IMU）
./tools/engineering/e01_run_sim_startup.sh results/w2/e08 tools/engineering/e08_sensor_pipeline.py \
    --out results/w2/e08 --duration 30 --mode physics

# 2) 订阅端实测（另一个终端，同一 ROS_DOMAIN_ID）
/usr/bin/python3 tools/engineering/e08_ros_verify.py --duration 18 --out results/w2/e08/ros

# 3) 录制 + 分析
ros2 bag record -o results/w2/e08/bag /points_raw /imu /clock
/usr/bin/python3 tools/engineering/e08_bag_analyze.py --bag results/w2/e08/bag --out results/w2/e08

# 4a) 想在 Isaac Sim GUI 里看：打开场景 → Script Editor 运行 tools/engineering/e08_gui_sensors.py
# 4b) 出图 + RViz 截图（RViz 需要一个能显示窗口的 DISPLAY）
/usr/bin/python3 tools/engineering/e08_make_figures.py --dir results/w2/e08
./tools/engineering/e08_rviz_check.sh
```

传感器参数改 `src/robot_sim/config/sensors.yaml`（脚本默认读它，命令行显式参数优先）。

---

## 11. 产出清单

| 文件 | 内容 |
| --- | --- |
| `src/robot_sim/config/sensors.yaml` | 传感器配置（规格/模型/话题/帧/QoS/发布机制/差异） |
| `docs/engineering/sensors.md` | 本文件 |
| `tools/engineering/e08_sensor_pipeline.py` | 仿真侧：射线投射雷达 + IMU + 事件触发发布 + 运动剧本 |
| `tools/engineering/e08_ros_verify.py` | 订阅端：类型/帧/频率/丢帧/数据极角 |
| `tools/engineering/e08_bag_analyze.py` | bag 分析：去重后的真实频率、几何统计 |
| `tools/engineering/e08_make_figures.py` | 点云俯视图 / 单帧对比 / IMU 时序 |
| `tools/engineering/e08_gui_sensors.py` | **GUI 可视化入口**（视口画点云 + 发话题 + 剧本驱动；`--headless-test` 可自动冒烟） |
| `tools/engineering/e08_rviz_check.sh` + `tools/engineering/e08_tf_publish.sh` + `tools/engineering/e08_shoot_window.py` | RViz 检查链路（含 X11 截图） |
| `config/e08_rviz.rviz` | RViz 配置（俯视正交、AxisColor、TF、Fixed Frame=odom） |
| `results/w2/e08/` | `sensors_log.csv`、`sensor_summary.txt`、`bag/`、`bag_analysis.txt`、`cloud_frames.csv`、`imu_frames.csv`、`cloud_sample.npz`、`ros/`、`cloud_topview.png`、`cloud_frames.png`、`imu_series.png`、`rviz_points.png`、`sample_record.txt` |

> **入库说明**：按 `.gitignore` 的既有约定，`*.db3`/`*.mcap` 这类大体积原始数据不进仓库
> （`results/w2/e08/bag/` 只留 `metadata.yaml`）。为了让"示例记录"本身可查，另外生成
> `results/w2/e08/sample_record.txt`：2 条 `PointCloud2` 的点坐标、6 条 `Imu` 的
> 加速度/角速度，含时间戳与坐标系，可直接对照 `sensors.yaml` 里的字段说明。

---

## 12. 遗留 / 待确认

1. **接口契约 E04 第 10 条**：`livox_frame` → `lidar_front_link` / `lidar_rear_link` / `imu_link`
   已在仿真侧落地并用 RViz 验证；仍需算法侧确认其 `pointcloud_to_laserscan`、SLAM、
   Nav2 costmap 配置是否已切到新帧名。
2. **雷达具体型号**：规格书只写"3D 固态雷达"，没给型号/线数/点频。E08 按 8 束建模，
   拿到真实型号后应更新 `sensors.yaml` 的 `h_rays/v_rays/point_per_frame` 并重测频率。
3. **第二台雷达**（对角安装）与**4 个超声**未建模。
4. **IMU 200 Hz 达不到**：需与算法侧确认 60 Hz 是否够用（EKF/SLAM 通常 50~100 Hz 可接受），
   若不够则要在 E12 之前换发布链路。
5. **近场盲区**（2.7 m 内无地面点）需要在 E10/E11 的 costmap 参数里显式处理。
