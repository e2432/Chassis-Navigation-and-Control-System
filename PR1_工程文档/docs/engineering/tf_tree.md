# TF 树与真值里程计（E09）

W3 / E09。目标（`工程侧详细任务路线.md`）：

> 发布真值里程计，建立唯一 TF 树。产出 `docs/engineering/tf_tree.md`，逐条列出父子帧、发布者、参考系和重置行为。
> 验收：静止时状态稳定；前进、侧移、旋转的位姿与速度均正确；无 TF 多发布者冲突或持续 extrapolation 错误。

本文所有数字都是本机实测（`results/w3/e09/`），配置文件是 `src/robot_sim/config/tf_tree.yaml`。

---

## 1. 结论摘要

| 项目 | 结果 | 证据 |
| --- | --- | --- |
| `/odom` | `nav_msgs/Odometry`，`header.frame_id=odom`、`child_frame_id=base_footprint`，时间戳=仿真时间 | `results/w3/e09/integration/verify_summary.txt` |
| `/odom` 频率 | **60.00 Hz**（= 物理步率，实测 1463 条 / 24.37 s），无重复时间戳 | 同上 |
| twist 帧 | **机体系**：与"真值位姿差分再转到机体"的误差 0.0000 m/s；世界系假设误差 0.2126 m/s | 同上 |
| 前进 / 侧移 / 原地转 | 机体位移 (+0.448, 0.000) m / (0.000, +0.448) m / Δyaw +1.567 rad（kinematic 模式，指令 0.15 m/s × 3 s、0.5 rad/s × 3.1 s） | 同上 |
| 静止稳定性 | 静止段最大线速度 0.0000 m/s、漂移 0.00000 m/s | 同上 |
| TF | `map→odom` identity、`odom→base_footprint` 与 `/odom` 一致（Δ=0）、`base_footprint→base_link`=0.105 m、`base_link→lidar_front_link`=(0.13,0.13,0.2185)、`lidar_front_link→imu_link`=0 | 同上 |
| TF 更新率 | 60.00 Hz（与 /odom 同源同频） | 同上 |
| 多发布者冲突 | `base_footprint` 只有父帧 `odom` 一个；静止时无位姿跳变 | 同上 |
| 重置一致性 | 真值跳变（(0.45,0.45)→(0,0)）后 0.2 s 内 `/odom` 与 TF 都已对齐新真值（Δ=0），`map→odom` 未被改写 | 同上 |
| 节点级单测 | 合成真值（yaw=45°、世界系 0.3 m/s + 0.5 rad/s）→ twist 转成机体 (0.2121, −0.2121)，误差 2.8e-17 | `results/w3/e09/node_unit_test.txt` |

---

## 2. 帧表（逐条：父子帧 / 发布者 / 参考系 / 重置行为）

| 子帧 | 父帧 | 发布者 | 类型 | 参考系与数值 | 重置行为 |
| --- | --- | --- | --- | --- | --- |
| `odom` | `map` | `robot_sim/map_odom_static_node.py`（E10 由 `truth_tf.launch.py` 拉起） | **静态** `/tf_static`（transient_local） | 真值模式固定 identity：xyz=(0,0,0)，yaw=0（`tf_tree.yaml` 可配） | 不随重置变化（全局地图原点与 odom 一直重合） |
| `base_footprint` | `odom` | `robot_sim/truth_odom_node.py`（真值适配节点） | **动态** `/tf`，60 Hz | 直接取仿真真值（不积分、不漂移）；z=0、roll=pitch=0 | 重置后**下一帧就跟到新真值**，不需要重新初始化 |
| `base_link` | `base_footprint` | `robot_state_publisher`（URDF 固定关节） | 静态 | z=+0.105 m | 不随重置变化（相对量） |
| `lidar_front_link` | `base_link` | `robot_state_publisher`（URDF） | 静态 | (0.13, 0.13, 0.2185) m | 同上 |
| `lidar_rear_link` | `base_link` | `robot_state_publisher`（URDF） | 静态 | (−0.13, −0.13, 0.2185) m | 同上 |
| `imu_link` | `lidar_front_link` | `robot_state_publisher`（URDF） | 静态 | (0, 0, 0)（PDF 2.5：IMU 集成在雷达内） | 同上 |
| `mount_plate_link` | `base_link` | `robot_state_publisher`（URDF） | 静态 | z=+0.146 m | 同上 |
| `wheel_{fl,fr,rl,rr}_link` | `base_link` | `robot_state_publisher`（URDF，连续关节） | 静态（当前按 0 位姿） | ±0.23546 m | E12 接 `/joint_states` 后轮子姿态才真实 |

**链**：`map → odom → base_footprint → base_link → {lidar_front_link → imu_link, lidar_rear_link, mount_plate_link, wheel_*_link}`

> **与路线书字面的差异（必须知道）**：路线书写的是 `map → odom → base_link`，本仓库 URDF 的根是
> `base_footprint`（地面投影，E05 定稿），所以真值适配节点发的是 `odom→base_footprint`，
> `base_footprint→base_link` 由 URDF 固定关节给出（0.105 m）。效果：
> `ros2 run tf2_ros tf2_echo odom base_link` 照样成立，Nav2 用 `base_link` 作 `robot_base_frame` 不受影响；
> 路线书里的 `lidar_link` 在本仓库对应 `lidar_front_link` / `lidar_rear_link`（E04 契约 #10）。

---

## 3. 话题与数据约定

| 话题 | 类型 | 帧语义 | 频率 | 说明 |
| --- | --- | --- | --- | --- |
| `/sim/ground_truth` | `nav_msgs/Odometry` | `header.frame_id=world`、`child_frame_id=base_footprint`，**twist 是世界系** | 每物理步（60 Hz） | 仿真原始真值；`publishRawVelocities=True`，故意不做坐标系转换 |
| `/odom` | `nav_msgs/Odometry` | `header.frame_id=odom`、`child_frame_id=base_footprint`，**twist 在机体系** | 跟随输入（60 Hz） | 真值适配节点输出；`pose` 原样透传，`twist` 做 R(−yaw) 旋转 |
| `/tf` | `tf2_msgs/TFMessage` | `odom→base_footprint` | 60 Hz | 唯一发布者 |
| `/tf_static` | `tf2_msgs/TFMessage` | `map→odom` + URDF 固定关节 | 一次锁定 | `map→odom` 由本仓库节点发，URDF 部分由 RSP 发 |
| `/clock` | `rosgraph_msgs/Clock` | — | 渲染 tick（~84 Hz，同值重复无害） | 所有 header.stamp 都用它 |

约定：

- **四元数**：ROS 消息与 TF 一律 **xyzw**；Isaac 侧 `quatd` 输入是 **IJKR**，在桥接脚本里换序（`[qx,qy,qz,qw]`）。
- **单位**：长度 m、角度 rad（URDF/TF 的 rpy 同样是弧度）、速度 m/s 与 rad/s。
- **时间**：全部用仿真时间；节点必须 `use_sim_time:=true`，否则 `/odom` 时间戳会和 TF 对不上（E10 launch 默认打开）。
- **tf 语义**：`pose` 是"子帧在 header 帧下的位姿"，`twist` 是"表达在 child_frame 下的速度"。
- **重置**：真值模式不积分，重置瞬间 `/odom` 与 TF 直接跳到新真值；为避免下游看到"几十 m/s 的假速度"，
  真值适配节点会把超过 `max_plausible_twist_mps`（默认 5 m/s）的 twist 置 0 并打 warn（实测重置那一帧被置 0，
  整段记录 twist 峰值 0.15 m/s）。

---

## 4. 互斥表（E09 检查项 6：不能叠加两套 TF）

| 发布者 | 发什么 | 何时可用 | 规则 |
| --- | --- | --- | --- |
| `robot_sim/truth_odom_node.py` | `odom→base_footprint` + `/odom` | 真值模式（默认） | 启动时扫 `/tf`+`/tf_static`，发现已有 `odom→base_*` 发布者就**拒绝发布并退出**（`exit 2`） |
| `robot_sim/map_odom_static_node.py` | `map→odom`（静态） | 真值模式 | 切 SLAM/定位时必须停用，否则与定位节点抢 `map→odom` |
| `tools/engineering/e08_tf_publish.sh` | 静态 `odom→base_footprint`（identity） | **只在 E08 的 RViz 演示** | 与真值模式冲突；真值模式下不要起 |
| `robot_a_sim_control/truth_localization.py` | 动态 `map→odom`（上一代 6 寸轮口径） | 既有链路专用 | 与本仓库真值节点互斥；`simulation.launch.py` 里由 `legacy:=false` 默认关闭 |
| SLAM/定位节点（E19 起） | 动态 `map→odom` | SLAM 模式 | 用 `truth_tf.launch.py truth:=false` 关掉本仓库的两个真值节点 |

---

## 5. 怎么跑 / 怎么验

```bash
cd ${NAV_WS}
source /opt/ros/humble/setup.bash && source install/setup.bash
export ROS_DOMAIN_ID=42

# 一条命令：节点级单测（不需要 Isaac，合成真值）
./tools/engineering/e09_node_test.sh

# 一条命令：仿真 + TF 树 + 外部核对（kinematic 模式做方向类验收）
./tools/engineering/e09_run_all.sh results/w3/e09/integration 34 kinematic
# 物理模式（真实动力学；平移只有指令的 ~30%，见第 7 节）
./tools/engineering/e09_run_all.sh results/w3/e09/physics_mode 22 physics

# 手动核对（仿真在跑时，另开终端）
ros2 topic echo /odom --once
ros2 topic hz /odom
ros2 run tf2_ros tf2_echo odom base_link
ros2 run tf2_ros tf2_echo base_link lidar_front_link
ros2 run tf2_ros tf2_echo lidar_front_link imu_link
```

---

## 6. 验证脚本都查了什么（`tools/engineering/e09_verify.py`）

1. `/odom` 帧语义与时间戳（是否仿真时间、是否落在 `/clock` 区间内）；2. 实测频率与抖动；
3. **twist 是不是机体系**（与真值差分对比，并证明不是世界系）；4. 静止稳定性；
5. 运动段自动识别（前进/左移/后退/左右转）并核对机体位移方向；
6. TF：`map→odom` identity、`odom→base_footprint` 与 `/odom` 一致、URDF 外参、TF 更新率；
7. 冲突：`base_footprint` 父帧唯一、静止时无跳变、twist 无假尖峰；
8. 重置：真值跳变后 `/odom` 与 TF 在 0.2 s 内对齐，`map→odom` 未改写。

---

## 7. 已知限制（诚实记录）

| 限制 | 说明 | 后续 |
| --- | --- | --- |
| 物理模式的平移方向不对 | 圆柱轮没有辊子，E06 已实测：平移只到指令的 ~30%，且前进/侧移都被"打偏"约 45°（`results/w3/e09/physics_mode/verify_summary.txt` 里这些方向检查是 FAIL，属于已知物理模型缺欠而非 TF/odom 的错误） | 方向类验收用 `mode:=kinematic`；要真实接触动力学得上辊子模型或换仿真器（E19 决策） |
| `base_footprint` 而不是 `base_link` 作为 `odom` 子帧 | 见第 2 节说明 | 若算法侧要求严格 `odom→base_link`，可把 URDF 根改成 `base_link`（会影响 E05 的 USD 引用，需评估） |
| 轮子只有 0 位姿 | 没有 `/joint_states` | E12 接 `joint_state_publisher` |
| RViz 里 `RobotModel` 显示 Status: Error | 底盘网格（STL）在 RViz 里没渲染出来；点云、TF、里程计轨迹都正常（见 `results/w3/e10/rviz_e10.png`） | E11/E12 一并处理：改用 `File` 源或排查 mesh 材质；不影响 `/odom`/TF 功能 |
| 没有 `map` 到世界的真值对齐偏差 | 真值模式 `map` 与仿真 world 完全重合 | 换场景/地图片时用 `map_odom_static_node` 的 x/y/yaw 参数对齐 |

---

## 8. 产出清单

| 文件 | 内容 |
| --- | --- |
| `src/robot_sim/config/tf_tree.yaml` | 帧表、发布者、频率、互斥、约定 |
| `src/robot_sim/robot_sim/nodes/truth_odom_node.py` | 真值适配节点（`/odom` + `odom→base_footprint`，含冲突自检、世界系→机体系转换、重置处理） |
| `src/robot_sim/robot_sim/nodes/map_odom_static_node.py` | `map→odom` 静态节点 |
| `src/robot_bringup/launch/truth_tf.launch.py` | RSP（URDF TF）+ 上述两个节点 |
| `tools/engineering/e09_unit_test_nodes.py`、`tools/engineering/e09_node_test.sh` | 节点级单测（合成真值，不需 Isaac） |
| `tools/engineering/e09_verify.py`、`tools/engineering/e09_run_all.sh` | 集成验证（仿真 + TF + 外部核对） |
| `tools/engineering/e09_sys_env.sh` | 干净 ROS 环境（避开本机 conda Python 3.13 破坏 rclpy） |
| `results/w3/e09/` | `node_unit_test.txt`、`integration/`（verify_summary、odom_samples.csv、启动与仿真日志、`rviz` 截图）、`physics_mode/` |
