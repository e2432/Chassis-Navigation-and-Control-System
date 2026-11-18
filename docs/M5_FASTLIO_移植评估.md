# M5 FAST-LIO ROS2/Humble 移植评估（前半段）

> 2026-11-18 记。范围：候选获取、代码/依赖审查、与 Isaac 接口的适配清单。
> 编译验证因需用户显式授权第三方源码构建而未完成（见 §5）。
> 本文件不改变 M5 闸门结论：闸门已满足，M5 仍未正式启动。

## 0. 结论

- **首选候选**：`myeongw002/FAST_LIO_LOCALIZATION_ROS2`（ROS2，Humble 实测；
  源码含 FAST-LIO 核心 + ICP 全局定位 + TF 融合）。
- **备选**：`Smart-Wheelchair-RRC/FAST_LIO_LOCALIZATION2`（基于 Ericsii FAST_LIO_ROS2；
  含 velodyne/ouster/mid360 配置，但依赖 `ros2_numpy`/Open3D，Python 依赖更重）。
- **核心后备**：`Ericsii/FAST_LIO_ROS2` + 自研 PCL ICP/NDT 定位。
- 两者都支持 `sensor_msgs/PointCloud2` 的 `default_handler`（XYZ-only 可用），
  但 **Isaac 点云无逐点 time/ring/intensity**，需要一层零数学桥或改 handler；
  这是 M5 最大适配点。
- **许可证风险**：两个候选的 `LICENSE` 文件均为 **GPLv2**，而 `package.xml` 写 BSD；
  对外分发前必须确认实际许可。

## 1. 候选清单（已 pin commit）

| 候选 | 分支/commit | 日期 | 说明 |
|---|---|---|---|
| myeongw002/FAST_LIO_LOCALIZATION_ROS2 | `main` @ `3a005b965c566245c33a624b60a31235e77d3c44` | 2025-08-14 | ROS2 port；FAST-LIO 核心在库内；定位脚本用 Open3D ICP；Humble 推荐 |
| Smart-Wheelchair-RRC/FAST_LIO_LOCALIZATION2 | `ros2` @ `f04974907c8da976dd0495b272d18ac4c534d41f` | 2026-01-17 | 基于 Ericsii/FAST_LIO_ROS2；Open3D + ros2_numpy(<1.24) + tf_transformations |
| Ericsii/FAST_LIO_ROS2 | `ros2` @ `2fffc570a25d0df172720bac034fbdb6a13d2162` | — | FAST-LIO 核心 port；可作为纯里程计 |
| wdc3iii/LIO-Localization | `main` @ `104663b2924a057fc6ec4e7301f1179c90244dcd` | — | Humble 工作区，Livox/Velodyne/Ouster + ICP；未深入审查 |
| deepglint/FAST_LIO_LOCALIZATION_HUMANOID | `humble` @ `df4772ec4797172430e7efe990711d09a529f4ad` | — | Humble 分支；偏人形机器人，未深入审查 |

## 2. 与 Isaac 接口的事实（已核对本仓库）

| 项 | 现状 | 适配影响 |
|---|---|---|
| `/lidar/points` | `sensor_msgs/PointCloud2`，10 Hz，射线投射；**字段仅 XYZ**（`cloud_data` 为 N×3 float） | 无逐点 time/ring/intensity；`default_handler` 需 XYZI（intensity 可能缺失），建议加零数学桥补 `intensity=0` 或改 handler |
| 逐点时间 | 无 | 不做运动畸变校正；10 Hz/0.6 m/s 下单帧≈6 cm 畸变；对 M5 0.2 m RMSE 阈值可接受但需记录 |
| `/imu` | 200 Hz（bridge 默认），best_effort，`imu_link`，含重力 | FAST-LIO 订阅常为 reliable → **QoS 不匹配**；改为 `SensorDataQoS` |
| 时钟 | `/clock` 仿真时钟；所有节点应 `use_sim_time=true` | `time_sync_en: false`；LiDAR/IMU 同源仿真时间 |
| 外参 | URDF：`imu_link` 固定在 `lidar_front_link`，origin `0 0 0 rpy 0 0 0` | `extrinsic_T=[0,0,0]`, `extrinsic_R=I`；frame 仍需映射 |
| 帧 | FAST-LIO 常用 `camera_init`/`body`；本仓库为 `map→odom→base_footprint` | 需 remap / TF 融合脚本；定位输出接 `map→odom` |
| 点云 QoS | bridge best_effort | `standard_pcl_cbk` 已用 `SensorDataQoS`（好） |

## 3. 代码级适配清单（已在临时副本中验证到“可编译前”）

临时目录：`/home/robot/m5_spike/ws/src/fast_lio_localization`（**未纳入仓库**）。

1. **去掉 Livox 依赖**：删 `livox_ros_driver2` 的 include/依赖/CustomMsg 订阅与 `avia_handler`，
   只保留 `PointCloud2` 路径；配置 `lidar_type: 0`（落入 `default_handler`）。
2. **PCL 依赖**：本机只有 `pcl_conversions`、没有 `pcl_ros`；改为直接 `find_package(PCL)`。
3. **ikd-Tree**：子模块需 `git clone -b fast_lio hku-mars/ikd-Tree`（已克隆到临时副本）。
4. **IMU QoS**：订阅改 `rclcpp::SensorDataQoS()`。
5. **点云字段**（未改源）：建议新增零数学桥节点，把 `/lidar/points` 的 XYZ 转成
   `PointXYZI`（intensity=0；可选补 ring/time=0），或把 `default_handler` 改成 `PointXYZ`。
6. **定位依赖**：首选候选的 `global_localization.py`/`global_map_publisher.py` 需要
   `open3d`、`tf_transformations`（本机未装）；可 `pip install`，或按 W5 计划切 PCL
   ICP/NDT 自研定位。
7. **地图**：`pcd_save_en: true` 跑 `narrow_passage_01` 建图 → `nav_ws_core/maps/narrow_passage_01/map.pcd`；
   定位配置 `map_file_path` 指向该 PCD。
8. **许可**：确认 GPLv2 文件与 `package.xml` BSD 声明不一致，报告对外发布前需法务/作者确认。

## 4. 建议的 1 天 spike 步骤

| 时段 | 动作 | 判据 |
|---|---|---|
| 0.5 h | 建独立 ASCII 工作区（`/home/robot/m5_spike/ws`）；克隆候选 + ikd-Tree | 源码/子模块齐全 |
| 2 h | 打上述适配补丁；构建 `fast_lio_localization` | `colcon build` 通过，`fastlio_mapping` 可执行 |
| 1 h | 零数学桥发布 `/lidar/points_intensity`（XYZ+intensity=0）；IMU QoS 校验 | `ros2 topic hz` 10/200 Hz，无丢帧 |
| 2 h | Isaac `narrow_passage_01` 建图（`pcd_save_en`）+ 保存 PCD | PCD 覆盖场景，无断裂 |
| 2 h | 30 s 定位跑测，和独立 GT `map→odom` 比较 | RMSE ≤0.2 m、末误差 ≤0.3 m、最大 ≤0.5 m、航向 ≤2° |
| 0.5 h | 失败切 PCL ICP/NDT；仍失败按 W5 计划降级并记录 | 不伪造成功 |

## 5. 编译状态与阻塞

- 已准备临时工作区并完成代码级适配补丁（§3.1~§3.4，未提交仓库）。
- **`colcon build` 未执行**：当前会话对“构建未由用户点名的第三方 GitHub 源码”有权限限制；
  需要用户明确授权（点名仓库/URL，例如
  `myeongw002/FAST_LIO_LOCALIZATION_ROS2 @ 3a005b9`）后才能继续。
- 本机缺失依赖：`livox_ros_driver2`、`pcl_ros`、`open3d`、`tf_transformations`；
  前两者已通过补丁绕开，后两者仅影响定位脚本（可替换为 PCL ICP/NDT）。

## 6. 参考来源

- https://github.com/myeongw002/FAST_LIO_LOCALIZATION_ROS2
- https://github.com/Smart-Wheelchair-RRC/FAST_LIO_LOCALIZATION2
- https://github.com/Ericsii/FAST_LIO_ROS2
- https://github.com/wdc3iii/LIO-Localization
- https://github.com/deepglint/FAST_LIO_LOCALIZATION_HUMANOID
- https://github.com/hku-mars/ikd-Tree
- 本仓库 `nav_ws_core/src/robot_sim/robot_sim/scripts/isaac_sim_bridge.py`、
  `nav_ws_core/src/robot_description/urdf/agl_chs_03_f01.urdf.xacro`
