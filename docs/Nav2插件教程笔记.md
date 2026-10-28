# Nav2 插件教程笔记（W2 交付：「Nav2 插件教程过完」）

> W2「Nav2 插件教程过完」的学习笔记，W5 C++ 插件骨架（`mpc_controller`）的依据；配套 docs/MPC公式化.md（§11 移植对应）。术语见 GLOSSARY.md，接口见 docs/接口契约.md。
>
> **版本口径（2026-10-28 实测）**：任务给的旧 URL `docs.nav2.org/plugin_tutorials/*` 已 404，现行目录为 `docs.nav2.org/rolling/tutorials/plugin_tutorials/*`（站点已按版本分档）。**教程正文是 rolling 版，本项目是 ROS 2 Humble——接口一律以 navigation2 `humble` 分支源码为准**，教程与 Humble 的差异在各节逐条标注（第 2 节末有专门对照）。

## 1. 教程通关清单

教程主线 = 写一个 pure pursuit 控制器（`nav2_pure_pursuit_controller`，配套全量样板在 `navigation2_tutorials` 仓库）。逐条通关记录：

| # | 步骤 | 教程原文做法 | 本项目 W5 落点（mpc_controller / MPC） |
|---|---|---|---|
| 1 | 建包 | 教程未单列建包步骤，直接以配套包为样板：`include/<pkg>/` + `src/` + 包根 plugin xml + CMakeLists + package.xml 五件结构 | `ros2 pkg create mpc_controller --build-type ament_cmake`；同包内还要带 `msg/`（`ObstacleState(.Array).msg` 按契约住本包，不新增第 8 包）→ package.xml 需额外加 rosidl 生成器与 `member_of_group`（见 §6） |
| 2 | 接口实现（教程 Step 1） | 建类 `nav2_pure_pursuit_controller::PurePursuitController` 继承 `nav2_core::Controller`，override 全部纯虚函数；算法集中在 `computeVelocityCommands()`；`configure()` 里用 `declare_parameter_if_not_declared` 声明参数（参数名 = `<插件ID>.<参数名>`，如 `FollowPath.desired_linear_vel`） | `mpc_controller::MpcController` 继承 `nav2_core::Controller`，**按 Humble 的 7 个方法实现**（§2）；`configure()` 声明 MPC 参数（N、dt、七项权重、ρ、d_safe、v_ref、SQP 次数）+ 底盘 limits（读底盘 YAML，方案见 §6.3）；`computeVelocityCommands()` 内跑「裁剪全局路径 → 装配 QP → OSQP → 取 u₀」 |
| 3 | 注册导出（教程 Step 2） | 四件套：①源文件末尾 `PLUGINLIB_EXPORT_CLASS(...)`；②包根 `*_plugin.xml`（`base_class_type="nav2_core::Controller"`）；③CMake `pluginlib_export_plugin_description_file(nav2_core ...)`；④package.xml `<export>` 里 `<nav2_core plugin="${prefix}/..."/>`。**③④缺一不可** | 原样照做，文件名 `mpc_controller_plugin.xml`；宏写在 `src/mpc_controller.cpp` 末行。详见 §3 |
| 4 | 构建验证（教程 Step 2-5） | `colcon build` 后 source，`ros2 plugin list` 应看到 `Plugin(... base='nav2_core::Controller')` | 同左；若本机 ros2cli 无 `plugin` 动词则改用：`find install -name "*plugin*.xml"` 查安装 + 启动 controller_server 看 configure 日志 + `ros2 param list /controller_server \| grep FollowPath`。**加载失败排查看 §6.4 坑表** |
| 5 | 参数配置（教程 Step 3） | `nav2_params.yaml` → `controller_server.ros__parameters`：`controller_plugins: ["FollowPath"]`，`FollowPath.plugin: "…" ` + 插件专属参数平铺在 `FollowPath:` 下 | 同结构：`FollowPath.plugin: "mpc_controller::MpcController"`，MPC 权重/底盘 limits 全放 `FollowPath:` 段；`controller_frequency: 20.0` 必须与契约 20 Hz / dt=0.05 一致（Humble 默认即 20.0）。样例见 §4.1 |
| 6 | 运行验证（教程 Step 4） | `ros2 launch nav2_bringup tb3_simulation_launch.py params_file:=…` → RViz「2D Pose Estimate」定位 →「Nav2 goal」发目标 → 机器人沿全局路径走 | 换成本项目 sim bringup（Isaac Sim / 降级 Gazebo）+ 同款 nav2_params；发目标 → `FollowPath` action 走起来 → `/cmd_vel` 有 20 Hz 输出。**W5 骨架验收 = 方案书「C++ 插件骨架（Nav2 加载成功）」**，先求「加载 + 空实现/直通逻辑不崩」，W6 才换真 MPC |

**教程（rolling）↔ Humble 差异（照抄会编译失败的点）**：

| rolling 教程写法 | Humble 实际 |
|---|---|
| `computeVelocityCommands(pose, velocity, goal_checker, transformed_global_plan, global_goal)`（5 参） | **3 参**：`(pose, velocity, goal_checker)`；路径变换由插件自己做 |
| `newPathReceived()` 响应新路径 | 无此方法 → 用 `setPlan(path)` |
| `cancel()` 处理取消 | 无此方法；取消由 server 处理（发零速） |
| 抛 `nav2_core::ControllerTFError` | Humble 的 `nav2_core/exceptions.hpp` **只有 `PlannerException`**，没有 `ControllerTFError` |

## 2. nav2_core::Controller 接口逐方法

来源：`navigation2/humble` 分支 `nav2_core/include/nav2_core/controller.hpp`（Humble 为准）。

```cpp
// 来源：humble/nav2_core/include/nav2_core/controller.hpp
using Ptr = std::shared_ptr<nav2_core::Controller>;
virtual ~Controller() {}

virtual void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr &,
                       std::string name,
                       std::shared_ptr<tf2_ros::Buffer>,
                       std::shared_ptr<nav2_costmap_2d::Costmap2DROS>) = 0;
virtual void cleanup() = 0;
virtual void activate() = 0;
virtual void deactivate() = 0;
virtual void setPlan(const nav_msgs::msg::Path & path) = 0;
virtual geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker) = 0;
virtual void setSpeedLimit(const double & speed_limit, const bool & percentage) = 0;
```

| 方法 | 语义 | controller_server 调用时机（humble 源码） | W5 备注 / 异常约定 |
|---|---|---|---|
| `configure(node_weak, name, tf, costmap_ros)` | 初始化插件：存 node/tf/costmap、**声明并读取参数** | server `on_configure` 时对每个 `controller_plugins` 条目调一次；`name` = 插件 ID（如 `FollowPath`），`costmap_ros` = server 自建的 **local_costmap**，`tf` = `costmap_ros_->getTfBuffer()` | 参数名必须拼 `name + "."` 前缀；node 是 **WeakPtr**，每次用 `.lock()`，勿存 shared_ptr（防节点泄漏）。此时生命周期节点正处 configure 态，是 declare 参数的唯一正确时机 |
| `activate()` | 激活发布器/起线程 | server `on_activate` 逐个调 | 本插件基本无 publisher 要激活；若有（诊断），放这里 |
| `deactivate()` | 停线程/停发布 | server `on_deactivate`（含取消、错误路径）逐个调 | server 随后会发零速 |
| `cleanup()` | 释放资源 | server `on_cleanup` 逐个调 | 置空指针、清缓存 |
| `setPlan(path)` | 接收全局规划器（NavFn）给的路径 | ①目标 `follow_path` action 首次接受时 `setPlannerPath()`；②有抢占新目标时 `updateGlobalPath()` 内；路径为空时 **server 已先抛** `PlannerException("Invalid path, Path is empty.")` | RPP 只是 `global_plan_ = path;`——W5 同样先存成员，变换/裁剪留到每拍做（路径会被抢占覆盖） |
| `computeVelocityCommands(pose, vel, goal_checker)` | 核心：给当前位姿/速度算最优指令，返回 `TwistStamped`（假设 plan 已 set） | 控制循环每拍（`rclcpp::WallRate(controller_frequency_)`，默认 20 Hz）：`getRobotPose`（取自 costmap，失败抛 `PlannerException("Failed to obtain robot pose")`）→ `progress_checker_->check()` → odom 速度（`nav_2d_utils`，过 `min_*_velocity_threshold` 阈值）→ 本方法 → `goal_checker` 判到达 → 发 `/cmd_vel` | 必须在一个控制周期（20 Hz → 50 ms）内返回，OSQP 求解预算放这里；返回值 server **只取 `.twist`**（header 丢弃，但建议仍填 base frame + 时间戳）。`goal_checker` 参数可忽略（教程示例即注释掉），到达判定归 server |
| `setSpeedLimit(limit, percentage)` | 限最大线速度：绝对 m/s（`percentage=false`）或最大速度百分比（`true`）；`nav2_costmap_2d::NO_SPEED_LIMIT` 哨兵 = 取消限速 | 不在控制循环里：server 订阅 `speed_limit` 话题（参数 `speed_limit_topic`，`nav2_msgs/msg/SpeedLimit`），回调把 limit 广播给**所有**已加载 controller | RPP 做法（humble 源码）：存 `base_desired_linear_vel_`，percentage 则 `base * limit / 100`，NO_SPEED_LIMIT 恢复基准值；角速度按比例带限。W5：把限速乘到 MPC 的 `v_max` box 约束上 |

**异常约定（Humble 关键坑）**：

- Humble `nav2_core/exceptions.hpp` **只定义了** `nav2_core::PlannerException : std::runtime_error`（教程/rolling 里的 `ControllerException`、`ControllerTFError` 在 Humble **不存在**，照抄 = 编译错）。
- server 控制循环的 catch 顺序：`catch (nav2_core::PlannerException & e)` → 走 `failure_tolerance` 宽限逻辑（容差内发零速继续，超限报 `"Controller patience exceeded"` 终止本 goal）；`catch (std::exception & e)` → 立即零速 + 终止本 goal。
- 因此 W5：**可恢复的失败（OSQP 不收敛）按 W2 回退策略返回零速命令 + 日志，不抛异常**（抛了会把 goal 杀掉，违背方案书「求解器回退」）；**不可恢复错误**（路径空、TF 变换失败）抛 `nav2_core::PlannerException`，借 failure_tolerance 通道（RPP 自身即抛 `PlannerException("Resulting plan has 0 poses in it.")`）。
- `setPlan`/`setSpeedLimit`/生命周期钩子里的错误：能记日志就记日志，别从这里抛终止整个 server。

## 3. pluginlib 注册三件套

### 3.1 导出宏（教程 Step 2-1）

```cpp
// 来源：教程 Step 2，写在 src/<your>_controller.cpp 的**末尾**
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_pure_pursuit_controller::PurePursuitController, nav2_core::Controller)
```

W5 落点：

```cpp
PLUGINLIB_EXPORT_CLASS(mpc_controller::MpcController, nav2_core::Controller)
```

### 3.2 plugin.xml（教程 Step 2-2，放包根目录）

```xml
<!-- 来源：教程 Step 2（原文文件名 pure_pursuit_controller_plugin.xml） -->
<library path="nav2_pure_pursuit_controller">
  <class type="nav2_pure_pursuit_controller::PurePursuitController"
         base_class_type="nav2_core::Controller">
    <description>This is pure pursuit controller</description>
  </class>
</library>
```

W5：`mpc_controller_plugin.xml`，`path` = `mpc_controller`（**不带 `lib` 前缀、不带 `.so`**，必须与 `add_library` 的目标名一致），`type` = `mpc_controller::MpcController`，`base_class_type` 写死 `nav2_core::Controller`。

### 3.3 CMakeLists.txt（教程 Step 2-3 + humble RPP 源码对照）

```cmake
# 来源：humble/nav2_regulated_pure_pursuit_controller/CMakeLists.txt（节选，Humble 风格）
find_package(ament_cmake REQUIRED)
find_package(nav2_common REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_util REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

nav2_package()                      # nav2_common 宏：统一编译选项
set(CMAKE_CXX_STANDARD 17)
include_directories(include)
set(library_name nav2_regulated_pure_pursuit_controller)

add_library(${library_name} SHARED src/regulated_pure_pursuit_controller.cpp)
ament_target_dependencies(${library_name} rclcpp geometry_msgs nav2_costmap_2d
  pluginlib nav_msgs nav2_util nav2_core tf2 tf2_geometry_msgs)

install(TARGETS ${library_name} ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include/)

ament_export_include_directories(include)
ament_export_libraries(${library_name})
ament_export_dependencies(...)

pluginlib_export_plugin_description_file(nav2_core nav2_regulated_pure_pursuit_controller.xml)  # ← 注册关键行
ament_package()
```

W5 差异：教程配套包是 rolling 写法（`target_link_libraries` 命名空间目标 + `nav2_ros_common`）——**`nav2_ros_common` 在 Humble 不存在，别抄**，用上面的 `ament_target_dependencies` 写法。可选：教程另有 `target_compile_definitions(... PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")`（防 pluginlib 拉 boost，Humble 非必需）。

### 3.4 package.xml（教程 Step 2-4 + humble RPP 源码对照）

```xml
<!-- 来源：humble/nav2_regulated_pure_pursuit_controller/package.xml（节选） -->
<buildtool_depend>ament_cmake</buildtool_depend>
<depend>nav2_common</depend>
<depend>nav2_core</depend>
<depend>nav2_util</depend>
<depend>nav2_costmap_2d</depend>
<depend>rclcpp</depend>
<depend>geometry_msgs</depend>
<depend>nav2_msgs</depend>
<depend>pluginlib</depend>
<depend>tf2</depend>
<depend>tf2_geometry_msgs</depend>

<export>
  <build_type>ament_cmake</build_type>
  <nav2_core plugin="${prefix}/nav2_regulated_pure_pursuit_controller.xml" />  <!-- ← 注册关键行 -->
</export>
```

W5 需再加：`rclcpp_lifecycle`、`tf2_ros`、`nav_msgs`、`std_msgs`；msg 同包时加

```xml
<buildtool_depend>rosidl_default_generators</buildtool_depend>
<exec_depend>rosidl_default_runtime</exec_depend>
<member_of_group>rosidl_interface_packages</member_of_group>
```

并注意：**CMake 宏（§3.3）与 package.xml export 标签（§3.4）两处都要写**——教程与 Nav2 自带插件（RPP humble）都是两处并存，只写一处是最常见的加载失败原因。

## 4. controller_server 集成

来源：humble `nav2_controller/src/controller_server.cpp` + `nav2_bringup/params/nav2_params.yaml`。

### 4.1 关键参数（controller_server 节点，humble）

| 参数 | 默认 / nav2_params 值 | 说明与 W5 落点 |
|---|---|---|
| `controller_plugins` | 默认 `{"FollowPath"}`；yaml 同 | 插件 ID 列表；ID 即参数命名空间、即 `setPlan`/`computeVelocityCommands` 的运行名 |
| `controller_plugins[i].plugin` | yaml：`"dwb_core::DWBLocalPlanner"` | 类型串经 `nav2_util::get_plugin_type_param` 读取 → `lp_loader_.createUniqueInstance(type)`；**W5 改为 `"mpc_controller::MpcController"`** |
| `controller_frequency` | 默认 **20.0**；bringup yaml 20.0 | 控制循环频率 = `/cmd_vel` 输出频率；**与契约 20 Hz、MPC dt=0.05 一一对应**，改了必须同步改 MPC dt（且与 `# dt 定死` 的契约冲突，故不改） |
| `progress_checker_plugin`（单数） | 默认 `"progress_checker"`；yaml：`nav2_controller::SimpleProgressChecker` | 每拍 `check(pose)`；`required_movement_radius: 0.5`、`movement_time_allowance: 10.0`；不动超时抛 `PlannerException("Failed to make progress")` → goal 失败。**不是插件的事，但调 MPC 时要意识到卡死会被它判死** |
| `goal_checker_plugins`（复数列表） | 默认 `{"goal_checker"}`；yaml：`general_goal_checker` → `SimpleGoalChecker`，`xy_goal_tolerance: 0.25`、`yaw_goal_tolerance: 0.25`、`stateful: True` | 到达判定在 server 层；**0.25/0.25 恰与 W2 到达口径（指标定义）一致**，MPC 不自己判到达 |
| `failure_tolerance` | 默认 0.0；bringup yaml 0.3 | 抛 `PlannerException` 后的宽限秒数；与 §2 回退策略直接相关 |
| `speed_limit_topic` | 默认 `"speed_limit"` | 触发 `setSpeedLimit()` 的话题 |
| `publish_zero_velocity` | 默认 true | 取消/出错/停用时补发零速 |
| `min_x/y/theta_velocity_threshold` | 0.0001（bringup：0.001/0.5/0.001） | odom 死区，进 `computeVelocityCommands` 的 `velocity` 已阈值化 |

### 4.2 每拍时序与 /cmd_vel 通路

```
bt_navigator: ComputePathToPose(planner_server → NavFn "GridBased")
            → FollowPath action("follow_path") → controller_server
目标接受: setPlannerPath → controllers_[id]->setPlan(path)
每拍 @ controller_frequency(20 Hz):
  costmap_ros_->isCurrent()? → updateGlobalPath()(抢占则重新 setPlan)
  → getRobotPose()(costmap, 失败抛 PlannerException)
  → progress_checker_->check(pose)
  → computeVelocityCommands(pose, odom_twist, goal_checker*)
  → goal_checker 到达? → break(成功)
  → vel_publisher_ 发 "cmd_vel"
发布: geometry_msgs::msg/Twist("cmd_vel", depth 1)，仅 activated 且有订阅者时；
      插件返回的 TwistStamped 只拷贝 .twist（header 丢弃）。
取消/异常/停用 → publishZeroVelocity()
```

- **`/cmd_vel` 契约吻合**：接口契约「/cmd_vel · geometry_msgs/Twist · 20 Hz · 发布方 controller_server」与 Humble 实现逐项一致——**插件不要自己发 `/cmd_vel`**，只管返回 `TwistStamped`，否则双发布打架。
- 速度上限另有 `speed_limit` 话题旁路（§2 末行），与底盘 limits（MPC box 约束）是两条独立通路，W6 可先不接。

### 4.3 与 costmap_ros 的关系

- server 在构造函数里自建 **局部代价地图**：`costmap_ros_ = std::make_shared<Costmap2DROS>("local_costmap", ns, "local_costmap")`，独立线程 `nav2_util::NodeThread` 转；`configure/activate/deactivate/cleanup` 与 server 生命周期联动。
- 传进插件 `configure()` 的就是它——**插件拿到的永远是 local costmap（滚动窗口），不是全局地图**；帧参数在 `local_costmap` 段（`global_frame: odom`、`robot_base_frame`、`rolling_window: true`、尺寸/分辨率同段）。
- server 自己用它：`getRobotPose()`、`getBaseFrameID()`/`getGlobalFrameID()` 变换目标位姿、`isCurrent()` 等待感知就绪（VoxelLayer 没更新前不放行控制）。
- footprint 归 costmap 配置（bringup：`robot_radius: 0.22`）——**W5 必须改成与底盘 YAML `footprint.radius`（契约 0.30 例）一致**，否则 costmap 碰撞检查与 MPC 软约束用两套半径，见 §5.4 坑。

## 5. costmap 使用要点（W5~6 ESDF/EDT 预备）

### 5.1 帧与取数

| 要点 | 内容 |
|---|---|
| 帧查询 | `costmap_ros_->getGlobalFrameID()`（local costmap 通常 = odom）、`getBaseFrameID()`（base_footprint/base_link）。**别硬编码帧名** |
| 取指针 | `configure()` 里 `costmap_ = costmap_ros_->getCostmap();`（`nav2_costmap_2d::Costmap2D*`，RPP 同款）；local costmap 常驻，指针可存成员 |
| 位姿来源 | `computeVelocityCommands` 的 `pose` 参数已是 server 用 costmap 取好的机器人位姿（global frame），**不必自己查 TF** |
| 全局路径变换 | Humble 的路径变换责任在插件自己。RPP `transformGlobalPlan(pose)` 范式（humble 源码）：①找全局路径上离机器人最近点（`nav2_util::geometry_utils::min_by`/`first_after_integrated_distance`）；②丢弃超出局部代价地图范围的点；③`tf_->transform(...)` 到 base frame；④裁掉已走过的前缀（`global_plan_.poses.erase(...)`）；裁空抛 `PlannerException`。**W5 的 MPC 参考轨迹输入照此办理** |

### 5.2 Costmap2D API 速查

```cpp
// 常用（来源：nav2_costmap_2d::Costmap2D，humble）
bool worldToMap(double wx, double wy, unsigned int & mx, unsigned int & my); // false = 出窗口
void mapToWorld(unsigned int mx, unsigned int my, double & wx, double & wy);
unsigned char getCost(unsigned int mx, unsigned int my);
unsigned int getIndex(unsigned int mx, unsigned int my);
unsigned int getSizeInCellsX() / getSizeInCellsY();
double getResolution();          // m/cell
double getOriginX() / getOriginY();
```

`worldToMap` 返回 false（查询点在窗口外）必须处理：夹到窗口边缘并给保守代价，或直接判不可行——**MPC 预测点跑出局部窗口是常态**（N=40×0.05s×0.6m/s ≈ 1.2 m 视界，窗口半宽要 ≥ 该值 + 余量，否则远端预测步全落在窗外）。

### 5.3 代价语义（来源：humble `nav2_costmap_2d/cost_values.hpp`，原文常量）

```cpp
static constexpr unsigned char NO_INFORMATION = 255;
static constexpr unsigned char LETHAL_OBSTACLE = 254;
static constexpr unsigned char INSCRIBED_INFLATED_OBSTACLE = 253;
static constexpr unsigned char MAX_NON_OBSTACLE = 252;
static constexpr unsigned char FREE_SPACE = 0;
```

| 值 | 含义 | EDT 用法 |
|---|---|---|
| 0 | FREE_SPACE | 非障碍 |
| 1~252 | 膨胀层衰减代价（离障碍越近越大） | **不要**当障碍——EDT 输入只用二值图 |
| 253 | INSCRIBED（底盘中心压上必撞） | 视作障碍源 |
| 254 | LETHAL（障碍本体） | EDT 的障碍源点 |
| 255 | NO_INFORMATION（未知） | 策略待定：按障碍保守处理 or 忽略（`isTrackingUnknown()` 可查 costmap 是否跟踪未知） |

### 5.4 局部窗口 → 栅格 EDT（W5~6 ESDF 落点，对齐公式化 §5.1）

1. **输入**：local costmap 整窗（滚动窗口本身即「局部窗口」，无需再切）；二值化 = `{253, 254}`（+ 未知按策略）→ 障碍点。
2. **一次距离变换**得每个 cell 到最近障碍的欧氏距离（EDT）；cell 距离 × `getResolution()` → 米。C++ 选项：OpenCV `cv::distanceTransform`（DIST_L2）或自写一维 Felzenszwalb EDT（~50 行零依赖，离线可编，适合入仓 vendor）。
3. **查询**：`worldToMap` → `getCost` 取 EDT 栅格值，一阶双线性插值得 d(p)；∇d 用数值梯度（口径与 W2 scipy sanity 脚本一致，见公式化 §5.1——该脚本正是为此排雷）。
4. **软约束代入**：`d_j(p̂_k) + ∇d_j(p̂_k)ᵀ(p_k − p̂_k) ≥ d_safe − s_{k,j}`（公式化 §5），静态走 EDT、动态走 `/mpc/obstacles` 预测位置，**同一条代码路径**。
5. **更新节奏**：costmap 由独立线程更新，EDT 至少每拍重算（或按层版本号缓存）；算完立刻用，别跨拍缓存 cell 坐标（窗口在滚动）。
6. **半径单一真源**：`r_r` 取 costmap 内切半径/footprint（`costmap_ros_->getRobotFootprint()`、`getLayeredCostmap()->getInscribedRadius()/getCircumscribedRadius()`）**或**底盘 YAML `footprint.radius`，二选一并让 `local_costmap.robot_radius` 与之相等——三个数必须同源，否则 costmap 说安全、MPC 说危险（或反之）。

## 6. W5 C++ 骨架清单（可执行 checklist）

### 6.1 文件清单

| 文件 | 放什么 |
|---|---|
| `mpc_controller/package.xml` | §3.4 依赖 + **两处导出**；msg 同包的 rosidl 三行（`rosidl_default_generators/runtime`、`member_of_group`） |
| `mpc_controller/CMakeLists.txt` | §3.3 Humble 模板：`nav2_package()` + `add_library(SHARED)` + `ament_target_dependencies` + install 库/头 + `pluginlib_export_plugin_description_file(nav2_core mpc_controller_plugin.xml)`；若 msg 同包加 `rosidl_generate_interfaces(...)`（注意与库目标的头依赖顺序） |
| `mpc_controller/mpc_controller_plugin.xml`（包根） | `<library path="mpc_controller">` + `type="mpc_controller::MpcController"` + `base_class_type="nav2_core::Controller"` |
| `mpc_controller/include/mpc_controller/mpc_controller.hpp` | 类声明：7 个 override + 成员——`node_`(WeakPtr)、`tf_`、`costmap_ros_`/`costmap_`、`plugin_name_`、`logger_`/`clock_`、`global_plan_`、MPC 参数成员（N/dt/七权重/ρ/d_safe/v_ref）、底盘 limits 成员、OSQP 求解器句柄、`std::mutex`（护障碍列表） |
| `mpc_controller/src/mpc_controller.cpp` | 7 方法实现 + 末行 `PLUGINLIB_EXPORT_CLASS(mpc_controller::MpcController, nav2_core::Controller)`；`configure()` = 声明/读参数（`nav2_util::declare_parameter_if_not_declared`，include `nav2_util/node_utils.hpp`） |
| `mpc_controller/src/qp_builder.cpp`（可拆） | numpy `P,q,A,l,u` 装配逐行移植（Eigen 稠密，200 变量量级，公式化 §7） |
| `mpc_controller/src/distance_field.cpp`（可拆） | §5.4 costmap → 二值 → EDT → 插值查询 |
| `mpc_controller/msg/ObstacleState.msg`、`ObstacleStateArray.msg` | 契约原文（id/position/velocity/radius/stamp，odom 帧） |
| （W6）`src/obstacle_tracker_node.cpp` | 障碍跟踪节点，暂住本包（ADR-0003）；订阅 `/lidar/points` + `/odom` → `/mpc/obstacles` |
| nav2_params.yaml `controller_server.FollowPath` 段 | §6.3 |

### 6.2 W2/W4 Python 原型 → W5 C++ 对应（移植不重排公式）

| W2/W4（prototype/mpc_numpy，公式化） | W5 落点 |
|---|---|
| `osqp` Python 包 | osqp C++ 库（同一 solver 语义，方案书「MPC 求解」）。**坑**：osqp 非 rosdep Humble 包——vendor 源码入 `third_party/` 用 `add_subdirectory`（离线可编，别用需联网的 FetchContent）；具体取舍 W5 首日定 |
| numpy 装配 P,q,A,l,u | Eigen 稠密装配，**装配顺序/权重语义/½ 因子逐行对应** |
| N=40、dt=0.05 | `configure()` declare `horizon_N`(40)、`dt`(0.05)；dt 与 `controller_frequency=20` 强绑定（改 frequency = 违契约） |
| 七项代价（q_lat 4.0/q_long 0.4/q_θ 1.5、R、R_Δ、ρ=80、Q_T=2×） | 全部 `declare_parameter_if_not_declared`，参数名挂 `FollowPath.` 下，默认值 = W2 定值（公式化 §10） |
| 障碍软约束（d_safe=0.20、L1 slack、SQP 2 次） | 同名参数 + QP builder |
| 解析距离场（调参形态） | costmap 局部窗口 EDT（§5.4），**动态障碍不走 costmap**，仍走 `/mpc/obstacles` 预测（公式化 §5.2 实现口径） |
| 场景脚本合成障碍状态 | 订阅 `mpc_controller/ObstacleStateArray`（odom 帧），回调与控制循环共享处加 mutex；W5 骨架期可先空实现 |
| limits 读底盘 YAML（diff→vy_max=0） | `configure()` 读入（§6.3）；`type` 判断逻辑原样保留（迁移冒烟零改动落点） |
| 求解回退：不收敛 → 本拍零速 + 日志、次拍重试、连 5 拍报警、**不换控制器** | `computeVelocityCommands` 返回 u=0 的 `TwistStamped` + RCLCPP_WARN/ERROR，**不抛异常**（§2 异常约定）；报警计数可映射诊断话题 |
| 完美运动学 rollout | `velocity` 参数即真实 odom 反馈（server 已给） |
| 到达判定 0.25 m / 0.25 rad | 不自己实现——server `goal_checker` 已是同口径（§4.1） |

### 6.3 底盘 YAML 参数读入（两种方案）

| 方案 | 做法 | 取舍 |
|---|---|---|
| A（起步推荐） | chassis schema（`type/footprint.radius/limits.*`）作为 `FollowPath:` 子键写进 nav2_params.yaml，`configure()` declare | 与教程完全一致、零新依赖；**W5 骨架期够用** |
| B（跨节点共享时升） | declare 一个 `chassis_yaml` 路径参数，插件内 yaml-cpp 解析 | 贴契约「底盘描述文件共享给 driver/safety_stop」的形态；多一个 `yaml-cpp` 依赖，W6 再升 |

### 6.4 常见坑（插件加载失败排查 + 编译/运行）

| # | 症状 | 原因 → 处理 |
|---|---|---|
| 1 | 启动报 `According to the loaded plugin descriptions the class ... does not exist` | 注册三件套缺件：查 ①`PLUGINLIB_EXPORT_CLASS` 宏是否在**编进库的 .cpp** 末尾；②plugin.xml 的 `type` 与 C++ 全限定名逐字符一致、`base_class_type="nav2_core::Controller"`；③CMake 宏 + package.xml export **两处**都写了（§3.3/3.4） |
| 2 | 报 `Could not load library .../libmpc_controller.so ... No such file` | plugin.xml `path` 与 `add_library` 目标名不一致（path 不带 `lib` 前缀/`.so`）；或该包没 build |
| 3 | 改了 xml/代码不生效 | 忘 `source install/setup.bash`；或 workspace overlay 取反（`echo $AMENT_PREFIX_PATH` 查加载的是不是旧 install；`find install -name "*plugin*.xml"` 比对） |
| 4 | 参数改了插件读不到 | 参数名漏 `plugin_name_` 前缀（正确：`FollowPath.lat_weight`，YAML 也必须在 `controller_server.FollowPath` 下）；或读参不在 `configure()`（在构造函数里 declare 时 `node_` 还没绑定，必崩） |
| 5 | 编译错：5 参 `computeVelocityCommands` 没有 / `ControllerTFError` 未声明 | 照抄了 **rolling 教程**。Humble 是 3 参、无 Controller 异常类（§1 差异表）——一切以 humble 分支源码为准 |
| 6 | `ament_target_dependencies` 找不到包 / `nav2_ros_common` 报错 | 照抄了 rolling 版教程包 CMake。Humble 用 §3.3 模板 |
| 7 | 插件能加载但 `/cmd_vel` 不动 | ①`controller_plugins` 列表里没这个 ID 或 `.plugin` 类型串拼错；②`computeVelocityCommands` 内异常被吞（看 failure_tolerance 日志）；③costmap `isCurrent()` 未就绪（传感器话题没配）；④自己发了 `/cmd_vel` 又只发一半——**发布归 server** |
| 8 | 运行时偶发卡顿/抖动 | 单拍超 50 ms（OSQP 迭代/SQP 2 次超预算）→ server 打 `Rate of controller is too low`；把求解预算计入并保留热启动（上一拍解 shift） |
| 9 | 生命周期态语义 | 插件**不是**生命周期节点，只是被 server 钩子驱动：`configure` 里能建订阅，**lifecycle publisher 必须 `activate()` 里 `on_activate()`**；`deactivate` 后别再发 |
| 10 | msg 与库同包编译顺序问题 | `rosidl_generate_interfaces` 先于库目标、库 `ament_target_dependencies` 加上本包生成的消息 target；忘加 `member_of_group` 则下游用 msg 报错 |
| 11 | 动态障碍数据竞争 | 控制循环（server 执行回调线程）读障碍列表 vs 订阅回调（executor 线程）写——**必须 mutex**，别无锁赌时序 |

W5 验收动作（方案书）：`colcon build` 通过 → `ros2 plugin list`/日志确认加载 → 仿真发目标，插件被调用（日志可见）→ 空实现也能走完 FollowPath（不追求避障效果，那是 W6）。

## 7. 术语对照（教程 ↔ GLOSSARY）

| Nav2 / 教程用语 | 本项目用语（GLOSSARY/契约） | 注意 |
|---|---|---|
| controller / controller plugin（教程 index 说 "local trajectory planner"） | **自研局部控制器**（`mpc_controller` Nav2 插件） | **勿写「局部规划器」**——GLOSSARY 明令该词是「障碍跟踪」的 avoid 项；教程的 "planner" 字样仅指全局侧 |
| planner plugin / global planner（本项目用 NavFn，yaml ID `GridBased`） | 全局规划（复用 Nav2，不自研） | 方案书决策：只自研控制器插件 |
| controller_server | Nav2 控制器服务器（复用框架） | 我们的边界是插件类，不含 server |
| costmap / local costmap / costmap layer（Obstacle/Voxel/Inflation） | 局部代价地图；感知喂给它的链路属**底盘感知**，不属算法通路的障碍决策 | costmap 障碍层 ≠ **障碍跟踪**：动态避障输入走 `/mpc/obstacles`（ADR-0003），costmap 只喂静态 EDT |
| obstacle layer 视角的「障碍」 | **障碍跟踪**（`ObstacleState` 列表，odom 帧） | 打分通路不得用仿真真值替代跟踪器 |
| goal checker / 到达判定 | 复用 server 判定（0.25 m/0.25 rad，同指标口径） | 自己不实现第二套 |
| speed limit / 底盘 limits（max_vel 等） | **底盘抽象层**：参数化底盘描述 YAML（`limits.*`、`footprint.radius`、`type`） | `diff → vy_max=0` 是迁移冒烟零改动落点 |
| `cmd_vel` 输出 | `/cmd_vel` 契约 20 Hz → 底盘抽象层 | 发布方 = controller_server |
| footprint / robot_radius | 底盘外形（与**真值几何**的仿真外形同源校准） | 三处半径必须一致（§5.4-6） |
| Nav2 recovery behaviors / safety 相关节点 | **安全急停层**（独立节点，0.15 m 硬停） | 两者无关：不拿 Nav2 行为树恢复机制当急停层，急停层也不进 Nav2 |
| behavior tree / bt_navigator | 复用（不自研） | 本插件只接 FollowPath 这一段 |

## 资料来源

1. 插件教程总目录（现行）：https://docs.nav2.org/rolling/tutorials/plugin_tutorials/ （旧 URL `https://docs.nav2.org/plugin_tutorials/index.html` 实测 404，2026-10-28）
2. 主线教程 Writing a New Controller Plugin（现行，rolling 版正文）：https://docs.nav2.org/rolling/tutorials/plugin_tutorials/writing_new_controller_plugin/writing_new_controller_plugin/
3. 站点入口（发现版本分档）：https://docs.nav2.org/rolling/
4. `nav2_core` 接口头（humble 分支，接口真源）：
   - https://github.com/ros-navigation/navigation2/tree/humble/nav2_core/include/nav2_core （controller.hpp / exceptions.hpp / goal_checker.hpp）
   - https://raw.githubusercontent.com/ros-navigation/navigation2/humble/nav2_core/include/nav2_core/controller.hpp
   - https://raw.githubusercontent.com/ros-navigation/navigation2/humble/nav2_core/include/nav2_core/exceptions.hpp
   - https://raw.githubusercontent.com/ros-navigation/navigation2/humble/nav2_core/include/nav2_core/goal_checker.hpp
5. controller_server 集成（humble）：https://raw.githubusercontent.com/ros-navigation/navigation2/humble/nav2_controller/src/controller_server.cpp
6. 官方插件样板（humble，接口/构建对照基准）：https://raw.githubusercontent.com/ros-navigation/navigation2/humble/nav2_regulated_pure_pursuit_controller/ （CMakeLists.txt、package.xml、src/regulated_pure_pursuit_controller.cpp）
7. 参数样例（humble）：https://raw.githubusercontent.com/ros-navigation/navigation2/humble/nav2_bringup/params/nav2_params.yaml
8. 代价常量（humble）：https://raw.githubusercontent.com/ros-navigation/navigation2/humble/nav2_costmap_2d/include/nav2_costmap_2d/cost_values.hpp
9. 教程配套包 navigation2_tutorials（`nav2_pure_pursuit_controller`，rolling 分支，注意其 CMake 的 `nav2_ros_common` 不适用 Humble）：https://github.com/ros-navigation/navigation2_tutorials
10. 备选旧文档（Foxy/Galactic 时代 "Writing a New Controller Plugin"，navigation.ros.org）——本次未取到可访问镜像；Humble 侧已用 humble 分支源码 + 现行教程交叉核实，无缺口。
