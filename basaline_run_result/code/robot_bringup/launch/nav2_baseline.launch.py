"""9-1/9-2 基线本地化启动：Nav2（DWB / MPPI / TEB）+ 共享平面限速 + 可选 RViz。

    ros2 launch robot_bringup nav2_baseline.launch.py baseline:=dwa
    ros2 launch robot_bringup nav2_baseline.launch.py baseline:=mppi rviz:=true
    # 自研 MPC（需先 source ~/mpc_build_ws/install/setup.bash）
    ros2 launch robot_bringup nav2_baseline.launch.py baseline:=mpc
    # TEB 未安装/未验证前会直接报错退出（不静默降级）

约定（与 docs/基线调参日志.md 一致）：
  * 共享配置一律来自 config/nav2_common.yaml（costmap / 全局规划器 / footprint / 限速参数）；
  * 控制器参数来自 config/baselines/<baseline>.yaml；
  * 速度通路：controller_server 输出 /cmd_vel_raw → flat_speed_limiter → /cmd_vel（唯一出口）；
  * 本 launch **不启动 Isaac**：仿真由 robot_bringup/simulation.launch.py 或
    robot_sim/isaac_sim_bridge.py 提供（/clock、/lidar/points、/odom、/tf）。
"""

import os

import yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_prefix, get_package_share_directory

BASELINES = ("dwa", "mppi", "teb", "mpc")

# 多个控制器可以同时注册在同一个 controller_server 上，由 FollowPath goal 的
# `controller_id` 选择。这样"一个场景 = 一次 Isaac 启动"就能跑完三个基线，
# 工作集 75 run / 正式 300 run 的墙钟时间能从 3 小时量级降到 1 小时量级。
# 代价：controller_server 层面的参数（频率、速度阈值、failure_tolerance、两个 checker）
# 只能有一份，所以下面这份 SHARED_CONTROLLER_PARAMS 是**唯一**来源，三个基线共用。
SHARED_CONTROLLER_PARAMS = {
    "controller_frequency": 20.0,
    "min_x_velocity_threshold": 0.001,
    "min_y_velocity_threshold": 0.001,     # 全向：横移速度必须进里程计
    "min_theta_velocity_threshold": 0.001,
    "failure_tolerance": 0.5,
    "progress_checker_plugins": ["progress_checker"],
    "goal_checker_plugins": ["general_goal_checker"],
    "progress_checker": {
        "plugin": "nav2_controller::SimpleProgressChecker",
        "required_movement_radius": 0.5,
        "movement_time_allowance": 10.0,
    },
    "general_goal_checker": {
        "plugin": "nav2_controller::SimpleGoalChecker",
        "xy_goal_tolerance": 0.25,          # 契约到达判定（0.25 m）
        "yaw_goal_tolerance": 0.25,         # 契约到达判定（0.25 rad）
        "stateful": True,
    },
}
# 基线名 → 参数文件名。**文件名必须与这里的映射一致**（曾经出现 dwa vs dwb.yaml 不一致，
# launch 直接找不到参数文件而失败，见 docs/9-1工程执行台账.md §G.2 #2）。
BASELINE_FILES = {"dwa": "dwa.yaml", "mppi": "mppi.yaml",
                  "teb": "teb.yaml", "mpc": "mpc.yaml"}
# mpc_controller 是自研方法方，不进三基线对比表，但跑同一套执行层与评分口径。


def _merge_controllers(share, baselines):
    """把各 baseline YAML 的 FollowPath 段挂到以基线名为键的位置，共用一份 server 参数。"""
    merged = dict(SHARED_CONTROLLER_PARAMS)
    merged["controller_plugins"] = []
    for b in baselines:
        path = os.path.join(share, "config", "baselines", BASELINE_FILES[b])
        doc = yaml.safe_load(open(path, encoding="utf-8"))
        per = doc["controller_server"]["ros__parameters"]
        follow = per.pop("FollowPath")
        merged[b] = follow
        merged["controller_plugins"].append(b)
        # 各 baseline YAML 里重复写 server 级参数容易漂移，这里做一致性校验
        for k, v in per.items():
            if k in SHARED_CONTROLLER_PARAMS and SHARED_CONTROLLER_PARAMS[k] != v:
                raise RuntimeError(
                    "%s 里的 %s=%r 与共享值 %r 不一致；多控制器同栈时 server 级参数只能有一份，"
                    "请改 SHARED_CONTROLLER_PARAMS 或删掉 YAML 里的重复项"
                    % (b, k, v, SHARED_CONTROLLER_PARAMS[k]))
    return {"controller_server": {"ros__parameters": merged}}


def _setup(context, *args, **kwargs):
    baseline = LaunchConfiguration("baseline").perform(context)
    # launch 的 LaunchConfiguration 一律是字符串；Nav2 节点里 use_sim_time 是 bool，
    # 直接传 "true" 会被 rclcpp 拒绝（InvalidParameterTypeException → 节点 abort）。
    use_sim_time = LaunchConfiguration("use_sim_time").perform(context) == "true"
    use_rviz = LaunchConfiguration("rviz").perform(context)
    use_bt = LaunchConfiguration("bt").perform(context)
    log_level = LaunchConfiguration("log_level").perform(context)
    map_yaml = LaunchConfiguration("map").perform(context).strip()
    if baseline not in BASELINES:
        raise RuntimeError("baseline 必须是 %s 之一，收到 %r" % (BASELINES, baseline))

    share = get_package_share_directory("robot_bringup")
    ctrls_arg = LaunchConfiguration("controllers").perform(context).strip()
    controllers = [c.strip() for c in ctrls_arg.split(",") if c.strip()] if ctrls_arg else [baseline]
    for c in controllers:
        if c not in BASELINES:
            raise RuntimeError("controllers 里的 %r 不在 %s 中" % (c, BASELINES))
    common = os.path.join(share, "config", "nav2_common.yaml")
    ctrl = os.path.join(share, "config", "baselines", BASELINE_FILES[baseline])
    merged = _merge_controllers(share, controllers)
    if not os.path.isfile(ctrl):
        raise RuntimeError("找不到基线参数文件：%s" % ctrl)

    if "teb" in controllers:
        # TEB 来自 ASCII 镜像构建（~/teb_build_ws）+ 解包到本地的 libg2o（~/teb_prebuilt）
        try:
            get_package_prefix("teb_local_planner")
        except Exception:
            raise RuntimeError(
                "找不到 teb_local_planner。请先：\n"
                "  export LD_LIBRARY_PATH=$HOME/teb_prebuilt/opt/ros/humble/lib/x86_64-linux-gnu:$LD_LIBRARY_PATH\n"
                "  source ~/teb_build_ws/install/setup.bash\n"
                "（构建说明见 docs/9-1工程执行台账.md §I）")

    # M2 采集口径：runner.run_nav2 直接驱动 planner_server 的 compute_path_to_pose 与
    # controller_server 的 follow_path 两个 action，不经过 BT——避免 BT 默认 xml 里
    # 的 recovery 节点依赖 behavior_server（基线实验不需要恢复行为，超时即判失败）。
    # 需要完整 NavigateToPose 时才用 bt:=true。
    nodes_pre = []
    if "mpc" in controllers:
        # 自研 MPC：插件来自 ASCII 镜像构建（~/mpc_build_ws），必须由调用方先 source
        try:
            get_package_prefix("mpc_controller")
        except Exception:
            raise RuntimeError(
                "找不到 mpc_controller 插件包。请先执行：\n"
                "  nav_ws_core/scripts/build_mpc_ascii.sh\n"
                "  source ~/mpc_build_ws/install/setup.bash\n"
                "（原因：工作区路径含中文会让 rosidl 生成接口失败，见 "
                "docs/与算法侧沟通_MPC构建.md）")
        nodes_pre.append(
            # 动态障碍心跳：mpc_controller 订阅 /mpc/obstacles（ObstacleStateArray），
            # **失联超过 obstacle_timeout（0.5 s）就输出零速**（实测：把 enabled 设成
            # False 会让 MPC 立刻"心跳失效→零速"，机器人一动不动、日志只有
            # "Failed to make progress"）。所以必须打开，用空数组表示"无动态障碍"。
            Node(package="mpc_controller", executable="mock_obstacle_heartbeat.py",
                 name="mock_obstacle_heartbeat", output="screen",
                 parameters=[{"use_sim_time": use_sim_time, "topic": "/mpc/obstacles",
                              "enabled": True}]))

    managed = ["controller_server", "planner_server"]
    if map_yaml:
        if not os.path.isfile(map_yaml):
            raise RuntimeError("map 参数指向的文件不存在：%s" % map_yaml)
        # 标准通路：nav2_map_server 加载 PGM+YAML 并发布 /map（transient_local）。
        # 不用 driver 自建 OccupancyGrid 直发——dead_end 那类带旋转 world_pose 的实例上
        # 全局 static_layer 收到地图却不生效，机器人穿墙（见台账 §L.3）。
        managed.insert(0, "map_server")
        nodes_pre.append(
            Node(package="nav2_map_server", executable="map_server", name="map_server",
                 output="screen",
                 parameters=[{"use_sim_time": use_sim_time, "yaml_filename": map_yaml,
                              "topic_name": "/map", "frame_id": "map"}]))
    if use_bt == "true":
        managed.append("bt_navigator")

    nodes = nodes_pre + [
        LogInfo(msg="[9-1] baseline=%s controllers=%s 共享配置=%s 控制器配置=%s bt=%s"
                % (baseline, controllers, common, ctrl, use_bt)),
        # 共享执行层第一级：平面限速（形态约束，四控制器同等生效）
        Node(package="robot_bringup", executable="flat_speed_limiter.py",
             name="flat_speed_limiter", output="screen", parameters=[common]),
        # 共享执行层第二级：安全层（安全事件 → 清零 + /safety/stop 上升沿）
        # 底盘只订阅 /cmd_vel，所以它必须是 `/cmd_vel` 的唯一发布者。
        Node(package="robot_bringup", executable="safety_supervisor.py",
             name="safety_supervisor", output="screen", parameters=[common]),
        Node(package="nav2_controller", executable="controller_server", output="screen",
             remappings=[("cmd_vel", "/cmd_vel_raw")],       # 限速前，先进入 limiter
             # log_level=debug 时把控制器的内部判定分支打出来（TEB 这类第三方插件
             # 抛的异常有时只在 debug 里看得到）
             arguments=["--ros-args", "--log-level", log_level],
             # 注意：内联 dict 是"参数名→值"，不能写成 YAML 文件的
             # {节点名: {ros__parameters: {...}}} 层级，否则一个都不会生效
             # （实测表现为 controller_server 用默认值、报 "No critics defined for FollowPath"）
             parameters=[common, merged["controller_server"]["ros__parameters"],
                         {"use_sim_time": use_sim_time}]),
        Node(package="nav2_planner", executable="planner_server", output="screen",
             parameters=[common, {"use_sim_time": use_sim_time}]),
        Node(package="nav2_lifecycle_manager", executable="lifecycle_manager",
             name="lifecycle_manager_navigation", output="screen",
             parameters=[{"use_sim_time": use_sim_time, "autostart": True,
                          "bond_timeout": 0.0, "node_names": managed}]),
    ]
    if use_bt == "true":
        nodes.append(Node(package="nav2_bt_navigator", executable="bt_navigator",
                          output="screen", parameters=[common, {"use_sim_time": use_sim_time}]))
    if use_rviz == "true":
        nodes.append(Node(package="rviz2", executable="rviz2", name="rviz2", output="screen",
                          arguments=["-d", os.path.join(share, "config", "e10_robot.rviz")]))
    return nodes


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument("baseline", default_value="dwa",
                              description="dwa | mppi | teb | mpc"),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("rviz", default_value="false"),
        DeclareLaunchArgument("controllers", default_value="",
                              description="逗号分隔的控制器列表（dwa,mppi,teb）；空=只用 baseline。"
                                          "多个控制器注册在同一个 controller_server 上，"
                                          "由 FollowPath 的 controller_id 选择"),
        DeclareLaunchArgument("map", default_value="",
                              description="Nav2 地图 yaml 路径（PGM+YAML）。给了就用 nav2_map_server "
                                          "发布 /map 给 static_layer；空 = 由采集器自己发"),
        DeclareLaunchArgument("log_level", default_value="info",
                              description="controller_server 日志级别（info|debug）"),
        DeclareLaunchArgument("bt", default_value="false",
                              description="true=另起 bt_navigator（NavigateToPose）；false=只起 "
                                          "controller_server + planner_server（M2 采集口径）"),
        OpaqueFunction(function=_setup),
    ])
