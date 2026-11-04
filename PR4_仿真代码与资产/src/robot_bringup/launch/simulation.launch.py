"""E10：一条命令拉起整条链路（模型 + 仿真桥接 + 真值里程计 + TF + RViz）。

    cd ${NAV_WS}
    source /opt/ros/humble/setup.bash && source install/setup.bash
    ros2 launch robot_bringup simulation.launch.py use_sim_time:=true

这一条命令会起：

  1. **Isaac Sim（headless）** + `isaac_sim_bridge.py`：打开场景、订阅 `/cmd_vel`、
     发布 `/clock`、`/lidar/points`、`/imu`、`/sim/ground_truth`；
  2. **TF 树**（include `truth_tf.launch.py`）：`robot_state_publisher`（URDF →
     base_footprint→base_link→传感器）、`map→odom` 静态、真值适配节点（`/odom` + `odom→base_footprint`）；
  3. **RViz2**（`config/e10_robot.rviz`：模型 + TF + 点云 + 里程计轨迹，Fixed Frame=map）；
  4. 可选 **演示指令源**（`demo:=true`，默认开）：按 M1 要求发 0.1 m/s 前进/左移/原地转。

常用参数：
  sim:=false        只起 ROS 2 侧（仿真已经在跑时用）
  rviz:=false       不起 RViz（无显示环境 / 只跑数据时）
  demo:=false       不发内置演示指令（自己发 /cmd_vel）
  mode:=physics     真实物理驱动（IMU 有效；平移只有指令的 ~30%，见 docs/engineering/motion_check.md）
  mode:=kinematic   理想运动学推进（位姿与指令一致，IMU 无效）
  scene:=<绝对路径> 指定场景（默认 $NAV_WS/assets/scenes/e05_min_scene.usd 或当前目录）
  duration:=120     仿真侧运行时长（仿真秒）；**duration:=0 = 不限时长**（Ctrl-C 才停）
  legacy:=true      额外拉起 2026-10-23 的既有链路（它自己发 map→odom，勿与 truth:=true 同开）

与既有链路的互斥（E09 验收第 7 条）：默认不启动 legacy；两套 map→odom / odom→base 不能同时存在。
"""

import os

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription,
                            LogInfo, TimerAction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import PythonExpression
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import Command, PathJoinSubstitution

DEFAULT_WORKSPACE = os.environ.get("NAV_WS", os.getcwd())


def generate_launch_description() -> LaunchDescription:
    use_sim_time = LaunchConfiguration("use_sim_time")
    start_sim = LaunchConfiguration("sim")
    use_rviz = LaunchConfiguration("rviz")
    demo = LaunchConfiguration("demo")
    scene = LaunchConfiguration("scene")
    mode = LaunchConfiguration("mode")
    duration = LaunchConfiguration("duration")
    truth = LaunchConfiguration("truth")
    rviz_config = LaunchConfiguration("rviz_config")
    legacy = LaunchConfiguration("legacy")
    gui = LaunchConfiguration("gui")

    bridge = [FindPackageShare("robot_sim"), "/scripts/isaac_sim_bridge.py"]
    desc_share = FindPackageShare("robot_description")
    robot_description = ParameterValue(
        Command(["xacro ", PathJoinSubstitution([desc_share, "urdf", "agl_chs_03_f01.urdf.xacro"]),
                 " mesh_dir:=file://", PathJoinSubstitution([desc_share, "meshes"])]),
        value_type=str)
    sim_python = os.environ.get(
        "ISAAC_PYTHON", "python3.11")

    # “sim 且 gui” / “sim 且非 gui” 两个条件（launch 里用 PythonExpression 拼字符串）
    gui_cond = PythonExpression(["('", start_sim, "' == 'true') and ('", gui, "' == 'true')"])
    headless_cond = PythonExpression(
        ["('", start_sim, "' == 'true') and not ('", gui, "' == 'true')"])

    legacy_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("robot_a_sim_control"), "/launch/robot_a_truth_navigation.launch.py"]),
        launch_arguments={"use_sim_time": use_sim_time, "use_rviz": "false"}.items(),
        condition=IfCondition(legacy),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("sim", default_value="true",
                                  description="是否拉起 Isaac Sim 侧桥接进程"),
            DeclareLaunchArgument("rviz", default_value="true"),
            DeclareLaunchArgument("demo", default_value="true",
                                  description="是否发内置 M1 演示指令（0.1 m/s 前进/左移/原地转）"),
            DeclareLaunchArgument("truth", default_value="true",
                                  description="真值模式（本 launch 发 map→odom 与 odom→base_footprint）"),
            DeclareLaunchArgument("legacy", default_value="false",
                                  description="额外拉起既有 robot_a_sim_control 链路（与 truth 互斥）"),
            DeclareLaunchArgument("mode", default_value="kinematic",
                                  choices=["physics", "kinematic"]),
            DeclareLaunchArgument("gui", default_value="false",
                                  description="true=打开 Isaac Sim 窗口（可和 RViz 同时看）；false=headless"),
            DeclareLaunchArgument("duration", default_value="120",
                                  description="仿真侧运行时长（仿真秒）；0 = 不限时长，Ctrl-C 才停"),
            DeclareLaunchArgument(
                "scene",
                default_value=os.path.join(DEFAULT_WORKSPACE, "assets/scenes/e05_min_scene.usd")),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=os.path.join(DEFAULT_WORKSPACE, "config/e10_robot.rviz")),
            LogInfo(msg=["[E10] 一条命令启动：sim=", start_sim, " rviz=", use_rviz, " demo=", demo,
                         " truth=", truth, " mode=", mode, " scene=", scene]),

            # 1a) 仿真侧：**GUI 模式**（Isaac 窗口 + 视口点云），与 RViz 同时看
            ExecuteProcess(
                cmd=[sim_python, bridge,
                     "--scene", scene,
                     "--mode", mode,
                     "--duration", duration,
                     "--out", os.path.join(DEFAULT_WORKSPACE, "results/w3/e10"),
                     "--cmd-vel", "--sensors", "--truth", "--draw", "--gui"],
                output="screen",
                emulate_tty=True,
                condition=IfCondition(gui_cond),
            ),

            # 1b) 仿真侧：headless 模式（跑数据/自动化用）
            ExecuteProcess(
                cmd=[sim_python, bridge,
                     "--scene", scene,
                     "--mode", mode,
                     "--duration", duration,
                     "--out", os.path.join(DEFAULT_WORKSPACE, "results/w3/e10"),
                     "--cmd-vel", "--sensors", "--truth", "--draw"],
                output="screen",
                emulate_tty=True,
                condition=IfCondition(headless_cond),
            ),

            # 2) TF 树 + 真值里程计
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [FindPackageShare("robot_bringup"), "/launch/truth_tf.launch.py"]),
                launch_arguments={"use_sim_time": use_sim_time, "truth": truth}.items(),
            ),

            # 3) 演示指令源（等仿真起来再发）
            TimerAction(
                period=12.0,
                actions=[
                    ExecuteProcess(
                        cmd=["ros2", "run", "robot_sim", "e10_demo_driver.py",
                             "--ros-args", "-p", "use_sim_time:=true"],
                        output="screen",
                        condition=IfCondition(demo),
                    )
                ],
            ),

            # 4) RViz2
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=["-d", rviz_config],
                parameters=[{"use_sim_time": use_sim_time,
                             "robot_description": robot_description}],
                condition=IfCondition(use_rviz),
            ),

            legacy_launch,
        ]
    )

# 附：点云话题在契约 v1.1 统一为 /lidar/points（原 /points_raw 废弃）；
# 需要兼容旧脚本时给桥接加 --cloud-alias /points_raw（本 launch 默认不加）。
