"""启动自建底盘、Isaac 桥接、真值里程计、TF 与 RViz。用法及环境见 nav_ws_core/readme.md。"""

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
    gui = LaunchConfiguration("gui")
    nav2 = LaunchConfiguration("nav2")
    nav2_delay = LaunchConfiguration("nav2_delay")
    log_level = LaunchConfiguration("log_level")
    controllers = LaunchConfiguration("controllers")
    map_yaml = LaunchConfiguration("map")

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
    # M2 套件：nav2 != none 时把 Nav2 基线栈接进同一条 launch；此时必须关掉内置演示指令，
    # 否则 /cmd_vel 会有两个发布者（演示驱动 + Nav2），采集到的轨迹不可解释。
    nav2_on = PythonExpression(["'", nav2, "' != 'none'"])
    demo_cond = PythonExpression(["('", demo, "' == 'true') and ('", nav2, "' == 'none')"])

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("sim", default_value="true",
                                  description="是否拉起 Isaac Sim 侧桥接进程"),
            DeclareLaunchArgument("rviz", default_value="true"),
            DeclareLaunchArgument("demo", default_value="true",
                                  description="是否发内置 M1 演示指令（0.1 m/s 前进/左移/原地转）；nav2!=none 时强制关闭"),
            DeclareLaunchArgument("truth", default_value="true",
                                  description="真值模式（本 launch 发 map→odom 与 odom→base_footprint）"),
            DeclareLaunchArgument("mode", default_value="kinematic",
                                  choices=["physics", "kinematic"]),
            DeclareLaunchArgument("gui", default_value="false",
                                  description="true=打开 Isaac Sim 窗口（可和 RViz 同时看）；false=headless"),
            DeclareLaunchArgument("nav2", default_value="none",
                                  description="M2 套件：none | dwa | mppi | teb | mpc（接入 Nav2 基线栈）"),
            DeclareLaunchArgument("map", default_value="",
                                  description="Nav2 地图 yaml（由 nav2_map_server 发布 /map）；空=采集器自己发"),
            DeclareLaunchArgument("controllers", default_value="",
                                  description="逗号分隔的控制器列表（如 dwa,mppi,teb）；空=只用 nav2 指定的那个。"
                                              "多个控制器注册在同一 controller_server 上，按 baseline 名选"),
            DeclareLaunchArgument("log_level", default_value="info",
                                  description="传给 Nav2 controller_server 的日志级别（排查用 debug）"),
            DeclareLaunchArgument("nav2_delay", default_value="30.0",
                                  description="Isaac 起来后再等多少秒启动 Nav2。实测 Isaac 冷启动 7~12 s，"
                                              "过早启动会让 controller_server 的 change_state 超时"
                                              "（日志：failed to send response to /controller_server/change_state）"),
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

            # 3) 演示指令源（等仿真起来再发；nav2 接管 /cmd_vel 时必须闭嘴）
            TimerAction(
                period=12.0,
                actions=[
                    ExecuteProcess(
                        cmd=["ros2", "run", "robot_sim", "e10_demo_driver.py",
                             "--ros-args", "-p", "use_sim_time:=true"],
                        output="screen",
                        condition=IfCondition(demo_cond),
                    )
                ],
            ),

            # 3b) M2 套件：Nav2 基线栈（等 /clock 与 TF 就绪后再起，避免生命周期激活超时）
            TimerAction(
                # TimerAction 只接受 str/float/Substitution，不接受 ParameterValue；
                # 用 PythonExpression 把 launch 参数字符串转成 float
                period=PythonExpression(["float('", nav2_delay, "')"]),
                actions=[
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(
                            [FindPackageShare("robot_bringup"), "/launch/nav2_baseline.launch.py"]),
                        launch_arguments={"baseline": nav2, "use_sim_time": use_sim_time,
                                          "log_level": log_level,
                                          "controllers": controllers,
                                          "map": map_yaml}.items(),
                        condition=IfCondition(nav2_on),
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

        ]
    )
