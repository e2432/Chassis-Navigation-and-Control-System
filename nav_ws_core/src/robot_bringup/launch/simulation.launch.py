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

        ]
    )
