"""W5 D3 最小 smoke：controller_server + 生命周期 + 假 TF/odom/scan + mock 心跳 + 检查器。"""

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, LogInfo,
                            RegisterEventHandler)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    chassis_type = LaunchConfiguration("chassis_type")
    base_yaw_deg = LaunchConfiguration("base_yaw_deg")
    use_sim_time = LaunchConfiguration("use_sim_time")
    params_file = PathJoinSubstitution(
        [FindPackageShare("mpc_controller"), "config", "nav2_mpc_params.yaml"])

    fake_sensors = Node(
        package="mpc_controller",
        executable="fake_sensors.py",
        name="fake_sensors",
        output="screen",
        parameters=[{
            "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
            "base_yaw_deg": ParameterValue(base_yaw_deg, value_type=float),
        }],
    )

    heartbeat = Node(
        package="mpc_controller",
        executable="mock_obstacle_heartbeat.py",
        name="mock_obstacle_heartbeat",
        output="screen",
        parameters=[{
            "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
            "rate_hz": 10.0,
            "enabled": False,
        }],
    )

    controller_server = Node(
        package="nav2_controller",
        executable="controller_server",
        name="controller_server",
        output="screen",
        parameters=[
            params_file,
            {
                "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                "FollowPath.type": ParameterValue(chassis_type, value_type=str),
            },
        ],
        remappings=[("cmd_vel", "/cmd_vel")],
    )

    lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_mpc",
        output="screen",
        parameters=[{
            "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
            "autostart": True,
            "node_names": ["controller_server"],
            "bond_timeout": 0.0,
        }],
    )

    smoke_check = Node(
        package="mpc_controller",
        executable="smoke_check.py",
        name="smoke_check",
        output="screen",
        parameters=[{
            "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
            "chassis_type": ParameterValue(chassis_type, value_type=str),
        }],
    )

    shutdown_on_exit = RegisterEventHandler(
        OnProcessExit(
            target_action=smoke_check,
            on_exit=[EmitEvent(event=Shutdown(reason="smoke check finished"))],
        )
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "chassis_type", default_value="omni", choices=["omni", "mecanum", "diff"]),
        DeclareLaunchArgument(
            "base_yaw_deg", default_value="0.0",
            description="假 TF 中机器人相对 odom 的初始偏航（度）"),
        DeclareLaunchArgument(
            "use_sim_time", default_value="false",
            description="D3 smoke 用系统时钟；有 /clock 时才设 true"),
        LogInfo(msg=["[W5 D3] mpc_controller smoke: chassis_type=", chassis_type,
                     " base_yaw_deg=", base_yaw_deg]),
        fake_sensors,
        heartbeat,
        controller_server,
        lifecycle_manager,
        smoke_check,
        shutdown_on_exit,
    ])
