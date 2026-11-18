"""W5 D3 最小 smoke：controller_server + 生命周期 + 假 TF/odom/scan + mock 心跳 + 检查器。"""

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, LogInfo,
                            RegisterEventHandler)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import (LaunchConfiguration, PathJoinSubstitution,
                                  PythonExpression)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    chassis_type = LaunchConfiguration("chassis_type")
    base_yaw_deg = LaunchConfiguration("base_yaw_deg")
    map_x = LaunchConfiguration("map_x")
    map_y = LaunchConfiguration("map_y")
    map_yaw_deg = LaunchConfiguration("map_yaw_deg")
    publish_map_odom = LaunchConfiguration("publish_map_odom")
    map_odom_dynamic_seconds = LaunchConfiguration("map_odom_dynamic_seconds")
    tf_failure_expected = LaunchConfiguration("tf_failure_expected")
    use_sim_time = LaunchConfiguration("use_sim_time")
    obstacle_source = LaunchConfiguration("obstacle_source")
    scenario_name = LaunchConfiguration("scenario_name")
    scenario_run = LaunchConfiguration("scenario_run")
    publish_static_map = LaunchConfiguration("publish_static_map")
    use_mock = IfCondition(
        PythonExpression(["'", obstacle_source, "' == 'mock'"]))
    use_scenario = IfCondition(
        PythonExpression(["'", obstacle_source, "' == 'scenario'"]))
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
            "map_x": ParameterValue(map_x, value_type=float),
            "map_y": ParameterValue(map_y, value_type=float),
            "map_yaw_deg": ParameterValue(map_yaw_deg, value_type=float),
            "publish_map_odom": ParameterValue(publish_map_odom, value_type=bool),
            "map_odom_dynamic_seconds": ParameterValue(
                map_odom_dynamic_seconds, value_type=float),
        }],
    )

    heartbeat = Node(
        package="mpc_controller",
        executable="mock_obstacle_heartbeat.py",
        name="mock_obstacle_heartbeat",
        output="screen",
        condition=use_mock,
        parameters=[{
            "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
            "rate_hz": 10.0,
            "enabled": False,
        }],
    )

    # TEST ONLY: non-empty obstacles from the frozen scenario table.
    scenario_obstacles = Node(
        package="mpc_controller",
        executable="scenario_obstacle_publisher.py",
        name="scenario_obstacle_publisher",
        output="screen",
        condition=use_scenario,
        parameters=[{
            "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
            "scenario": ParameterValue(scenario_name, value_type=str),
            "run": ParameterValue(scenario_run, value_type=int),
            "rate_hz": 10.0,
            "frame_id": "odom",
        }],
    )

    # TEST ONLY: static map for the distance-field path.
    static_map = Node(
        package="mpc_controller",
        executable="fake_static_map.py",
        name="fake_static_map",
        output="screen",
        condition=IfCondition(publish_static_map),
        parameters=[{"use_sim_time": ParameterValue(use_sim_time, value_type=bool)}],
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
            "base_yaw_deg": ParameterValue(base_yaw_deg, value_type=float),
            "map_x": ParameterValue(map_x, value_type=float),
            "map_y": ParameterValue(map_y, value_type=float),
            "map_yaw_deg": ParameterValue(map_yaw_deg, value_type=float),
            "tf_failure_expected": ParameterValue(tf_failure_expected, value_type=bool),
            "map_odom_dynamic_seconds": ParameterValue(
                map_odom_dynamic_seconds, value_type=float),
            "obstacle_source": ParameterValue(obstacle_source, value_type=str),
            "expect_static_field": ParameterValue(publish_static_map, value_type=bool),
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
            "map_x", default_value="0.0", description="假 map->odom 平移 x（m）"),
        DeclareLaunchArgument(
            "map_y", default_value="0.0", description="假 map->odom 平移 y（m）"),
        DeclareLaunchArgument(
            "map_yaw_deg", default_value="0.0",
            description="假 map->odom 偏航（度）"),
        DeclareLaunchArgument(
            "publish_map_odom", default_value="true",
            description="false 用于 TF 缺失安全零速测试"),
        DeclareLaunchArgument(
            "map_odom_dynamic_seconds", default_value="0.0",
            description=">0 时 map->odom 改为动态 TF，到期停止以测试超时"),
        DeclareLaunchArgument(
            "tf_failure_expected", default_value="false",
            description="true 时检查器验证 TF 缺失/超时下的零速"),
        DeclareLaunchArgument(
            "use_sim_time", default_value="false",
            description="D3 smoke 用系统时钟；有 /clock 时才设 true"),
        DeclareLaunchArgument(
            "obstacle_source", default_value="mock",
            choices=["mock", "scenario"],
            description="mock=空数组心跳（默认）；scenario=TEST ONLY 非空场景障碍"),
        DeclareLaunchArgument(
            "scenario_name", default_value="dynamic_pedestrian",
            description="obstacle_source=scenario 时的场景键"),
        DeclareLaunchArgument(
            "scenario_run", default_value="1",
            description="obstacle_source=scenario 时的 run 序号"),
        DeclareLaunchArgument(
            "publish_static_map", default_value="false",
            description="TEST ONLY: 发布假 /map 以验证静态距离场"),
        LogInfo(msg=["[W5 D3] mpc_controller smoke: chassis_type=", chassis_type,
                     " base_yaw_deg=", base_yaw_deg, " map=(",
                     map_x, ",", map_y, ",", map_yaw_deg, ")",
                     " publish_map_odom=", publish_map_odom,
                     " map_odom_dynamic_seconds=", map_odom_dynamic_seconds,
                     " tf_failure_expected=", tf_failure_expected,
                     " obstacle_source=", obstacle_source]),
        fake_sensors,
        heartbeat,
        scenario_obstacles,
        static_map,
        controller_server,
        lifecycle_manager,
        smoke_check,
        shutdown_on_exit,
    ])
