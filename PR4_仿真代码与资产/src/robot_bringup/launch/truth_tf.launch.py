"""E09：唯一 TF 树 + 真值里程计（ROS 2 侧，不含 Isaac Sim 进程）。

拉起三件事，正好覆盖 `map → odom → base_footprint → base_link → 传感器`：

  1. `robot_state_publisher`：按 URDF 发 `base_footprint→base_link` 以及
     `base_link→{lidar_front_link, lidar_rear_link, mount_plate_link, wheel_*_link}`、
     `lidar_front_link→imu_link`（全部是固定关节，来自 robot_description）；
  2. `robot_sim/map_odom_static_node`：`map→odom` 固定变换（真值模式为 identity，发 /tf_static）；
  3. `robot_sim/truth_odom_node`：订阅仿真原始真值 `/sim/ground_truth`，
     发布 `/odom` + 唯一一条 `odom→base_footprint` TF（twist 从世界系转到机体系）。

用法：
    ros2 launch robot_bringup truth_tf.launch.py use_sim_time:=true
    # 切 SLAM/定位模式时（由算法侧发 map→odom 与 odom→base）：
    ros2 launch robot_bringup truth_tf.launch.py truth:=false

注意（E09 验收第 7 条，互斥）：
  * 真值模式下不要再起 `tools/engineering/e08_tf_publish.sh`（它会发静态 odom→base_footprint），
    也不要起既有包的 `truth_localization.py`（它发 map→odom）；truth_odom 启动时会检测并拒绝抢发。
  * Isaac Sim 侧只需跑 `src/robot_sim/robot_sim/scripts/isaac_sim_bridge.py --truth` 提供 `/sim/ground_truth` 与 `/clock`。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import Command, PathJoinSubstitution


def generate_launch_description() -> LaunchDescription:
    use_sim_time = LaunchConfiguration("use_sim_time")
    truth = LaunchConfiguration("truth")
    map_odom_yaw = LaunchConfiguration("map_odom_yaw_deg")
    odom_frame = LaunchConfiguration("odom_frame")
    base_frame = LaunchConfiguration("base_frame")

    desc_share = FindPackageShare("robot_description")
    xacro_file = PathJoinSubstitution([desc_share, "urdf", "agl_chs_03_f01.urdf.xacro"])
    mesh_dir = PathJoinSubstitution([desc_share, "meshes"])

    robot_description = ParameterValue(
        Command(["xacro ", xacro_file, " mesh_dir:=file://", mesh_dir]),
        value_type=str,
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true",
                                  description="参与仿真的节点必须为 true（/clock 为准）"),
            DeclareLaunchArgument("truth", default_value="true",
                                  description="true=真值模式（本 launch 发 map→odom 与 odom→base）；"
                                              "false=交给 SLAM/定位发（本 launch 只起 robot_state_publisher）"),
            DeclareLaunchArgument("map_odom_yaw_deg", default_value="0.0",
                                  description="map→odom 固定变换的偏航（度）"),
            DeclareLaunchArgument("odom_frame", default_value="odom"),
            DeclareLaunchArgument("base_frame", default_value="base_footprint"),
            LogInfo(msg=["[E09] truth_tf：robot_state_publisher + map→odom + 真值 odom→base；truth=",
                         truth, " use_sim_time=", use_sim_time]),

            # 1. URDF → TF（传感器外参的唯一来源，E09 检查项 5）
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                output="screen",
                parameters=[{"robot_description": robot_description,
                             "use_sim_time": use_sim_time,
                             "frame_prefix": ""}],
            ),

            # 2. map→odom（固定）
            Node(
                package="robot_sim",
                executable="map_odom_static_node.py",
                name="map_odom_static",
                output="screen",
                condition=IfCondition(truth),
                parameters=[{"use_sim_time": use_sim_time,
                             "odom_frame": odom_frame,
                             "yaw_deg": map_odom_yaw}],
            ),

            # 3. 真值适配：/odom + odom→base_footprint
            Node(
                package="robot_sim",
                executable="truth_odom_node.py",
                name="truth_odom",
                output="screen",
                condition=IfCondition(truth),
                parameters=[{"use_sim_time": use_sim_time,
                             "odom_frame": odom_frame,
                             "base_frame": base_frame,
                             "input_twist_frame": "world",
                             "publish_tf": True}],
            ),
        ]
    )
