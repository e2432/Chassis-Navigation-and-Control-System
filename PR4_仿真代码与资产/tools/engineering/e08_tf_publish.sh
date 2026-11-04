#!/usr/bin/env bash
# E08：把 URDF 的传感器坐标系发到 /tf（robot_state_publisher + 静态 odom）。
#
# 目的：
#   1) 让 /lidar/points(frame_id=lidar_front_link)、/imu(frame_id=imu_link) 能在 RViz 里
#      经真实 URDF 帧链变换到 odom —— 顺带验证 E08 用到的帧名与 URDF 完全一致；
#   2) 消掉 RViz "Fixed Frame 不在 TF 树里" 的黄色告警。
# 注意：这里没有轮子关节状态，robot_state_publisher 会按 0 位姿发轮子帧；
#       E12 接上 joint_states 后才是正确姿态。地图/里程计在 E09-E12 补。
#
# 用法: ./tools/engineering/e08_tf_publish.sh &   然后 Ctrl-C 结束
set -uo pipefail
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
set +u
source /opt/ros/humble/setup.bash
set -u
WS="$(cd "$(dirname "$0")/../.." && pwd)"
URDF=/tmp/e08_robot.urdf
xacro "$WS/src/robot_description/urdf/agl_chs_03_f01.urdf.xacro" \
    mesh_dir:="file://$WS/src/robot_description/meshes" > "$URDF" || exit 1
echo "[e08-tf] URDF: $URDF ($(wc -l < "$URDF") 行)"
ros2 run robot_state_publisher robot_state_publisher \
    --ros-args -p robot_description:="$(cat "$URDF")" &
RSP=$!
ros2 run tf2_ros static_transform_publisher --x 0 --y 0 --z 0 \
    --frame-id odom --child-frame-id base_footprint &
ST=$!
trap 'kill -INT $RSP $ST 2>/dev/null' INT TERM
wait
