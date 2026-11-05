#!/usr/bin/env bash
# E09/E10 用的「干净 ROS 2 环境」：本机 conda base（Python 3.13）在 PATH 里，
# 会让 ros2 CLI / 节点 import rclpy 失败（缺 cpython-313 的 pybind 扩展）。
# 与 tools/engineering/build_workspace.sh 保持同一口径。
export PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin
unset PYTHONPATH AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH ROS_DISTRO 2>/dev/null || true
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
set +u
source /opt/ros/humble/setup.bash
WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
[ -f "$WS/install/setup.bash" ] && source "$WS/install/setup.bash"
set -u
