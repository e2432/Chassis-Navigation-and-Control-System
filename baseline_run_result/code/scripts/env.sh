#!/usr/bin/env bash
# 用 source scripts/env.sh 加载系统 ROS；ISAAC_PYTHON 单独指定，不使用 conda Python 跑 ROS。
if [[ ! -f /opt/ros/humble/setup.bash ]]; then
    echo "需要安装 ROS 2 Humble: /opt/ros/humble/setup.bash" >&2
    return 1 2>/dev/null || exit 1
fi
export PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin
unset PYTHONPATH AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH ROS_DISTRO
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
export NAV_WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/humble/setup.bash
if [[ -f "$NAV_WS/install/setup.bash" ]]; then
    source "$NAV_WS/install/setup.bash"
fi
