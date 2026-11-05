#!/usr/bin/env bash
# 构建 nav_ws。额外参数会透传给 colcon（例如 --packages-select robot_benchmark）。
#
# 关键：必须用系统 python3 构建。
#   本机 conda base（${CONDA_ROOT}/bin/python3，Python 3.13）在 PATH 里，
#   ament_package_xml 会调用它，而它没有 catkin_pkg，导致
#   `ModuleNotFoundError: No module named 'catkin_pkg'`。
#   因此这里把 PATH 收敛到系统路径，构造环境与“纯 Ubuntu 机器”一致。
set -uo pipefail

REPO_ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
cd "$REPO_ROOT"

export PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin
unset PYTHONPATH AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH ROS_DISTRO 2>/dev/null || true

if [ ! -f /opt/ros/humble/setup.bash ]; then
    echo "[build] 找不到 /opt/ros/humble/setup.bash，先按 docs/engineering/dependencies.md 安装 ROS 2 Humble" >&2
    exit 1
fi

set +u
source /opt/ros/humble/setup.bash
set -u

echo "[build] python3=$(command -v python3) ($(python3 -V 2>&1))"
echo "[build] ROS_DISTRO=$ROS_DISTRO"

colcon build --symlink-install "$@"
status=$?

if [ $status -eq 0 ]; then
    echo
    echo "[build] 构建完成。加载环境："
    echo "  source $REPO_ROOT/install/setup.bash"
else
    echo "[build] 构建失败（exit $status）" >&2
fi
exit $status
