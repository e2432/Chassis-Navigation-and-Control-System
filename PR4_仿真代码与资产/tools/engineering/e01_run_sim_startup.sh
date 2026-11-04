#!/usr/bin/env bash
# 启动 E01 探针：headless 空场景 + 启用 isaacsim.ros2.bridge。
#
# 默认与 E02 一致：先 source 系统 ROS 2 Humble。这是必须的——不 source 时
# isaacsim.ros2.bridge 会因 AMENT_PREFIX_PATH 缺失而启动失败（见 docs/engineering/environment.md 第 6 节）。
#
# 用法:
#   e01_run_sim_startup.sh <输出目录> <python脚本> [参数...]        # 正常（source ROS 2）
#   E01_NO_ROS_SOURCE=1 e01_run_sim_startup.sh <输出目录> ...       # 故意不 source，复现失败
set -uo pipefail

OUT="${1:?用法: e01_run_sim_startup.sh <输出目录> <python脚本> [参数...]}"
shift
SCRIPT="${1:?缺少 python 脚本}"
shift || true
mkdir -p "$OUT"

SIM_PY="${ISAAC_PYTHON:?请设置 ISAAC_PYTHON 为 Isaac Sim Python 3.11}"

if [ "${E01_NO_ROS_SOURCE:-0}" = "1" ]; then
    echo "[e01] 警告：E01_NO_ROS_SOURCE=1，故意不 source ROS 2（用于复现 Bridge 启动失败）" >&2
    exec env -u ROS_DISTRO -u ROS_VERSION -u ROS_PYTHON_VERSION -u ROS_DOMAIN_ID \
        -u AMENT_PREFIX_PATH -u CMAKE_PREFIX_PATH -u COLCON_PREFIX_PATH -u PYTHONPATH \
        -u LD_LIBRARY_PATH -u RMW_IMPLEMENTATION \
        OMNI_KIT_ACCEPT_EULA=YES \
        "$SIM_PY" "$SCRIPT" "$@"
fi

export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
set +u
source /opt/ros/humble/setup.bash
# 工作空间已构建时一并 source：这样 URDF 里的 package:// 路径也能被导入器解析
if [ -f "$(cd "$(dirname "$0")/../.." && pwd)/install/setup.bash" ]; then
    source "$(cd "$(dirname "$0")/../.." && pwd)/install/setup.bash"
fi
set -u
echo "[e01] ROS_DISTRO=$ROS_DISTRO RMW_IMPLEMENTATION=$RMW_IMPLEMENTATION" >&2

exec "$SIM_PY" "$SCRIPT" "$@"
