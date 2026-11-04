#!/usr/bin/env bash
# 启动 E02 仿真侧检查台：source 系统 ROS 2 Humble（Isaac Sim 5.1 的 bridge 用 system_default），
# 然后以 headless 方式跑 e02_sim_harness.py。
#
# 用法: e02_run_harness.sh <输出目录> [额外参数...]
set -uo pipefail

OUT="${1:?用法: e02_run_harness.sh <输出目录> [额外参数...]}"
shift || true
mkdir -p "$OUT"

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
SIM_PY="${ISAAC_PYTHON:?请设置 ISAAC_PYTHON 为 Isaac Sim Python 3.11}"

# 统一 DDS 域，与外部 ROS 2 终端一致；sim 与外部都用系统 ROS 2 Humble
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
set +u
source /opt/ros/humble/setup.bash
set -u
export ROS_DOMAIN_ID

echo "[e02_run_harness] ROS_DISTRO=$ROS_DISTRO ROS_DOMAIN_ID=$ROS_DOMAIN_ID RMW_IMPLEMENTATION=$RMW_IMPLEMENTATION" >&2

exec "$SIM_PY" "$SCRIPT_DIR/e02_sim_harness.py" --out "$OUT" "$@"
