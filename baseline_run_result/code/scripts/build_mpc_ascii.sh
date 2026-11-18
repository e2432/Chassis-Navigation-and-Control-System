#!/usr/bin/env bash
# 在 ASCII 路径下构建 mpc_controller —— 规避中文路径导致的 rosidl 构建失败。
#
# 根因（2026-11-14 实测确认）：
#   本机工作区路径含中文（…/桌面/…），rosidl 生成接口时会把 `<repo>/` 前缀
#   整个截掉，报错：
#       list index: 1 out of range (-1, 0)          ← rosidl_generate_interfaces.cmake:240
#       Target dependency '/Chassis-…/build/mpc_controller/rosidl_adapter/mpc_controller/
#                          msg/ObstacleState.idl' does not exist
#   注意报错里的路径是 `/Chassis-…` 而不是 `<repo>/Chassis-…`，正好在中文目录处断开。
#
#   证据：
#     * 同一份源码在 /tmp/ascii_ws（ASCII 路径）`colcon build` → Finished <<< mpc_controller [9.03s]
#     * 用符号链接 /tmp/cnlink -> …/桌面/Chassis-… 从 ASCII 路径构建 → 仍然失败
#       （colcon/CMake 会 realpath 回中文路径，符号链接救不了）
#     * 算法侧在 /home/robot/…（ASCII）构建通过（out/w5_d3_smoke/mpc_d3_build.log）
#
# 用法：
#   scripts/build_mpc_ascii.sh            # 构建到 ~/mpc_build_ws
#   source ~/mpc_build_ws/install/setup.bash   # 之后 Nav2 才能 pluginlib 加载它
#
# 注意：这是**临时绕行**，不是最终方案。真正修法二选一：
#   1) 把整个工作区放到 ASCII 路径（推荐，CI/CI 机器也一致）；
#   2) 给 rosidl 打补丁修非 ASCII 路径（要动系统 ROS，不推荐）。
# 在绕行状态下，`ros2 launch robot_bringup nav2_baseline.launch.py baseline:=mpc` 需要先
# source 本脚本产出的 install/setup.bash。

set -euo pipefail

SRC_REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"   # nav_ws_core 的上级
PKG="mpc_controller"
DEST_WS="${MPC_ASCII_WS:-$HOME/mpc_build_ws}"

if [[ "$SRC_REPO" =~ [^[:ascii:]] ]]; then
    echo "[信息] 源码路径含非 ASCII 字符：$SRC_REPO"
    echo "       正是这个原因需要 ASCII 构建镜像，继续。"
fi
if [[ "$DEST_WS" =~ [^[:ascii:]] ]]; then
    echo "[错误] 目标构建目录也不能含非 ASCII 字符：$DEST_WS" >&2
    exit 1
fi

mkdir -p "$DEST_WS/src"
echo "[1/3] 同步源码 → $DEST_WS/src/$PKG"
rsync -a --delete \
      --exclude build --exclude install --exclude log \
      --exclude __pycache__ \
      "$SRC_REPO/nav_ws_core/src/$PKG/" "$DEST_WS/src/$PKG/"

echo "[2/3] 准备干净 ROS 环境"
export PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin
unset PYTHONPATH AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH ROS_DISTRO || true
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
# ROS 的 setup.bash 会引用可能未定义的变量，source 时必须先关掉 set -u
set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
set -u

echo "[3/3] colcon build --packages-select $PKG"
cd "$DEST_WS"
colcon build --packages-select "$PKG"

echo
echo "[完成] 产物：$DEST_WS/install/$PKG"
echo "       插件库：$DEST_WS/install/$PKG/lib/libmpc_controller.so"
echo "使用前先执行：source $DEST_WS/install/setup.bash"
echo "自检：        ros2 pkg prefix $PKG && ros2 interface list | grep ObstacleState"
