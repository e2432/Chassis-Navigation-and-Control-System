#!/usr/bin/env bash
# E01：核对真实环境（只读检查，不做任何安装）
# 用法: ./e01_env_inventory.sh [输出目录]
# 默认输出到 nav_ws/results/w1/e01
set -uo pipefail

OUT="${1:-$(cd "$(dirname "$0")/../.." && pwd)/results/w1/e01}"
mkdir -p "$OUT"
LOG="$OUT/env_inventory.txt"
: > "$LOG"

say() { printf '\n===== %s =====\n' "$1"; }
run() { printf '\n$ %s\n' "$*" >> "$LOG"; bash -c "$*" >> "$LOG" 2>&1; printf '[exit: %s]\n' "$?" >> "$LOG"; }

exec > >(tee -a "$LOG") 2>&1

echo "采集时间（墙上时间）: $(date -Is)"
echo "主机名: $(hostname)"
echo "执行用户: $(id -un)"
echo "采集脚本: $0"

say "1. 操作系统"
run 'lsb_release -a'
run 'cat /etc/os-release'
run 'uname -a'

say "2. CPU / 内存"
run 'grep -m1 "model name" /proc/cpuinfo'
run 'grep -c ^processor /proc/cpuinfo'
run 'free -h'

say "3. 磁盘剩余空间"
run 'df -h / /home /tmp'
run 'lsblk -o NAME,SIZE,FSTYPE,MOUNTPOINT | head -30'

say "4. GPU 硬件与驱动"
run 'lspci | grep -iE "vga|3d|display"'
run 'nvidia-smi'
run 'nvidia-smi --query-gpu=name,driver_version,memory.total,memory.used,utilization.gpu,temperature.gpu,compute_cap --format=csv'
run 'cat /proc/driver/nvidia/version'
run 'lsmod | grep -i nvidia'

say "5. ROS 2"
run 'ls -d /opt/ros/*'
run 'source /opt/ros/humble/setup.bash && printenv ROS_DISTRO ROS_VERSION ROS_PYTHON_VERSION'
run 'source /opt/ros/humble/setup.bash && which ros2 && ros2 --version 2>/dev/null || true'
run 'dpkg-query -W -f="\${Package} \${Version}\n" "ros-humble-desktop" "ros-humble-ros-base" "ros-humble-nav2-bringup" "ros-humble-rviz2" "ros-humble-tf2-tools" "ros-humble-rmw-fastrtps-cpp" 2>/dev/null'
run 'dpkg -l | grep -c "^ii  ros-humble"'
run 'ls /etc/apt/sources.list.d/ 2>/dev/null; grep -rh "packages.ros.org" /etc/apt/sources.list /etc/apt/sources.list.d/ 2>/dev/null | head -3'

say "6. DDS / RMW 环境"
run 'echo "RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-<未设置>}"; echo "ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-<未设置，默认 0>}"; echo "ROS_LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY:-<未设置>}"; echo "ROS_AUTOMATIC_DISCOVERY_RANGE=${ROS_AUTOMATIC_DISCOVERY_RANGE:-<未设置>}"; echo "LD_LIBRARY_PATH=$LD_LIBRARY_PATH"; echo "CYCLONEDDS_URI=${CYCLONEDDS_URI:-<未设置>}"; echo "FASTRTPS_DEFAULT_PROFILES_FILE=${FASTRTPS_DEFAULT_PROFILES_FILE:-<未设置>}"'
run 'dpkg-query -W -f="\${Package} \${Version}\n" "ros-humble-rmw-implementation" "ros-humble-rmw-fastrtps-cpp" "ros-humble-rmw-cyclonedds-cpp" "ros-humble-fastrtps" 2>/dev/null'
run 'source /opt/ros/humble/setup.bash && ros2 doctor --report 2>&1 | head -60'

say "7. Python 环境（外部 ROS 侧 vs 仿真侧）"
run 'which -a python3 python3.10 python3.11 2>/dev/null'
run '/usr/bin/python3 -V'
run 'source /opt/ros/humble/setup.bash && /usr/bin/python3 -c "import rclpy, sys; print(\"system rclpy OK:\", rclpy.__file__); print(sys.version)"'
run 'command -v python3; python3 -V'
run '"${ISAAC_PYTHON:?设置 ISAAC_PYTHON}" -V'

say "8. 构建工具"
run 'gcc --version | head -1'
run 'cmake --version | head -1'
run 'which colcon && colcon --help >/dev/null 2>&1 && echo "colcon OK"'
run 'which rosdep || echo "rosdep 未安装"'
run 'which git && git --version'

say "9. Isaac Sim 安装（pip 方式）"
run 'ls -d ${ISAAC_SIM_ROOT}* | head'
run 'cat ${ISAAC_SIM_ROOT}/VERSION 2>/dev/null'
run '"${ISAAC_PYTHON:?设置 ISAAC_PYTHON}" -m pip list | grep -i isaac'
run 'ls -la ${ISAAC_SIM_ROOT}/isaac-sim.sh'
run 'cat ${HOME}/.local/share/applications/IsaacSim.desktop'
run 'ls "${HOME}/.nvidia-omniverse/logs/Kit/Isaac-Sim Full/5.1" 2>/dev/null | tail -5'

say "10. ROS 2 Bridge 扩展（Isaac Sim 5.1 自带）"
run 'B=${ISAAC_SIM_ROOT}/exts/isaacsim.ros2.bridge; sed -n "1,20p" $B/config/extension.toml'
run 'B=${ISAAC_SIM_ROOT}/exts/isaacsim.ros2.bridge; grep -nE "ros_distro|publish_without_verification|enable_nitros" $B/config/extension.toml; ls $B; ls $B/humble/lib | grep -E "^librcl.so|^librmw|fastrtps" | head'

echo
echo "采集完成：$LOG"
