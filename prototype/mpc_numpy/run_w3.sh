#!/usr/bin/env bash
# W3 验收①：一条命令拉起原型（真 NavFn 参考，S1/S2/S3 + 验收四条）
# 规划栈（map_server+planner_server）由 run_w3.py 逐场景拉起/回收（防串图）；
# 本脚本负责：环境检查、出图、静态 TF、执行、清理。
# 前置（一次性，需 sudo）：sudo apt-get update && sudo apt-get install -y ros-humble-navigation2
set -euo pipefail
cd "$(dirname "$0")"

PY=/home/robot/miniconda3/envs/pro/bin/python
ROS_SETUP=/opt/ros/humble/setup.bash

[[ -x "$PY" ]] || { echo "找不到 conda pro 的 Python：$PY" >&2; exit 1; }
[[ -f "$ROS_SETUP" ]] || { echo "找不到 ROS humble：$ROS_SETUP" >&2; exit 1; }
set +u   # setup.bash 引用未绑定变量，与 nounset 冲突
source "$ROS_SETUP"
set -u
ros2 pkg executables nav2_map_server >/dev/null 2>&1 || {
  echo "Nav2 未安装。请执行： sudo apt-get update && sudo apt-get install -y ros-humble-navigation2" >&2
  exit 1
}

# 隔离本机其他 ROS 图；调试终端（ros2 node list 等）请 export 同值
export ROS_DOMAIN_ID=42
echo "ROS_DOMAIN_ID=$ROS_DOMAIN_ID"

echo "== W3 地图栅格化（同源几何 0.05 m，动态障碍不进图）=="
"$PY" -m mpc_proto.mapping --out out/maps

PIDS=()
cleanup() {
  set +e
  for p in "${PIDS[@]:-}"; do kill "$p" 2>/dev/null; done
  # 兜底收残留节点（[x] 正则技巧防止匹配到执行本脚本的 shell 自身）
  pkill -f '[n]av2_map_server' 2>/dev/null
  pkill -f '[n]av2_planner' 2>/dev/null
  pkill -f '[s]tatic_transform_publisher' 2>/dev/null
  wait 2>/dev/null
}
trap cleanup EXIT INT TERM

# 静态 TF map→base_link（三场景起点均为原点；一次性规划只需起点位姿）
ros2 run tf2_ros static_transform_publisher \
  --x 0 --y 0 --z 0 --roll 0 --pitch 0 --yaw 0 \
  --frame-id map --child-frame-id base_link >/dev/null 2>&1 &
PIDS+=($!)

echo "== W3 原型推演（真 NavFn 参考）=="
"$PY" run_w3.py --all --quiet "$@"
