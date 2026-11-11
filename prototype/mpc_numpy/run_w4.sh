#!/usr/bin/env bash
# W4 实例入口：实例表 → 逐实例地图 → NavFn 参考 → MPC 推演 / 批量（conda pro + ROS Humble）
#
# 用法（在 prototype/mpc_numpy 下）：
#   ./run_w4.sh --instance static_avoid:1 --no-plot --quiet
#   ./run_w4.sh --suite --runs 1-5 --out out/tuning/round0 --no-plot --quiet
#   ./run_w4.sh --suite --runs all --out out --no-plot --quiet
#   ./run_w4.sh --suite --runs all --map-only --quiet      # 100 条路径可行性扫描
#
# 说明：本脚本只做 ROS 环境/静态 TF/清理；实例地图由 run_w3.py 在每条 run 前写入
# （与 run_w3.sh 的 S1~S3 预生成地图不同）。run_w2.sh / run_w3.sh 保持不动。
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

# 隔离本机其他 ROS 图；调试终端请 export 同值
export ROS_DOMAIN_ID=42
export PYTHONUNBUFFERED=1
echo "ROS_DOMAIN_ID=$ROS_DOMAIN_ID"

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

# 静态 TF map→base_link（实例起点均为原点；一次性规划只需起点位姿）
ros2 run tf2_ros static_transform_publisher \
  --x 0 --y 0 --z 0 --roll 0 --pitch 0 --yaw 0 \
  --frame-id map --child-frame-id base_link >/dev/null 2>&1 &
PIDS+=($!)

echo "== W4 实例推演（NavFn 参考）=="
"$PY" run_w3.py "$@"
