#!/usr/bin/env bash
# E09 一键验证：TF 树节点 + 仿真真值桥接 + 外部订阅端核对。
#
#   1) ros2 launch robot_bringup truth_tf.launch.py（robot_state_publisher + map→odom + 真值适配）
#   2) isaac_sim_bridge.py --truth --sensors --demo（仿真侧：真值 + 点云 + IMU + 运动剧本）
#   3) tools/engineering/e09_verify.py（订阅端核对 /odom、TF、方向、twist 帧、冲突、重置）
set -uo pipefail
WS="$(cd "$(dirname "$0")/../.." && pwd)"
cd "$WS"
source tools/engineering/e09_sys_env.sh
OUT="${1:-results/w3/e09/integration}"
MODE_ARG_NOTE="（mode 见下面统计日志的 mode= 字段）"
mkdir -p "$OUT"
rm -f /dev/shm/fastrtps_* 2>/dev/null || true
DUR="${2:-24}"
MODE="${3:-kinematic}"

cleanup() {
    for p in $(ps -eo pid,args | awk '/robot_sim\/.*_node\.py/ && !/awk/ {print $1}'); do
        kill -9 "$p" 2>/dev/null
    done
    for p in $(ps -eo pid,args | awk '/robot_state_publisher/ && !/awk/ {print $1}'); do
        kill -9 "$p" 2>/dev/null
    done
    for p in $(pgrep -f "isaac_sim_bridge.py" 2>/dev/null); do kill -9 "$p" 2>/dev/null; done
}
trap cleanup EXIT INT TERM

# 1) TF 树
ros2 launch robot_bringup truth_tf.launch.py use_sim_time:=true > "$OUT/tf_launch.log" 2>&1 &
sleep 6

# 2) 仿真侧
SCRIPTS_PY="${ISAAC_PYTHON:?请设置 ISAAC_PYTHON 为 Isaac Sim Python 3.11}"
ROS_DOMAIN_ID="$ROS_DOMAIN_ID" "$SCRIPTS_PY" src/robot_sim/robot_sim/scripts/isaac_sim_bridge.py \
    --truth --sensors --demo --duration "$DUR" --out "$OUT" --mode "$MODE" \
    > "$OUT/sim_bridge.log" 2>&1 &
SIM=$!
sleep 10

# 3) 订阅端核对
python3 tools/engineering/e09_verify.py --duration "$((DUR - 8))" --out "$OUT"
RC=$?
sleep 2
wait $SIM 2>/dev/null
echo "--- 仿真侧统计 ---"
grep -aE "^\[SIM\]" "$OUT/sim_bridge.log" | tail -8
exit $RC
