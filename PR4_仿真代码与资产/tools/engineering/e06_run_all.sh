#!/usr/bin/env bash
# E06 一键：仿真侧驱动 + 外部指令 + 分析
# 用法: e06_run_all.sh <输出目录> [mode: kinematic|physics] [仿真时长]
set -uo pipefail
OUT="${1:?用法: e06_run_all.sh <输出目录> [kinematic|physics] [时长]}"
MODE="${2:-kinematic}"
DUR="${3:-45}"
mkdir -p "$OUT"
S="$(cd "$(dirname "$0")" && pwd)"
OUT_ABS="$(cd "$OUT" && pwd)"

# 两侧必须在同一 DDS 域，否则 /clock、/cmd_vel 都看不到（踩过一次）
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
echo "[e06] 输出=$OUT_ABS mode=$MODE ROS_DOMAIN_ID=$ROS_DOMAIN_ID"

# 1) 仿真侧
"$S/e01_run_sim_startup.sh" "$OUT_ABS" "$S/e06_omni_drive.py" \
    --out "$OUT_ABS" --mode "$MODE" --duration "$DUR" --target-rtf 1.0 \
    > "$OUT_ABS/sim_stdout.log" 2>&1 &
SIM_PID=$!
echo "[e06] 仿真侧 PID=$SIM_PID，等待 /clock ..."

# 2) 等 /clock
set +u; source /opt/ros/humble/setup.bash; set -u
export ROS_DOMAIN_ID
DEADLINE=$((SECONDS + 240)); FOUND=0
while [ $SECONDS -lt $DEADLINE ]; do
    ros2 topic list > /tmp/e06_topics.txt 2>/dev/null
    if grep -qx "/clock" /tmp/e06_topics.txt; then FOUND=1; break; fi
    kill -0 $SIM_PID 2>/dev/null || { echo "[e06] 仿真侧已退出"; break; }
    sleep 2
done
echo "[e06] /clock 可见=$FOUND"

# 3) 外部指令
if [ "$FOUND" = "1" ]; then
    /usr/bin/python3 "$S/e06_motion_test.py" --out "$OUT_ABS" 2>&1 | tee "$OUT_ABS/ext_stdout.log"
fi

wait $SIM_PID 2>/dev/null || echo "[e06] 仿真侧退出码 $?"
echo "[e06] 仿真侧结束"

# 4) 分析
/usr/bin/python3 "$S/e06_analyze.py" --out "$OUT_ABS"
