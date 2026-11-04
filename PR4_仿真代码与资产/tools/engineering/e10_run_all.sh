#!/usr/bin/env bash
# E10 一键验收：一条命令启动 → M1 检查 → RViz 截图。
#
# 记录：启动命令、启动日志（results/w3/e10/launch.log）、M1 检查结果、RViz 截图。
set -uo pipefail
WS="$(cd "$(dirname "$0")/../.." && pwd)"
cd "$WS"
source tools/engineering/e09_sys_env.sh
export NAV_WS="$WS"
export DISPLAY="${SHOT_DISPLAY:-:1}"    # 让 Isaac/RViz/截图都用同一个显示
OUT="${1:-results/w3/e10}"
DUR="${2:-22}"
MODE="${3:-kinematic}"
GUI="${4:-false}"        # true = 打开 Isaac Sim 窗口（与 RViz 同时看）
mkdir -p "$OUT"
# 上一轮被杀掉的进程常常留下 fastrtps 共享内存段，会让本次 DDS 发现变慢/收不到数据
rm -f /dev/shm/fastrtps_* 2>/dev/null || true

cleanup() {
    [ -n "${LAUNCH_PID:-}" ] && kill -INT "$LAUNCH_PID" 2>/dev/null
    sleep 3
    for pat in "isaac_sim_bridge.py" "e10_demo_driver.py" "robot_sim/.*_node.py" \
               "robot_state_publisher" "rviz2"; do
        for p in $(ps -eo pid,args | awk -v pat="$pat" -v self="$$" -v par="$PPID" \
                   '$1 != self && $1 != par && $0 ~ pat && !/awk/ {print $1}'); do
            kill -9 "$p" 2>/dev/null
        done
    done
}
trap cleanup EXIT INT TERM

echo "== E10 一条命令启动 ==" | tee "$OUT/launch_command.txt"
echo "ros2 launch robot_bringup simulation.launch.py use_sim_time:=true mode:=$MODE gui:=$GUI" \
    | tee -a "$OUT/launch_command.txt"

ros2 launch robot_bringup simulation.launch.py \
    use_sim_time:=true mode:="$MODE" gui:="$GUI" > "$OUT/launch.log" 2>&1 &
LAUNCH_PID=$!
echo "launch pid=$LAUNCH_PID，等待仿真起来（约 20 s：Isaac 启动 ~8 s + 图构建 + DDS 发现）"
sleep 20

python3 tools/engineering/e10_m1_check.py --duration "$DUR" --out "$OUT"
RC=$?

echo "--- 截图（Isaac 窗口 + RViz 窗口）---"
if [ "$GUI" = "true" ]; then
    python3 tools/engineering/tile_windows.py --display "$DISPLAY" >/dev/null 2>&1 || true
    sleep 2
    python3 tools/engineering/e08_shoot_window.py "Isaac Sim" "$OUT/isaac_e10.png" || echo "Isaac 截图失败"
fi
python3 tools/engineering/e08_shoot_window.py "RViz" "$OUT/rviz_e10.png" \
    || echo "截图失败（DISPLAY=$DISPLAY）"
sleep 1
echo "--- 启动日志摘要 ---"
grep -aE "\[E10\]|\[SIM\]|devices|Started|error" "$OUT/launch.log" | head -12
exit $RC
