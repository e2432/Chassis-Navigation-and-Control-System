#!/usr/bin/env bash
# Isaac GUI 里看动态避障（一个场景 + 一个基线）。
#
# 用法：
#   bash scripts/demo_gui.sh [场景] [基线] [run序号]
#     bash scripts/demo_gui.sh mixed_01 dwa      # 默认
#     bash scripts/demo_gui.sh mixed_01 mppi
#     bash scripts/demo_gui.sh mixed_01 teb
#   不想开 GUI（自测/录数据）：GUI=false bash scripts/demo_gui.sh mixed_01 dwa
#
# 流程：清残留 → 起栈（Isaac GUI + Nav2 单栈）→ 等全栈激活 → 跑一个 episode
#       （episode 会发 /sim/episode_start 启动行人相位，并发 /ped/corridor 给栏杆层）
set +e
cd "$(dirname "${BASH_SOURCE[0]}")/.."          # → nav_ws_core
ROOT="$(cd .. && pwd)"                          # → 仓库根
source scripts/env.sh
export ISAAC_PYTHON=${ISAAC_PYTHON:-$HOME/miniconda3/envs/env_isaaclab/bin/python3}
[ -f "$HOME/teb_prebuilt/env.sh" ] && source "$HOME/teb_prebuilt/env.sh"
[ -f "$HOME/teb_build_ws/install/setup.bash" ] && source "$HOME/teb_build_ws/install/setup.bash"

SCENE=${1:-mixed_01}
BASE=${2:-dwa}
RUN=${3:-1}
GUI=${GUI:-true}
SRC="$ROOT/out/mixed_v2"
[ -d "$SRC/scenes" ] || SRC="$ROOT/out/mixed_on"
LOG="$ROOT/out/demo_gui_${SCENE}_${BASE}.log"

echo "== 场景 $SCENE  基线 $BASE  GUI=$GUI  场景文件 $SRC/scenes/$SCENE.usda"
bash scripts/cleanup_sim.sh
( ros2 launch robot_bringup simulation.launch.py \
    scene:=$SRC/scenes/$SCENE.usda map:=$SRC/maps/$SCENE.yaml \
    nav2:=$BASE controllers:=$BASE gui:=$GUI rviz:=false \
    demo:=false duration:=0 nav2_delay:=30.0 > "$LOG" 2>&1 & )
echo "== 等 Nav2 全栈激活（日志 $LOG）"
for i in $(seq 1 40); do
  sleep 5
  grep -q "Managed nodes are active" "$LOG" 2>/dev/null && { echo "== 已激活（第 $((i*5)) 秒）"; break; }
done
sleep 8
echo "== 跑 episode（这一拍会复位仿真、启动行人、下发目标）"
cd src/robot_benchmark
timeout 240 python3 -m robot_benchmark.runner --backend nav2 --baseline "$BASE" \
  --scenario mixed --runs "$RUN" --out "$ROOT/out/demo_gui_run" \
  --scene "$SRC/scenes/$SCENE.usda"
echo "== 结束。GUI 里此时可以直接关窗口；再跑下一个基线请重新执行本脚本（会先清残留）"
