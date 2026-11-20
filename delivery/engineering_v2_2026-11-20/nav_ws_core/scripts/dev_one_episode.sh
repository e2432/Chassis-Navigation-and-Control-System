#!/usr/bin/env bash
# 起一次栈 + 跑一个 episode，同时抓局部代价地图 ASCII。
# 用法: bash scripts/dev_one_episode.sh <场景名如 mixed_04> <baseline> <runs序号> [探针秒数]
set +e
cd "$(dirname "${BASH_SOURCE[0]}")/.."          # → nav_ws_core
ROOT="$(cd .. && pwd)"
source scripts/env.sh
export ISAAC_PYTHON=${ISAAC_PYTHON:-$HOME/miniconda3/envs/env_isaaclab/bin/python3}
if [ -f "$HOME/teb_prebuilt/env.sh" ]; then source "$HOME/teb_prebuilt/env.sh"; fi
if [ -f "$HOME/teb_build_ws/install/setup.bash" ]; then source "$HOME/teb_build_ws/install/setup.bash"; fi

SCENE=${1:-mixed_04}
BASE=${2:-dwa}
RUN=${3:-4}
PROBE=${4:-40}
SRC="$ROOT/out/mixed_on"
[ -d "$SRC" ] || SRC="$ROOT/out/mixed_gate"
[ -d "$SRC" ] || SRC="$ROOT/out/mixed_fix"

bash scripts/cleanup_sim.sh
( ros2 launch robot_bringup simulation.launch.py \
    scene:=$SRC/scenes/$SCENE.usda map:=$SRC/maps/$SCENE.yaml \
    nav2:=$BASE controllers:=$BASE gui:=false rviz:=false \
    demo:=false duration:=0 nav2_delay:=30.0 > ../out/dev_stack.log 2>&1 & )
for i in $(seq 1 30); do
  sleep 5
  grep -q "Activating planner_server" ../out/dev_stack.log 2>/dev/null && { sleep 20; break; }
done
cd src/robot_benchmark
( timeout 120 python3 -m robot_benchmark.runner --backend nav2 --baseline "$BASE" \
    --scenario mixed --runs "$RUN" --out "$ROOT/out/dev_run" \
    --scene $SRC/scenes/$SCENE.usda > /tmp/dev_driver.log 2>&1 & )
cd ../..
sleep 15
( timeout "$PROBE" python3 scripts/dev_probe_cmd.py "$PROBE" > ../out/dev_probe_cmd.log 2>&1 & )
timeout "$PROBE" python3 scripts/dev_probe_costmap.py "$PROBE" > ../out/dev_probe.log 2>&1
sleep 15
tail -2 /tmp/dev_driver.log
