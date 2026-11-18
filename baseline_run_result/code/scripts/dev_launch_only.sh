#!/usr/bin/env bash
# 只起一次栈（不跑 runner），用于手工观测。用法: bash scripts/dev_launch_only.sh <场景> <基线> [日志]
set +e
cd "$(dirname "${BASH_SOURCE[0]}")/.."
ROOT="$(cd .. && pwd)"
source scripts/env.sh
export ISAAC_PYTHON=${ISAAC_PYTHON:-$HOME/miniconda3/envs/env_isaaclab/bin/python3}
[ -f "$HOME/teb_prebuilt/env.sh" ] && source "$HOME/teb_prebuilt/env.sh"
[ -f "$HOME/teb_build_ws/install/setup.bash" ] && source "$HOME/teb_build_ws/install/setup.bash"
SCENE=${1:-mixed_02}; BASE=${2:-dwa}; LOG=${3:-$ROOT/out/manual_launch.log}
SRC="$ROOT/out/mixed_on"
bash scripts/cleanup_sim.sh > /dev/null 2>&1
exec ros2 launch robot_bringup simulation.launch.py \
  scene:=$SRC/scenes/$SCENE.usda map:=$SRC/maps/$SCENE.yaml \
  nav2:=$BASE controllers:=$BASE gui:=false rviz:=false \
  demo:=false duration:=0 nav2_delay:=30.0 > "$LOG" 2>&1
