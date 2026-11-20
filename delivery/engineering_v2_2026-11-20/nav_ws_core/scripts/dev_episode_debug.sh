#!/usr/bin/env bash
# 起栈（controller_server=debug）+ 跑一个 episode，在 goal 进行中抓 gdb 栈。
# 用法: bash scripts/dev_episode_debug.sh <场景> <基线> <runs序号>
set +e
cd "$(dirname "${BASH_SOURCE[0]}")/.."
ROOT="$(cd .. && pwd)"
source scripts/env.sh
export ISAAC_PYTHON=${ISAAC_PYTHON:-$HOME/miniconda3/envs/env_isaaclab/bin/python3}
[ -f "$HOME/teb_prebuilt/env.sh" ] && source "$HOME/teb_prebuilt/env.sh"
[ -f "$HOME/teb_build_ws/install/setup.bash" ] && source "$HOME/teb_build_ws/install/setup.bash"
SCENE=${1:-mixed_04}; BASE=${2:-dwa}; RUN=${3:-4}
SRC="$ROOT/out/mixed_on"
bash scripts/cleanup_sim.sh
( ros2 launch robot_bringup simulation.launch.py \
    scene:=$SRC/scenes/$SCENE.usda map:=$SRC/maps/$SCENE.yaml \
    nav2:=$BASE controllers:=$BASE gui:=false rviz:=false log_level:=debug \
    demo:=false duration:=0 nav2_delay:=30.0 > ../out/dev_stack_dbg.log 2>&1 & )
for i in $(seq 1 30); do
  sleep 5
  grep -q "Activating planner_server" ../out/dev_stack_dbg.log 2>/dev/null && { sleep 20; break; }
done
cd src/robot_benchmark
( timeout 150 python3 -m robot_benchmark.runner --backend nav2 --baseline "$BASE" \
    --scenario mixed --runs "$RUN" --out "$ROOT/out/dev_run" \
    --scene $SRC/scenes/$SCENE.usda > /tmp/dev_driver.log 2>&1 & )
cd ../..
sleep 25
PID=$(pgrep -f "nav2_controller/controller_server" | head -1)
echo "== controller_server pid $PID ==" 
timeout 60 python3 scripts/dev_probe_cmd.py 8 > ../out/dev_probe_cmd.log 2>&1 &
sleep 4
echo 149248 | sudo -S sh -c "gdb -p $PID -batch -ex 'thread apply all bt 14' 2>/dev/null" > ../out/dev_bt.txt 2>&1
grep -E "^Thread|compute|dwa|costmap_2d>>|nav2_core" ../out/dev_bt.txt | head -40
sleep 20
grep -cE "cmd_vel" /dev/null
tail -5 ../out/dev_probe_cmd.log
