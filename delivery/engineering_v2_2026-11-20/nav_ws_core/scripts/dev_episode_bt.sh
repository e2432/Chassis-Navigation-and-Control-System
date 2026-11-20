#!/usr/bin/env bash
# 起栈 + 跑一个 episode，在 goal 进行中用 gdb 断点判断控制器是否进入
# computeAndPublishVelocity / isGoalReached。
# 用法: bash scripts/dev_episode_bt.sh <场景> <基线> <runs序号>
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
    nav2:=$BASE controllers:=$BASE gui:=false rviz:=false \
    demo:=false duration:=0 nav2_delay:=30.0 > ../out/dev_stack.log 2>&1 & )
for i in $(seq 1 30); do
  sleep 5
  grep -q "Activating planner_server" ../out/dev_stack.log 2>/dev/null && { sleep 20; break; }
done
cd src/robot_benchmark
( timeout 150 python3 -m robot_benchmark.runner --backend nav2 --baseline "$BASE" \
    --scenario mixed --runs "$RUN" --out "$ROOT/out/dev_run" \
    --scene $SRC/scenes/$SCENE.usda > /tmp/dev_driver.log 2>&1 & )
cd ../..
sleep 25
PID=$(pgrep -f "nav2_controller/controller_server" | head -1)
echo "== pid $PID ==" 
echo 149248 | sudo -S timeout 45 sh -c "gdb -p $PID -batch \
  -ex 'set pagination off' \
  -ex 'break nav2_controller::ControllerServer::computeAndPublishVelocity()' \
  -ex 'break nav2_controller::ControllerServer::isGoalReached()' \
  -ex 'continue' -ex 'bt 6' -ex 'continue' -ex 'bt 6' 2>&1" \
  | grep -vE "^\[New LWP|^\[Thread" > ../out/dev_bt2.txt
grep -E "Breakpoint [0-9]+,|^#[0-7] " ../out/dev_bt2.txt | head -20
echo "== hits: $(grep -c 'Breakpoint' ../out/dev_bt2.txt) =="
