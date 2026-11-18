#!/usr/bin/env bash
# 逐个复测指定 (场景, 基线, runs序号) 组合，每个组合单独起一次栈。
# 用法: bash scripts/dev_targets.sh <输出子目录> "<场景>:<基线>:<run>" ...
#   bash scripts/dev_targets.sh trialA mixed_02:dwa:2 mixed_03:dwa:3 mixed_01:teb:1
set +e
cd "$(dirname "${BASH_SOURCE[0]}")/.."
ROOT="$(cd .. && pwd)"
source scripts/env.sh
export ISAAC_PYTHON=${ISAAC_PYTHON:-$HOME/miniconda3/envs/env_isaaclab/bin/python3}
[ -f "$HOME/teb_prebuilt/env.sh" ] && source "$HOME/teb_prebuilt/env.sh"
[ -f "$HOME/teb_build_ws/install/setup.bash" ] && source "$HOME/teb_build_ws/install/setup.bash"
TAG=$1; shift
SRC="$ROOT/out/mixed_on"
OUT="$ROOT/out/$TAG"
mkdir -p "$OUT/logs"
for spec in "$@"; do
  SCENE=${spec%%:*}; rest=${spec#*:}; BASE=${rest%%:*}; RUN=${rest##*:}
  echo "=== $SCENE $BASE run=$RUN  $(date +%H:%M:%S)"
  bash scripts/cleanup_sim.sh > /dev/null 2>&1
  ( ros2 launch robot_bringup simulation.launch.py \
      scene:=$SRC/scenes/$SCENE.usda map:=$SRC/maps/$SCENE.yaml \
      nav2:=$BASE controllers:=$BASE gui:=false rviz:=false \
      demo:=false duration:=0 nav2_delay:=30.0 > "$OUT/logs/stack_${SCENE}_${BASE}.log" 2>&1 & )
  # 等 Nav2 全栈真正激活（"Managed nodes are active"），再给代价地图几拍时间
  for i in $(seq 1 40); do
    sleep 5
    grep -q "Managed nodes are active" "$OUT/logs/stack_${SCENE}_${BASE}.log" 2>/dev/null && { sleep 10; break; }
  done
  for attempt in 1 2; do
    ( cd src/robot_benchmark && timeout 180 python3 -m robot_benchmark.runner --backend nav2 \
        --baseline "$BASE" --scenario mixed --runs "$RUN" --out "$OUT" \
        --scene $SRC/scenes/$SCENE.usda > "$OUT/logs/driver_${SCENE}_${BASE}.log" 2>&1 )
    if grep -q "成功\|run=" "$OUT/logs/driver_${SCENE}_${BASE}.log"; then break; fi
    echo "  第 $attempt 次失败，重试"
    sleep 20
  done
  grep -E "run=|失败|Traceback" "$OUT/logs/driver_${SCENE}_${BASE}.log" | tail -2
done
bash scripts/cleanup_sim.sh > /dev/null 2>&1
echo "全部完成"
