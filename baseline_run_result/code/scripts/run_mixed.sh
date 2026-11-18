#!/usr/bin/env bash
# 跑 mixed 场景的工作集（5 种子 × 3 基线），可指定代价地图口径。
#
# 用法：
#   bash scripts/run_mixed.sh <输出目录名> [NAV2_COSTMAP_YAML]
#     bash scripts/run_mixed.sh mixed_gate                     # 默认 corridor 口径
#     bash scripts/run_mixed.sh mixed_lidar nav2_common_lidar.yaml   # 雷达口径
set +e
cd "$(dirname "${BASH_SOURCE[0]}")/.."          # → nav_ws_core
ROOT="$(cd .. && pwd)"                          # → 仓库根
source scripts/env.sh
export ISAAC_PYTHON=${ISAAC_PYTHON:-$HOME/miniconda3/envs/env_isaaclab/bin/python3}
if [ -f "$HOME/teb_prebuilt/env.sh" ]; then source "$HOME/teb_prebuilt/env.sh"; fi
if [ -f "$HOME/teb_build_ws/install/setup.bash" ]; then source "$HOME/teb_build_ws/install/setup.bash"; fi
if [ -n "${2:-}" ]; then export NAV2_COSTMAP_YAML="$2"; fi

bash scripts/cleanup_sim.sh
echo "NAV2_COSTMAP_YAML=${NAV2_COSTMAP_YAML:-<default nav2_common.yaml>}"
python3 scripts/run_workset.py \
   --out "$ROOT/out/$1" \
   --scenarios mixed --runs 1 2 3 4 5 --baselines dwa mppi teb --retries 1
