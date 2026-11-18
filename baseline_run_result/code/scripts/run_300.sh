#!/usr/bin/env bash
# 9-2 正式全量：3 基线 × 5 场景 × 20 种子 = 300 run（走廊口径，当前冻结配置）。
# 用法: bash scripts/run_300.sh [输出目录名，默认 9-2_official_v3]
set +e
cd "$(dirname "${BASH_SOURCE[0]}")/.."
ROOT="$(cd .. && pwd)"
source scripts/env.sh
export ISAAC_PYTHON=${ISAAC_PYTHON:-$HOME/miniconda3/envs/env_isaaclab/bin/python3}
[ -f "$HOME/teb_prebuilt/env.sh" ] && source "$HOME/teb_prebuilt/env.sh"
[ -f "$HOME/teb_build_ws/install/setup.bash" ] && source "$HOME/teb_build_ws/install/setup.bash"
OUT=${1:-9-2_official_v3}
mkdir -p "$ROOT/out/$OUT/logs"
bash scripts/cleanup_sim.sh > /dev/null 2>&1
echo "=== 第一遍：300 run"
python3 scripts/run_workset.py --out "$ROOT/out/$OUT" \
  --runs 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16 17 18 19 20 \
  --baselines dwa mppi teb --retries 2
echo "=== 补跑：缺失的 run.json（最多两遍）"
for pass in 1 2; do
  missing=$(python3 - "$ROOT/out/$OUT" <<'PY'
import sys, pathlib
root = pathlib.Path(sys.argv[1])
have = {(p.parent.parent.name, p.parent.name) for p in root.glob('*/runs/*/*/run.json')}
miss = []
for b in ('dwa', 'mppi', 'teb'):
    for sc in ('static_avoid', 'dynamic_pedestrian', 'mixed', 'narrow_passage', 'dead_end'):
        for r in range(1, 21):
            seed = {'static_avoid': 100, 'dynamic_pedestrian': 200, 'mixed': 500,
                    'narrow_passage': 300, 'dead_end': 400}[sc] + r
            if not (root / b / 'runs' / sc / str(seed) / 'run.json').exists():
                miss.append('%s:%s:%d' % (sc, b, r))
print(' '.join(miss))
PY
)
  if [ -z "$missing" ]; then echo "=== 无缺失，结束"; break; fi
  echo "=== 第 $pass 遍补跑：$missing"
  for spec in $missing; do
    IFS=: read -r sc b r <<< "$spec"
    python3 scripts/run_workset.py --out "$ROOT/out/$OUT" \
      --scenarios "$sc" --runs "$r" --baselines "$b" --retries 2
  done
done
bash scripts/cleanup_sim.sh > /dev/null 2>&1
echo "=== 汇总"
cd "$ROOT/nav_ws_core/src/robot_benchmark"
for b in dwa mppi teb; do
  python3 -m robot_benchmark.summarize --runs-root "../../../out/$OUT/$b" \
      --out "../../../out/$OUT/$b/summary.md" --json "../../../out/$OUT/$b/summary.json"
done
echo "=== 完成 $(date +%H:%M:%S)"
