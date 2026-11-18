#!/usr/bin/env bash
# W6 algorithm-side numeric checks. Run in the conda `pro` environment.
# The C++ dump executables must be built first:
#   cd nav_ws_core && source scripts/env.sh && colcon build --packages-select mpc_controller
set -euo pipefail

cd "$(dirname "${BASH_SOURCE[0]}")"
PY="${PY:-/home/robot/miniconda3/envs/pro/bin/python}"
ROOT="$(cd ../.. && pwd)"
BUILD="$ROOT/nav_ws_core/build/mpc_controller"

if [[ ! -x "$BUILD/mpc_qp_dump" || ! -x "$BUILD/mpc_sqp_dump" || ! -x "$BUILD/mpc_ref_dump" ]]; then
  echo "missing C++ dump executables in $BUILD; build mpc_controller first" >&2
  exit 2
fi

for script in \
  check_mpc_cpp_qp.py \
  check_mpc_cpp_sqp.py \
  check_mpc_cpp_ref.py \
  check_mpc_cpp_field.py \
  field_sim_demo.py \
  field_sim_scenarios.py \
  bench_mpc_py.py
do
  echo "=== $script ==="
  "$PY" "$script"
done

if [[ -x "$BUILD/distance_field_selftest" ]]; then
  echo "=== distance_field_selftest ==="
  "$BUILD/distance_field_selftest"
fi

echo "W6 numeric checks: PASS"
