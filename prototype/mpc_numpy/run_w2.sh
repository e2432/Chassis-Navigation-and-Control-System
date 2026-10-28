#!/usr/bin/env bash
# W2 验收①：一条命令跑通 numpy 开环模拟（S1/S2/S3 + 验收四条 + 诊断图）
# 用法：./run_w2.sh
set -euo pipefail
cd "$(dirname "$0")"

PY=/home/robot/miniconda3/envs/pro/bin/python
if [[ ! -x "$PY" ]]; then
  echo "找不到 conda pro 的 Python：$PY" >&2
  exit 1
fi

echo "== W2 numpy MPC 开环模拟（conda pro）=="
"$PY" run_sim.py --all --quiet
echo
echo "== EDT sanity（非验收，W5~6 排雷）=="
"$PY" edt_sanity.py
echo
echo "全部完成。诊断图见 out/S{1,2,3}.png"
