#!/usr/bin/env bash
# E09：节点级测试（不需要 Isaac Sim）——起 map→odom 静态节点 + 真值适配节点，跑合成真值测试。
set -uo pipefail
source "$(dirname "$0")/e09_sys_env.sh"
OUT="${1:-results/w3/e09}"
mkdir -p "$OUT"
ros2 run robot_sim map_odom_static_node.py > /tmp/e09_map_odom.log 2>&1 &
N1=$!
ros2 run robot_sim truth_odom_node.py > /tmp/e09_truth_odom.log 2>&1 &
N2=$!
sleep 6
python3 tools/engineering/e09_unit_test_nodes.py --duration 8 --out "$OUT"
RC=$?
# ros2 run 只是包装器，真正节点是子进程：按脚本名兜底清理，避免残留发布者(zombie)污染下次测试
kill -INT $N1 $N2 2>/dev/null; sleep 1
for p in $(ps -eo pid,args | awk '/lib\/robot_sim\/.*_node\.py/ && !/awk/ {print $1}'); do kill -9 "$p" 2>/dev/null; done
echo "--- 节点日志 ---"
grep -aE "已发布静态 TF|真值适配已启动|统计" /tmp/e09_map_odom.log /tmp/e09_truth_odom.log | head -4
exit $RC
