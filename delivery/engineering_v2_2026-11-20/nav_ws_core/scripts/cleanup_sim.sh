#!/usr/bin/env bash
# 清掉所有仿真 / Nav2 残留进程。
# 用法：bash scripts/cleanup_sim.sh
# 注意：pkill 的模式放在变量里，避免匹配到脚本自身命令行。
set +e
for pat in \
  "isaac_sim_bridge.py" \
  "simulation.launch.py" \
  "controller_server" \
  "planner_server" \
  "bt_navigator" \
  "behavior_server" \
  "velocity_smoother" \
  "waypoint_follower" \
  "lifecycle_manager" \
  "robot_state_publisher" \
  "truth_odom_node" \
  "map_odom_static_node" \
  "flat_speed_limiter" \
  "safety_supervisor" \
  "rviz2" \
  "e10_demo_driver" \
  "robot_benchmark.runner" \
  "run_workset.py" ; do
  pkill -f "$pat" >/dev/null 2>&1
done
sleep 3
left=$(ps -eo cmd | grep -cE "isaac_sim_bridge|controller_server|planner_server")
echo "残留进程数: $left"
