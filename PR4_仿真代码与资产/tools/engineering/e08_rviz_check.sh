#!/usr/bin/env bash
# E08：用 RViz2 检查 /lidar/points（bag 回放，不依赖 Isaac 在跑）。
#
# 步骤：回放 results/w2/e08/bag → 启动 rviz2（config/e08_rviz.rviz，Fixed Frame=lidar_front_link，
# 因为没有 /tf 也不需要有）→ 抓 rviz 窗口截图到 results/w2/e08/rviz_points.png。
#
# 用法: ./tools/engineering/e08_rviz_check.sh [bag路径] [输出png]
# 注意: 需要能显示窗口（DISPLAY），且 ROS_DOMAIN_ID 与录制时一致（默认 42）。
set -uo pipefail

BAG="${1:-results/w2/e08/bag}"
OUTPNG="${2:-results/w2/e08/rviz_points.png}"
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
set +u
source /opt/ros/humble/setup.bash
set -u

# 1) TF 树（URDF 帧链 + static odom），让 lidar_front_link/imu_link 能变换到 odom
./tools/engineering/e08_tf_publish.sh > /tmp/e08_tf.log 2>&1 &
TF=$!
sleep 3
ros2 bag play "$BAG" --loop --rate 1 > /tmp/e08_bagplay.log 2>&1 &
PLAY=$!
sleep 2
rviz2 -d config/e08_rviz.rviz > /tmp/e08_rviz.log 2>&1 &
RVIZ=$!
sleep 12
/usr/bin/python3 tools/engineering/e08_shoot_window.py "RViz" "$OUTPNG" || \
    /usr/bin/python3 tools/engineering/e08_shoot_window.py "rviz" "$OUTPNG" || echo "截图失败（见 /tmp/e08_rviz.log）"
kill -INT $RVIZ 2>/dev/null; kill -INT $PLAY 2>/dev/null; kill -INT $TF 2>/dev/null
sleep 2
kill -9 $RVIZ $PLAY $TF 2>/dev/null
# ros2 run / 脚本会派生孙进程，按 comm 名兜底清理（避免残留 publisher 一直发 /tf）
for p in $(ps -eo pid,comm | awk '$2 ~ /robot_state_pub|static_transf/ {print $1}'); do
    kill -9 "$p" 2>/dev/null
done
echo "rviz 日志尾部："; tail -5 /tmp/e08_rviz.log
