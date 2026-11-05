# ADR-0006：真值里程计与 TF 唯一归属

- 状态：工程侧已采用，提交团队互审
- 日期：2026-11-04

## 决策与后果

专用静态节点唯一发布 map→odom；truth_odom_node 唯一发布 odom→base_footprint 并输出 /odom，twist 从世界系转换到机体系；robot_state_publisher 发布 URDF 固定链。仿真只发布 /sim/ground_truth，不发布 TF。

禁止同时启用旧真值链与新真值链，禁止 SLAM/AMCL 与真值节点重复占有 map→odom。节点启动检查发布者冲突，重置或跳变导致 twist 超限时置零。

依据：`docs/engineering/tf_tree.md`、`results/w3/e09/integration/verify_summary.txt`（60.00 Hz、最大间隔 16.7 ms、机体系误差 0.0000 m/s）。
