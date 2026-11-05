# 工程证据索引

这些文件来自原 Linux/Isaac 工程环境，路径按当前布局脱敏；没有修改数值或将历史话题名改写为新测量。

| 验收项 | 证据 |
|---|---|
| E01 环境与启动 | `results/w1/e01/env_inventory.txt`、`gui_startup_evidence.txt` |
| E02 桥接与时钟 | `results/w1/e02/run_monotonic/summary.txt`、`run_reset/summary.txt`、`use_sim_time_probe.txt` |
| E05 模型 | `results/w2/e05/import_check.txt`、`scene_preview.png`、`model_top_view.png` |
| E06 运动对照 | `results/w2/e06/run_kinematic/summary.txt`、`run_physics/summary.txt` |
| E08 点云和 IMU | `results/w2/e08/bag_analysis.txt`、`sensor_summary.txt`、`cloud_topview.png`、`rviz_points.png` |
| E09 真值和 TF | `results/w3/e09/node_unit_test.txt`、`integration/verify_summary.txt`、`physics_mode/verify_summary.txt` |
| E10 M1 | `results/w3/e10/m1_check.txt`、`launch_command.txt`、`rviz_e10.png` |
| 原环境完整验收 | `results/w3/e10/acceptance_check_historical.txt`（PASS=23 FAIL=0；当时生成器测试为 9 项） |
| E14 可视化变体 | `results/w3/scenarios_usd_check.txt`（富化 USD；不同于 100 实例评分清单） |
| 本次打包校验 | `results/upload_validation.md`（Windows；不包含 Isaac/ROS 实机验收） |

bag/npz/高频大 CSV 未分发，旧报告中对其引用表示原始工程数据位置；需要重新取证时运行相应脚本。
本次 QoS 修改需在原环境复测；不使用旧 reliable 数据冒充 best_effort 的新证据。
