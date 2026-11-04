# 本次工程交付验证（2026-11-04）

检查环境：Windows、Python 3.10.20。源工程 nav_ws 保持原状，修改仅在独立交付副本。

| 检查 | 结果 |
|---|---|
| 场景单元测试 | 原有测试 `12 passed in 78.86s`，包含全部冻结及富化实例有效性 |
| 冻结实例再生成比对与有效性 | 五类 × 20 = 100 个，全部通过 `--check` |
| Python 语法 | 39 个工程 Python 文件解析通过 |
| Shell 语法 | 所有工程 `.sh` 文件通过 Git Bash `bash -n` |
| 包清单及配置 | 四个 package.xml 及全部工程 YAML 可解析 |
| 自建资产 | 12 份分发资产 SHA-256 校验通过；11 份 USD 层外部文件引用均存在 |
| USD 路径 | 五个场景内嵌绝对引用改相对；生成器保存时也使用相对引用 |
| 排除项 | 未含 bag/db3/npz/大 CSV/构建缓存/第三方机器人资产，未检出原机器 home 路径或 token/私钥格式 |
| 文档 | 工程文档、W1–W3 合并、ADR 0004–0007、契约 v1.1 提案及 Q1–Q7 |

## 需要在 Linux + ROS 2 + Isaac Sim 上复跑

本机未安装 ROS 2 Humble / Isaac Sim，**未运行** colcon 构建、完整 acceptance_check、ROS launch、
GPU 运动与传感器复测。原环境 `PASS=23 FAIL=0` 仅保存在
`results/w3/e10/acceptance_check_historical.txt`；旧记录当时生成器为 9 项测试，当前版本为 12 项，
不能混作同一次验收。

点云/IMU 的 best_effort 发布及配套订阅/RViz 已显式配置，依据
[NVIDIA 5.1 QoS 文档](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/ros2_tutorials/tutorial_ros2_qos.html)。
运行验证待补做，历史 10.02 / 60.11 Hz 属于原 reliable 发布配置。

USD 外部文件引用检查通过，但原导入模型的 `/visuals/base_footprint`、`/visuals/mount_plate_link`、
`/visuals/imu_link` 存在未解析 prim 警告（空视觉节点）；本次未重建底盘，需 E11/E12 一并检查。
11 层均可打开，不将外部文件引用完整描述为全部视觉节点无警告。

## 对上传清单的必要调整

- 清单标题称 26 个脚本，逐名统计实际 29 个；另补四个启动/GUI 辅助依赖，共 33 个脚本。
- 补齐 bringup 的 CMakeLists/config、benchmark 的 resource/setup.cfg/__init__ 等构建必需文件。
- 自建底盘入口 USD 保留在 assets 根目录，补四个 configuration 层；场景相对引用依赖此布局。
- 只交付清单要求的四个包；八包是原私仓背景，旧 robot_a_sim_control 和无关骨架未扩入本次上传。
- 根目录 scenarios 是唯一场景数据源，测试、生成器、setuptools、验收、可视化入口同步调整。
- 原始证据实际数量以本次文件清单为准，仅选 txt/md/png 和小于 50,000 字节 CSV；另转存一份历史验收 txt。
- 根 README 原为 UTF-16，转为 UTF-8 后增加工程入口，以便 GitHub 正常显示和维护。
- 契约 v1.1 保留待互审状态，不代填 Q1–Q7，不声称 E15/E16、Nav2 或安全层已经实现。
