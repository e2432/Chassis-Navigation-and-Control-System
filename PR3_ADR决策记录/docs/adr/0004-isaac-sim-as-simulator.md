# ADR-0004：仿真器选用 Isaac Sim 5.1

- 状态：工程侧已采用，提交团队互审
- 日期：2026-11-04

## 决策与后果

以 Isaac Sim 5.1.0、PhysX、原生 ROS 2 桥接 4.12.4 冻结 M1 环境。已具备的机器人资产仅作为工程参考，未确认许可的资产不再分发。

代价：历史 GPU 8151 MiB 显存仅跑单实例；headless 带 DISPLAY 会卡 GLFW 初始化，因此脚本清除 DISPLAY；Mid-360 资产缺失，以 ADR-0005 的射线模型替代。

替代方案为重建 Gazebo 链路，会丢失已有 Isaac 验证与资产适配工作；现阶段不切换。依据：`docs/engineering/environment.md`、`results/w1/e01/`。
