# ADR-0005：以射线投射替代雷达模型

- 状态：工程侧已采用，提交团队互审
- 日期：2026-11-04

## 决策与后果

工程环境官方 18 个 RTX 配置没有 Mid-360，传感器资产服务不可用；采用 PhysX raycast 360×8、水平 360°、垂直 −7°~52°、40 m、10 Hz。PointCloud2 只有 x/y/z，point_step=12。

这不等价于 Mid-360：无花瓣扫描、无 intensity/ring/time、无测距噪声和运动畸变，目前只建前雷达。最下沿落地约 2.63 m，近场低矮障碍覆盖不足。IMU 标称 200 Hz，physics 链路历史发布 60.11 Hz。

依据：`docs/engineering/sensors.md`、`results/w2/e08/bag_analysis.txt`。旧 reliable 测量不是本次 best_effort 复测。
