# ObstacleState / ObstacleStateArray（冻结 v1）

口径来源：`docs/接口契约.md` v1.4「障碍状态消息」、`docs/W5执行计划.md` 决策 4/5、ADR-0003。
本目录为两消息的唯一真源；**不加字段**，扩展时升版本。

## 坐标系与时间戳

- `position` / `velocity` 表达在 **odom 帧**。
- `ObstacleStateArray.header.frame_id = odom`。
- `header.stamp` 为最新估计时刻；单障碍 `stamp` 为该障碍的估计时刻。
- 超时判定统一使用 ROS 仿真时钟（`use_sim_time`）。
- `id` 跨帧稳定；`radius` 为单圆近似，单位 m；不加协方差字段。

## 空数组 / 过期 / 失联语义

| 条件 | 语义 |
|---|---|
| 时间戳有效的空数组 | 清空动态障碍（测试场景声明无障碍，不证明真实感知正常） |
| 单障碍 `stamp` 超过 `obstacle_timeout = 0.5 s` | 丢弃该障碍 |
| `header` 过期，或连续 0.5 s 无新消息 | 跟踪器失效 → 告警 + 零速，**不得视为无障碍** |

W5 用独立 mock 节点以 10 Hz 发布有效空数组（`frame_id=odom`、仿真时间戳、空列表）；
收到首条有效消息前保持零速。W6 换真跟踪器，正式导航禁止模拟心跳。
