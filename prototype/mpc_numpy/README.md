# mpc_numpy —— W2 numpy 开环模拟原型

自包含滚动推演（不接 ROS/Isaac，闭环于完美运动学模型，障碍为场景脚本数据）。
数学依据：[docs/MPC公式化.md](../../docs/MPC公式化.md)；W5~6 移植 C++ Nav2 插件（`mpc_controller`，见 [docs/Nav2插件教程笔记.md](../../docs/Nav2插件教程笔记.md)）。

## 一条命令（W2 验收①）

```bash
./run_w2.sh          # conda pro 环境；S1/S2/S3 + 验收四条 + 诊断图 + EDT sanity
```

等价分步：

```bash
PY=/home/robot/miniconda3/envs/pro/bin/python
$PY run_sim.py --all                  # 三场景 + 验收表 + out/S*.png
$PY run_sim.py --scenario S2 --N 20   # 单场景 / 换时域
$PY sweep_horizon.py                  # N ∈ {20,40,60} 敏感性
$PY edt_sanity.py                     # 非验收：栅格 EDT 梯度排雷
$PY run_sim.py --all --chassis chassis/diff.yaml   # 迁移冒烟预演（vy_max=0）
```

## 验收四条（每场景）

| # | 判据 | 口径 |
|---|---|---|
| ① | 一条命令跑通 | `./run_w2.sh` 退出码 0 |
| ② | 逐步求解成功 | 每控制拍 OSQP `solved`，零速回退 = 0 |
| ③ | 推演全程间隙 > 0 | 外缘间隙（真值几何）全程 > 0 |
| ④ | Δu 总变差有界 | TV/拍 < 0.30（归一化控制量总变差） |

到达判定复用指标定义 0.25 m / 0.25 rad。

## 文件

| 文件 | 内容 |
|---|---|
| `run_sim.py` | 场景入口 + 验收表 |
| `run_w2.sh` | 一条命令 |
| `sweep_horizon.py` | N 敏感性扫描（定 N 依据） |
| `edt_sanity.py` | **非验收**：scipy EDT + 有限差分梯度 vs 解析梯度 |
| `mpc_proto/mpc.py` | 七项代价栈 QP 装配 + OSQP + 回退 |
| `mpc_proto/reference.py` | 折线参考几何 + 时间律推进 + 目标截断 |
| `mpc_proto/distance.py` | 解析距离场（静态 + 匀速预测动态） |
| `mpc_proto/scenarios.py` | S1 静绕 / S2 动横穿 / S3 静+动混合 |
| `mpc_proto/sim.py` | 滚动推演引擎 + 验收计算 |
| `mpc_proto/plotting.py` | 诊断图（轨迹俯视 / 间隙 / 控制量） |
| `chassis/*.yaml` | 底盘参数化描述（契约 schema v1；`diff` → `vy_max=0`） |

## 口径备忘

- 控制步长 dt = 0.05 s（契约 20 Hz）；仿真积分 dt_sim = 0.01 s（零阶保持）。
- 参考几何 = 场景 waypoints（NavFn 全局路径的 W2 替身）：静态绕行的几何由参考路径
  承担；MPC 负责跟踪 + 障碍软约束余量 + 动态避让（VO 管道 + 匀速直线预测）。
- 直参考穿障不采用：q_long ≤ 0.1·q_lat 下横向代价压过沿程收益（宁可迟到不可贴障），
  静态绕行几何归全局规划——与 ADR-0001 分工一致。
- OSQP 失败 → 零速 + 日志，连续 5 拍报警，不切第二控制器。
