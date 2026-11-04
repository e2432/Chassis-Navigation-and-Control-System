# 底盘导航控制系统

面向可移动底盘的自主导航系统：SLAM 建图 + 雷达避障 + 点到点巡航。算法先在 Isaac Sim
仿真中验证（RViz 可视化），再迁移到真实机器人。核心验收标准：**避障算法的优越性与
可迁移性**（定义见 [GLOSSARY.md](GLOSSARY.md)）。

技术路线：Nav2 框架 + 自研局部控制器插件（MPC），对比 DWA/TEB/MPPI 基线；
感知 Livox Mid-360（3D 固态雷达），SLAM 走 FAST-LIO2。

## 目录导读

| 位置 | 内容 |
|---|---|
| [方案书.md](方案书.md) | 项目总纲：已定决策、里程碑、每周任务、开放问题（随访谈实时更新） |
| [GLOSSARY.md](GLOSSARY.md) | 术语表：项目词汇的唯一口径 |
| [docs/](docs/) | 设计文档：架构、接口契约、指标定义、场景需求、MPC 公式化、教程笔记 |
| [docs/adr/](docs/adr/) | 架构决策记录（ADR-0001~0003） |
| [prototype/mpc_numpy/](prototype/mpc_numpy/) | numpy MPC 原型（W2 开环模拟 + W3 真 NavFn 参考），自带 [README](prototype/mpc_numpy/README.md) |
| `recording/` | 访谈存档（按编号归档，团队约定见 [提交指南.md](提交指南.md)） |
| [提交指南.md](提交指南.md) | 分支规则、提交规范、PR 流程（feature 分支 + PR 合入 main） |

## 关键文档

- 架构文档：[docs/架构文档.md](docs/架构文档.md)（系统拓扑、运行模式、包划分）
- 接口契约 v1：[docs/接口契约.md](docs/接口契约.md)（话题 / 消息 / 底盘描述 schema / 基准 API）
- 指标定义 v1.0：[docs/指标定义.md](docs/指标定义.md)（7 指标 + 公共判定）
- 5 场景需求清单：[docs/5场景需求清单.md](docs/5场景需求清单.md)（基准场景套件，交搭档实装）
- MPC 公式化：[docs/MPC公式化.md](docs/MPC公式化.md)（公式全量 + W2 参数定值 + W3 换源规格 §12）

## 快速开始（原型）

```bash
cd prototype/mpc_numpy

# W2 开环模拟（无 ROS 依赖，conda pro 环境）
./run_w2.sh

# W3 闭环第一刀（真 NavFn 参考；需先装 Nav2）
#   sudo apt-get update && sudo apt-get install -y ros-humble-navigation2
./run_w3.sh
```

详细启动指令、环境说明与逐文件讲解见 [prototype/mpc_numpy/README.md](prototype/mpc_numpy/README.md)。

## 提交约定

所有改动经 feature 分支 + Pull Request 合入 `main`（main 有分支保护，禁止直推），
流程与提交信息规范见 [提交指南.md](提交指南.md)。


## 工程侧入口

Isaac Sim + ROS 2 的 M1 启动、模型、TF、传感器、E14 五类场景与证据索引见 [工程侧说明](docs/engineering/README.md)。
冻结 100 个实例见 [scenarios/test_list.yaml](scenarios/test_list.yaml)。
