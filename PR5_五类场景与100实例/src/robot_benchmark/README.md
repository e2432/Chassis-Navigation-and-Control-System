# robot_benchmark

**状态：W1 交付“契约”，未交付“执行”。** 数据结构/状态枚举/指标键可用，运行入口一律报未实现。

## 当前可用的东西

| 模块 | 内容 |
|---|---|
| `robot_benchmark/contract.py` | `Scenario`（场景实例）、`RunResult`（运行结果）、`RunStatus`（5 类状态）、`METRIC_KEYS`（七项指标键）、`run()` 接口语义 |
| `robot_benchmark/metrics.py` | 七项指标的单位、好坏方向、空值策略、待确认问题 |
| `robot_benchmark/run.py` `batch.py` `summarize.py` | 目标命令行接口（参数已定，执行未实现，退出码 2） |

## 自检（不需要仿真）

```bash
source /opt/ros/humble/setup.bash && source install/setup.bash
python3 -c "import robot_benchmark as b; print(b.METRIC_KEYS); print(b.RunStatus.SUCCESS)"
python3 -m robot_benchmark.run --scenario static_001 --controller dwb --seed 1; echo "exit=$?"
```

## 未实现（按路线书排期）

| 功能 | 任务 | 计划 |
|---|---|---|
| 五类场景模板与实例生成 | E14 | W4 |
| `benchmark.run(scenario)` 全流程（重置→执行→采集→保存→停车） | E15 | W4 |
| 七项指标采集与汇总、原始记录绑定 | E16 | W4 |
| 批量清单与正式批次 | E24/E25 | W7 |

## 契约来源

`docs/接口契约.md` 第 8 节。改契约先改文档，再改代码。
