"""结果汇总入口（目标接口，未实现）。

目标命令（路线书 E27）：
    python3 -m robot_benchmark.summarize --input results/<batch_id>

要求（E25）：汇总表每行都能追溯到 run、配置、轨迹与原因；缺失项清楚可查。
当前行为：明确报告未实现（退出码 2）。
"""

import argparse
import sys

from .contract import BenchmarkNotImplementedError


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="python3 -m robot_benchmark.summarize",
        description="汇总批量结果，生成 summary.csv 与重跑清单（E25 实现）",
    )
    parser.add_argument("--input", required=True, help="批次目录，例如 results/<batch_id>")
    parser.add_argument("--output", default=None, help="输出前缀；默认写回批次目录")
    return parser


def main(argv=None) -> int:
    args = build_parser().parse_args(argv)
    print("[robot_benchmark] 请求汇总：input=%s output=%s" % (args.input, args.output))
    exc = BenchmarkNotImplementedError(
        "benchmark.summarize 尚未实现：计划 E25（W7）。结果目录结构见 docs/接口契约.md 第 8 节。"
    )
    print("[robot_benchmark] 未实现：%s" % exc, file=sys.stderr)
    return 2


if __name__ == "__main__":
    sys.exit(main())
