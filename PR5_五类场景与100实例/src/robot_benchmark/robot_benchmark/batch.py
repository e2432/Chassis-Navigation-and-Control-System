"""批量基准入口（目标接口，未实现）。

目标命令（路线书 E27）：
    python3 -m robot_benchmark.batch --manifest config/evaluation.yaml

当前行为：解析参数并明确报告未实现（退出码 2）。E24/E25 实现。
"""

import argparse
import sys

from .contract import BenchmarkNotImplementedError


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="python3 -m robot_benchmark.batch",
        description="按清单批量执行基准任务（E24/E25 实现）",
    )
    parser.add_argument("--manifest", required=True, help="批量清单 yaml，例如 config/evaluation.yaml")
    parser.add_argument("--dry-run", action="store_true", help="只检查清单与实例，不执行")
    return parser


def main(argv=None) -> int:
    args = build_parser().parse_args(argv)
    print("[robot_benchmark] 请求批量：manifest=%s dry_run=%s" % (args.manifest, args.dry_run))
    exc = BenchmarkNotImplementedError(
        "benchmark.batch 尚未实现：计划 E24/E25（W7）。清单格式见 config/evaluation.yaml。"
    )
    print("[robot_benchmark] 未实现：%s" % exc, file=sys.stderr)
    return 2


if __name__ == "__main__":
    sys.exit(main())
