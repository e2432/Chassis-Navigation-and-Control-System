"""单次基准入口（目标接口，未实现）。

目标命令（路线书 E27）：
    python3 -m robot_benchmark.run --scenario static_001 --controller dwb --seed 1

当前行为：解析参数、打印将要做什么，然后以退出码 2 报告“未实现”。
不伪造结果文件——宁可明确失败，也不要生成看起来成功的数据。
"""

import argparse
import sys

from .contract import BenchmarkNotImplementedError, run


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="python3 -m robot_benchmark.run",
        description="执行一个场景实例的一次基准任务（E15 实现）",
    )
    parser.add_argument("--scenario", required=True, help="场景实例 id，例如 static_001")
    parser.add_argument("--controller", default="dwb", choices=["dwb", "mppi", "teb", "mpc"])
    parser.add_argument("--seed", type=int, default=1, help="随机种子；最终以实例配置为准")
    parser.add_argument("--instance", default=None, help="实例文件路径（E14 生成）")
    parser.add_argument("--output-dir", default="results/runs", help="结果目录")
    return parser


def main(argv=None) -> int:
    args = build_parser().parse_args(argv)
    print("[robot_benchmark] 请求：scenario=%s controller=%s seed=%d instance=%s out=%s"
          % (args.scenario, args.controller, args.seed, args.instance, args.output_dir))
    try:
        run(None, controller=args.controller)  # type: ignore[arg-type]
    except BenchmarkNotImplementedError as exc:
        print("[robot_benchmark] 未实现：%s" % exc, file=sys.stderr)
        return 2
    return 0


if __name__ == "__main__":
    sys.exit(main())
