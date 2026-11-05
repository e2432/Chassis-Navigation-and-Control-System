#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""生成/校验五类场景的实例清单（E14 产出）。

    # 生成 5×20 个实例 + 冻结清单（覆盖 scenarios/instances/*.yaml 与 test_list.yaml）
    python3 -m robot_benchmark.gen_scenarios --out scenarios --runs 20

    # 只校验已有清单（CI 用）：数量、确定性、有效性规则
    python3 -m robot_benchmark.gen_scenarios --out scenarios --check

退出码：0 = 正常；2 = 校验不通过（便于脚本卡住）。
"""

from __future__ import annotations

import argparse
import os
import sys

import yaml

from robot_benchmark.scenario_gen import (SCENARIO_KEYS, generate_scenario,
                                          validate_instance)

GENERATOR_VERSION = "1.0.0"


def _load_templates(root: str) -> dict:
    tpls = {}
    for key in SCENARIO_KEYS:
        path = os.path.join(root, "%s.yaml" % key)
        with open(path, encoding="utf-8") as f:
            tpls[key] = yaml.safe_load(f)
    return tpls


def generate(root: str, runs: int) -> int:
    tpls = _load_templates(root)
    inst_dir = os.path.join(root, "instances")
    os.makedirs(inst_dir, exist_ok=True)
    summary = {}
    for key in SCENARIO_KEYS:
        insts = generate_scenario(key, tpls[key], runs)
        out = {
            "scenario": key,
            "generator_version": GENERATOR_VERSION,
            "asset_version": tpls[key]["asset_version"],
            "map_version": tpls[key]["map_version"],
            "runs": runs,
            "seed_rule": "seed = 场景序(1~5)×100 + run 序(1~20)",
            "instances": insts,
        }
        path = os.path.join(inst_dir, "%s_instances.yaml" % key)
        with open(path, "w", encoding="utf-8") as f:
            yaml.safe_dump(out, f, allow_unicode=True, sort_keys=False, width=100)
        n_dyn = sum(len(i["dynamic"]) for i in insts)
        n_unsolvable = sum(1 for i in insts if not i["solvable"])
        summary[key] = {"count": len(insts), "dynamic_agents": n_dyn,
                        "unsolvable": n_unsolvable,
                        "ids": [i["scenario_id"] for i in insts]}
        print("%-19s %d 个实例（动态障碍 %d 个，无解 %d 个）→ %s"
              % (key, len(insts), n_dyn, n_unsolvable, path))

    test_list = {
        "manifest_version": "1.0.0",
        "generator_version": GENERATOR_VERSION,
        "note": "所有控制器（自研 MPC + DWA/TEB/MPPI）复用同一批实例，逐 run 配对；"
                "实例取值已冻结在 instances/*.yaml 里，不依赖重新抽样。",
        "runs_per_scenario": runs,
        "scenarios": summary,
    }
    tl_path = os.path.join(root, "test_list.yaml")
    with open(tl_path, "w", encoding="utf-8") as f:
        yaml.safe_dump(test_list, f, allow_unicode=True, sort_keys=False, width=100)
    print("冻结清单 → %s（合计 %d 个实例）" % (tl_path, runs * len(SCENARIO_KEYS)))
    return 0


def check(root: str) -> int:
    tpls = _load_templates(root)
    bad = []
    total = 0
    for key in SCENARIO_KEYS:
        path = os.path.join(root, "instances", "%s_instances.yaml" % key)
        if not os.path.exists(path):
            bad.append("%s：缺实例文件 %s" % (key, path))
            continue
        with open(path, encoding="utf-8") as f:
            doc = yaml.safe_load(f)
        insts = doc["instances"]
        total += len(insts)
        if len(insts) != doc["runs"]:
            bad.append("%s：实例数 %d ≠ runs %d" % (key, len(insts), doc["runs"]))
        # 确定性：用同一模板重新生成一遍，必须与文件内容一致
        regen = generate_scenario(key, tpls[key], doc["runs"])
        if regen != insts:
            bad.append("%s：重新生成结果与文件不一致（确定性被破坏）" % key)
        for inst in insts:
            problems = validate_instance(
                {"start": (inst["robot_start"]["x"], inst["robot_start"]["y"]),
                 "goal": (inst["goal"]["x"], inst["goal"]["y"]),
                 "obstacles": inst["obstacles"], "dynamic": inst["dynamic"],
                 "solvable": inst["solvable"], "recovery_required": inst["recovery_required"],
                 "passage": inst.get("passage"), "dead_end": inst.get("dead_end")}, key)
            bad.extend("%s：%s" % (inst["scenario_id"], p) for p in problems)
        print("%-19s 校验 %d 个实例：%s" % (key, len(insts), "OK" if not bad else "有问题"))
    if bad:
        print("\n校验失败：")
        for b in bad:
            print("  -", b)
        return 2
    print("\n全部通过：%d 个实例，%d 个场景" % (total, len(SCENARIO_KEYS)))
    return 0


def main(argv=None) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default="scenarios",
                    help="scenarios 目录（默认工作空间相对路径）")
    ap.add_argument("--runs", type=int, default=20, help="每个场景的实例数（需求：20）")
    ap.add_argument("--check", action="store_true", help="只校验，不重新生成")
    a = ap.parse_args(argv)
    root = a.out
    if not os.path.isdir(root):
        print("找不到 scenarios 目录：%s" % root)
        return 2
    return check(root) if a.check else generate(root, a.runs)


if __name__ == "__main__":
    sys.exit(main())
