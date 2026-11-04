#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""E14 场景实例生成器的单元测试。

覆盖：
  1. 确定性：同一 seed 生成两次结果一致；与仓库里冻结的实例文件一致；
  2. 数量：5 场景 × 20 实例 = 100，且 scenario_id/seed 规则正确；
  3. 有效性：每个实例通过 validate_instance（起点/目标不在障碍内、连通性可达、
     窄通道净宽 ≥1.3 m、dead_end 标注 recovery_required）；
  4. 拒绝能力：明显不合理的实例必须被判为不合格（防止"生成器永远说 OK"）。
"""

import os

import pytest
import yaml

from robot_benchmark.scenario_gen import (SCENARIO_KEYS, SEED_BASE, generate_scenario,
                                          validate_instance)

from pathlib import Path
ROOT = str(Path(__file__).resolve().parents[3] / "scenarios")


def _tpl(key):
    with open(os.path.join(ROOT, "%s.yaml" % key), encoding="utf-8") as f:
        return yaml.safe_load(f)


def _instances(key):
    with open(os.path.join(ROOT, "instances", "%s_instances.yaml" % key), encoding="utf-8") as f:
        return yaml.safe_load(f)


def test_five_keys():
    assert tuple(SCENARIO_KEYS) == ("static_avoid", "dynamic_pedestrian",
                                    "narrow_passage", "dead_end", "mixed")
    assert SEED_BASE == {"static_avoid": 100, "dynamic_pedestrian": 200,
                         "narrow_passage": 300, "dead_end": 400, "mixed": 500}


def test_deterministic_across_calls():
    for key in SCENARIO_KEYS:
        a = generate_scenario(key, _tpl(key), 20)
        b = generate_scenario(key, _tpl(key), 20)
        assert a == b, "%s 的生成结果不确定" % key


def test_matches_frozen_manifest():
    """仓库里冻结的实例必须与"现在重新生成"逐字段一致（防止悄悄改了生成器）。"""
    for key in SCENARIO_KEYS:
        doc = _instances(key)
        regen = generate_scenario(key, _tpl(key), doc["runs"])
        assert regen == doc["instances"], "%s 的冻结清单与重新生成结果不一致" % key


def test_counts_and_ids():
    total = 0
    for key in SCENARIO_KEYS:
        doc = _instances(key)
        insts = doc["instances"]
        assert len(insts) == 20
        total += len(insts)
        for i, inst in enumerate(insts, start=1):
            assert inst["scenario_id"] == "%s_%02d" % (key, i)
            assert inst["seed"] == SEED_BASE[key] + i
            assert inst["scenario"] == key
            assert inst["timeout_s"] == _tpl(key)["timeout_s"]
    assert total == 100


def test_all_instances_valid():
    for key in SCENARIO_KEYS:
        for inst in _instances(key)["instances"]:
            problems = validate_instance({
                "start": (inst["robot_start"]["x"], inst["robot_start"]["y"]),
                "goal": (inst["goal"]["x"], inst["goal"]["y"]),
                "obstacles": inst["obstacles"], "dynamic": inst["dynamic"],
                "solvable": inst["solvable"], "recovery_required": inst["recovery_required"],
                "passage": inst.get("passage"), "dead_end": inst.get("dead_end"),
            }, key)
            assert not problems, "%s: %s" % (inst["scenario_id"], problems)


def test_narrow_passage_clearance():
    for inst in _instances("narrow_passage")["instances"]:
        p = inst["passage"]
        assert p["net_width_m"] >= 1.30 - 1e-9
        assert p["margin_per_side_m"] >= 0.30 - 1e-9


def test_dead_end_flagged():
    for inst in _instances("dead_end")["instances"]:
        assert inst["recovery_required"] is True
        assert inst["dead_end"]["depth_m"] == pytest.approx(2.0)


def test_dynamic_pedestrians_reproducible():
    for key in ("dynamic_pedestrian", "mixed"):
        for inst in _instances(key)["instances"]:
            assert inst["dynamic"], "%s 应至少有一个动态障碍" % inst["scenario_id"]
            for d in inst["dynamic"]:
                assert d["type"] == "pedestrian"
                assert 0.6 - 1e-9 <= abs(d["velocity"]["vy"]) <= 1.0 + 1e-9
                assert d["start_time_s"] >= 0.0


def test_rejects_bad_instances():
    """生成器必须能识别不合格实例，否则"全部通过"没有意义。"""
    # 目标点落在障碍内部
    bad1 = {"start": (0.0, 0.0), "goal": (5.0, 0.0),
            "obstacles": [{"type": "circle", "x": 5.0, "y": 0.0, "r": 0.6}],
            "dynamic": [], "solvable": True}
    assert validate_instance(bad1, "static_avoid")
    # 障碍把整条走廊堵死（不可达）
    bad2 = {"start": (0.0, 0.0), "goal": (5.0, 0.0),
            "obstacles": [{"type": "box", "x": 2.5, "y": 0.0, "w": 0.4, "h": 12.0}],
            "dynamic": [], "solvable": True}
    assert any("可通行路径" in p for p in validate_instance(bad2, "static_avoid"))
    # 窄通道净宽不足
    bad3 = {"start": (0.0, 0.0), "goal": (10.0, 0.0), "obstacles": [], "dynamic": [],
            "solvable": True,
            "passage": {"net_width_m": 1.10, "length_m": 3.0,
                        "margin_per_side_m": 0.219, "min_required_margin_m": 0.30}}
    assert validate_instance(bad3, "narrow_passage")


# --------------------------------------------------------------------------- #
# 富化（rich）变体：给可视化/压力查看用
# --------------------------------------------------------------------------- #
def test_richify_deterministic_and_valid():
    from robot_benchmark.scenario_gen import core_of, richify
    for key in SCENARIO_KEYS:
        inst = _instances(key)["instances"][0]
        a = richify(key, inst)
        b = richify(key, inst)
        assert a == b, "%s 的富化结果不确定" % key
        assert not validate_instance(core_of(a), key), "%s 富化后场景不可达" % key
        assert a["rich"]["reachability_rechecked"] is True


def test_richify_adds_complexity():
    from robot_benchmark.scenario_gen import richify
    # 富化后障碍/行人数量都必须比原始实例多（否则"更复杂"没意义）
    for key in SCENARIO_KEYS:
        inst = _instances(key)["instances"][0]
        r = richify(key, inst)
        assert len(r["obstacles"]) > len(inst["obstacles"]) or len(r["dynamic"]) > len(inst["dynamic"]), key
    assert len(richify("static_avoid", _instances("static_avoid")["instances"][0])["obstacles"]) >= 12
    assert len(richify("mixed", _instances("mixed")["instances"][0])["dynamic"]) >= 3


def test_richify_all_instances_stay_valid():
    """20 个实例全部富化后仍必须可达（不能为了"复杂"造出无解场景）。"""
    from robot_benchmark.scenario_gen import core_of, richify
    for key in SCENARIO_KEYS:
        for inst in _instances(key)["instances"]:
            r = richify(key, inst)
            problems = validate_instance(core_of(r), key)
            assert not problems, "%s: %s" % (inst["scenario_id"], problems)
