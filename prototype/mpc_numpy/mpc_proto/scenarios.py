"""场景数据：W2 模拟场景（S1~S3）+ W4 基准实例加载器。

S1~S3 是模拟场景（docs/MPC公式化.md §9），只服务算法数学验证、不参与基准评分。
W4 起从 docs/scenarios/instances.yaml（契约 v1.4，generator_version 2.0.0）只读加载
5 个基准场景的逐种子实例；表内数值即最终事实，加载器不改表、不重新抽样。

参考几何（waypoints）默认取 start→goal 直线（名义行进线）；基准实例的静态绕行
几何由调用方用 NavFn 折线 `dataclasses.replace(sc, waypoints=...)` 替换（run_w3.py）。
"""
from __future__ import annotations

import math
import os
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np
import yaml

from .distance import Obstacle, Wall

SCENARIO_KEYS = ("static_avoid", "dynamic_pedestrian", "narrow_passage", "dead_end", "mixed")
INSTANCE_FIELDS = frozenset(
    (
        "scenario",
        "scenario_index",
        "run",
        "seed",
        "path_length_m",
        "time_limit_s",
        "goal",
        "obstacles",
        "pedestrians",
        "walls",
    )
)
GENERATOR_VERSION = "2.0.0"
INSTANCE_RUNS = tuple(range(1, 21))


@dataclass
class Scenario:
    name: str
    desc: str
    start: np.ndarray  # (3,)
    goal: np.ndarray  # (3,)
    waypoints: np.ndarray  # (M,2) 参考几何（含起点→目标）
    obstacles: list[Obstacle]
    t_max: float = 40.0  # 场景时限（s）
    walls: list[Wall] = field(default_factory=list)  # 轴对齐矩形墙（S1~S3 为空）


@dataclass
class InstanceMeta:
    """实例表元信息（run 记录指回实例表用）。"""

    scenario: str
    scenario_index: int
    run: int
    seed: int
    scenario_id: str
    time_limit_s: float
    path_length_m: float


# ---- W2 模拟场景（不参与基准评分） ----


def s1_static_avoid() -> Scenario:
    # 静障 (4.0, 0.0) r=0.6：参考几何从上方绕行（最近点 y=1.35 → 间隙 0.45）
    return Scenario(
        name="S1",
        desc="静态绕障",
        start=np.array([0.0, 0.0, 0.0]),
        goal=np.array([8.0, 0.0, 0.0]),
        waypoints=np.array([[0.0, 0.0], [2.0, 1.0], [4.0, 1.35], [6.5, 1.0], [8.0, 0.0]]),
        obstacles=[Obstacle(1, np.array([4.0, 0.0]), np.zeros(2), 0.6)],
        t_max=40.0,
    )


def s2_dynamic_crossing() -> Scenario:
    # 直参考；行人 (4.0, −4.9) 以 0.7 m/s 横穿，与机器人约 t≈7 s 在 x≈4 相遇
    return Scenario(
        name="S2",
        desc="动态横穿（0.7 m/s 匀速行人）",
        start=np.array([0.0, 0.0, 0.0]),
        goal=np.array([8.0, 0.0, 0.0]),
        waypoints=np.array([[0.0, 0.0], [8.0, 0.0]]),
        obstacles=[Obstacle(2, np.array([4.0, -4.9]), np.array([0.0, 0.7]), 0.3)],
        t_max=40.0,
    )


def s3_mixed_passage() -> Scenario:
    # 通道：静障 (5.0, 0.8) r=0.45 从上方压道，对向动障 0.7 m/s 沿通道来
    return Scenario(
        name="S3",
        desc="静+动混合（通道 1 静障 + 1 对向动障，双障碍）",
        start=np.array([0.0, 0.0, 0.0]),
        goal=np.array([10.0, 0.0, 0.0]),
        waypoints=np.array([[0.0, 0.0], [3.0, -0.2], [5.0, -0.35], [7.5, -0.2], [10.0, 0.0]]),
        obstacles=[
            Obstacle(3, np.array([5.0, 0.8]), np.zeros(2), 0.45),
            Obstacle(4, np.array([10.5, -0.5]), np.array([-0.7, 0.0]), 0.3),
        ],
        t_max=50.0,
    )


ALL = [s1_static_avoid, s2_dynamic_crossing, s3_mixed_passage]


def by_name(name: str) -> Scenario:
    for fn in ALL:
        sc = fn()
        if sc.name == name:
            return sc
    raise KeyError(f"未知场景: {name}（可选 S1/S2/S3）")


# ---- W4 基准实例加载器（docs/scenarios/instances.yaml，只读） ----


def default_instances_path() -> Path:
    """实例表路径：NAV_WS/docs/scenarios/instances.yaml → 仓库相对路径。"""
    candidates = []
    env = os.environ.get("NAV_WS")
    if env:
        candidates.append(Path(env) / "docs/scenarios/instances.yaml")
    candidates.append(Path(__file__).resolve().parents[3] / "docs/scenarios/instances.yaml")
    for path in candidates:
        if path.is_file():
            return path
    raise FileNotFoundError(
        "找不到实例表 docs/scenarios/instances.yaml（可用 NAV_WS 指定仓库根目录）："
        + ", ".join(str(p) for p in candidates)
    )


def _finite(value) -> bool:
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def _validate_instance(inst: dict, index: int) -> None:
    label = f"instances[{index}]"
    if not isinstance(inst, dict) or set(inst) != INSTANCE_FIELDS:
        raise ValueError(f"{label}: 字段必须恰为契约十字段 {sorted(INSTANCE_FIELDS)}")
    key = inst["scenario"]
    if key not in SCENARIO_KEYS:
        raise ValueError(f"{label}: 未知场景键 {key!r}（可选 {SCENARIO_KEYS}）")
    run = inst["run"]
    if type(run) is not int or run not in INSTANCE_RUNS:
        raise ValueError(f"{label}: run 必须为 1..20")
    scenario_index = SCENARIO_KEYS.index(key) + 1
    if inst["scenario_index"] != scenario_index or inst["seed"] != scenario_index * 100 + run:
        raise ValueError(f"{label}: scenario_index/seed 不符合固定编号规则（场景序×100+run）")

    goal = inst["goal"]
    if not isinstance(goal, dict) or set(goal) != {"x", "y", "yaw"} or not all(
        _finite(v) for v in goal.values()
    ):
        raise ValueError(f"{label}: goal 必须包含有限数值 x/y/yaw")
    if not _finite(inst["path_length_m"]) or inst["path_length_m"] <= 0.0:
        raise ValueError(f"{label}: path_length_m 必须为正数")
    if not math.isclose(inst["path_length_m"], math.hypot(goal["x"], goal["y"]), abs_tol=1e-6):
        raise ValueError(f"{label}: path_length_m 与起终点直线长度不一致")
    if not math.isclose(goal["yaw"], math.atan2(goal["y"], goal["x"]), abs_tol=1e-6):
        raise ValueError(f"{label}: goal.yaw 必须为起终点连线方向")
    if not _finite(inst["time_limit_s"]) or inst["time_limit_s"] <= 0.0:
        raise ValueError(f"{label}: time_limit_s 必须为正数")

    if not isinstance(inst["obstacles"], list):
        raise ValueError(f"{label}: obstacles 必须为列表")
    for o in inst["obstacles"]:
        if not isinstance(o, dict) or set(o) != {"x", "y", "r"} or not all(
            _finite(o[k]) for k in ("x", "y", "r")
        ) or o["r"] <= 0.0:
            raise ValueError(f"{label}: 圆障碍字段应为 x/y/r，半径为正")

    if not isinstance(inst["pedestrians"], list) or len(inst["pedestrians"]) > 2:
        raise ValueError(f"{label}: pedestrians 必须为列表且不超过 2 个")
    for p in inst["pedestrians"]:
        if (
            not isinstance(p, dict)
            or set(p) != {"x_cross", "speed", "t_meet", "dir", "r"}
            or not all(_finite(p[k]) for k in ("x_cross", "speed", "t_meet", "dir", "r"))
            or p["speed"] <= 0.0
            or p["r"] <= 0.0
            or type(p["dir"]) is not int
            or p["dir"] not in (-1, 1)
            or p["t_meet"] < 0.0
        ):
            raise ValueError(
                f"{label}: 行人应为 x_cross/speed/t_meet/dir/r，speed>0、dir=±1、t_meet≥0"
            )

    if not isinstance(inst["walls"], list):
        raise ValueError(f"{label}: walls 必须为列表")
    for w in inst["walls"]:
        if (
            not isinstance(w, dict)
            or set(w) != {"x0", "y0", "x1", "y1"}
            or not all(_finite(w[k]) for k in ("x0", "y0", "x1", "y1"))
            or w["x0"] >= w["x1"]
            or w["y0"] >= w["y1"]
        ):
            raise ValueError(f"{label}: 墙体角点须为有限值且 x0<x1、y0<y1")


def load_instance_table(path: str | Path | None = None) -> list[dict]:
    """读取并校验实例表，返回 instances 列表（只读，不修改表内数值）。"""
    table_path = Path(path) if path is not None else default_instances_path()
    doc = yaml.safe_load(table_path.read_text(encoding="utf-8"))
    if not isinstance(doc, dict):
        raise ValueError(f"{table_path}: 实例表顶层必须为映射")
    if int(doc.get("schema_version", -1)) != 1:
        raise ValueError(f"{table_path}: schema_version 须为 1")
    if str(doc.get("generator_version")) != GENERATOR_VERSION:
        raise ValueError(
            f"{table_path}: generator_version={doc.get('generator_version')!r}，"
            f"仅接受 {GENERATOR_VERSION}（2.0.0 重冻结表）"
        )
    if doc.get("coordinate_frame") != "path":
        raise ValueError(f"{table_path}: coordinate_frame 须为 path")

    instances = doc.get("instances")
    if not isinstance(instances, list) or not instances:
        raise ValueError(f"{table_path}: instances 必须为非空列表")
    for i, inst in enumerate(instances):
        _validate_instance(inst, i)
    return instances


def _build_instance(inst: dict) -> tuple[Scenario, InstanceMeta]:
    key = inst["scenario"]
    run = int(inst["run"])
    scenario_id = f"{key}_{run:02d}"
    goal = inst["goal"]
    goal_xy = np.array([float(goal["x"]), float(goal["y"])])

    obstacles: list[Obstacle] = []
    for o in inst["obstacles"]:
        obstacles.append(
            Obstacle(len(obstacles) + 1, np.array([float(o["x"]), float(o["y"])]), np.zeros(2), float(o["r"]))
        )
    # 行人 = 常速横穿：x(t)=x_cross、y(t)=dir·speed·(t−t_meet)；t=0 锚点 → p0_y = −dir·speed·t_meet
    for p in inst["pedestrians"]:
        v = np.array([0.0, float(p["dir"]) * float(p["speed"])])
        p0 = np.array([float(p["x_cross"]), -v[1] * float(p["t_meet"])])
        obstacles.append(Obstacle(len(obstacles) + 1, p0, v, float(p["r"])))

    walls = [Wall(float(w["x0"]), float(w["y0"]), float(w["x1"]), float(w["y1"])) for w in inst["walls"]]

    sc = Scenario(
        name=key,
        desc=f"实例 {scenario_id}（实例表 2.0.0）",
        start=np.zeros(3),
        goal=np.array([float(goal["x"]), float(goal["y"]), float(goal["yaw"])]),
        waypoints=np.vstack([np.zeros(2), goal_xy]),
        obstacles=obstacles,
        t_max=float(inst["time_limit_s"]),
        walls=walls,
    )
    meta = InstanceMeta(
        scenario=key,
        scenario_index=int(inst["scenario_index"]),
        run=run,
        seed=int(inst["seed"]),
        scenario_id=scenario_id,
        time_limit_s=float(inst["time_limit_s"]),
        path_length_m=float(inst["path_length_m"]),
    )
    return sc, meta


def load_instance(
    scenario: str, run: int, path: str | Path | None = None
) -> tuple[Scenario, InstanceMeta]:
    if scenario not in SCENARIO_KEYS:
        raise KeyError(f"未知基准场景键 {scenario!r}（可选 {SCENARIO_KEYS}）")
    if type(run) is not int or run not in INSTANCE_RUNS:
        raise ValueError("run 必须为 1..20")
    for inst in load_instance_table(path):
        if inst["scenario"] == scenario and inst["run"] == run:
            return _build_instance(inst)
    raise KeyError(f"实例表中没有 {scenario}:{run}")


def load_instances(
    scenario: str | None = None,
    runs: list[int] | tuple[int, ...] | None = None,
    path: str | Path | None = None,
) -> list[tuple[Scenario, InstanceMeta]]:
    """按场景/run 过滤加载；runs=None 表示全部 1..20。结果按场景键序、run 升序。"""
    if scenario is not None and scenario not in SCENARIO_KEYS:
        raise KeyError(f"未知基准场景键 {scenario!r}（可选 {SCENARIO_KEYS}）")
    run_set = set(INSTANCE_RUNS) if runs is None else set(runs)
    out = []
    for inst in load_instance_table(path):
        if scenario is not None and inst["scenario"] != scenario:
            continue
        if inst["run"] not in run_set:
            continue
        out.append(_build_instance(inst))
    out.sort(key=lambda sm: (SCENARIO_KEYS.index(sm[0].name), sm[1].run))
    return out


def parse_instance_arg(text: str) -> tuple[str, int]:
    """解析 `--instance <scenario>:<run>`。"""
    parts = text.split(":")
    if len(parts) != 2:
        raise ValueError(f"--instance 需 <scenario>:<run>，如 static_avoid:1（得到 {text!r}）")
    key, run_text = parts
    if key not in SCENARIO_KEYS:
        raise ValueError(f"--instance 场景键 {key!r} 未知（可选 {SCENARIO_KEYS}）")
    try:
        run = int(run_text)
    except ValueError as exc:
        raise ValueError(f"--instance run 必须为整数（得到 {run_text!r}）") from exc
    if run not in INSTANCE_RUNS:
        raise ValueError("--instance run 必须为 1..20")
    return key, run


def parse_runs_spec(spec: str) -> list[int]:
    """解析 run 列表：`all` / `1-5` / `1,3,5`（支持混写，如 `1-3,7`）。"""
    text = spec.strip().lower()
    if text in ("all", ""):
        return list(INSTANCE_RUNS)
    runs: set[int] = set()
    for part in text.split(","):
        part = part.strip()
        if not part:
            continue
        try:
            if "-" in part:
                lo, hi = part.split("-", 1)
                a, b = int(lo), int(hi)
                if a > b:
                    raise ValueError
                runs.update(range(a, b + 1))
            else:
                runs.add(int(part))
        except ValueError as exc:
            raise ValueError(f"--runs 无法解析 {part!r}（支持 all / 1-5 / 1,3,5）") from exc
    if not runs or any(r not in INSTANCE_RUNS for r in runs):
        raise ValueError("--runs 取值必须在 1..20")
    return sorted(runs)
