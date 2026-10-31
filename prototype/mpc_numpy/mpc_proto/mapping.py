"""W3 地图栅格化：场景几何 → PGM+YAML（map_server 消费，docs/MPC公式化.md §12）。

与解析距离场（distance.py）同一份 Scenario.obstacles 数据，防两套几何漂移。
只画静态障碍（v=0 的 Obstacle）；动态障碍永不进图、不进 bounds
（动态避让归 MPC 的 VO，NavFn 看不见行人）。
"""
from __future__ import annotations

import argparse
import re
from pathlib import Path

import numpy as np

from .scenarios import ALL, Scenario

RESOLUTION = 0.05  # m/cell（§12 定值，与 costmap resolution 一致）
BOUND_MARGIN = 1.5  # m（≫ inflation 0.35，给 NavFn 绕行留空间）
FREE, OCCUPIED = 254, 0  # PGM 像素值（map_server trinary 约定）


def scenario_bounds(sc: Scenario, margin: float = BOUND_MARGIN) -> tuple[float, float, float, float]:
    """(xmin, ymin, xmax, ymax)：起点/目标/静态圆盘 ±radius 的 bbox 外扩 margin。"""
    pts = [np.asarray(sc.start[:2], dtype=float), np.asarray(sc.goal[:2], dtype=float)]
    for o in sc.obstacles:
        if not o.dynamic:
            pts.append(np.asarray(o.p0) - o.radius)
            pts.append(np.asarray(o.p0) + o.radius)
    arr = np.vstack(pts)
    xmin, ymin = arr.min(axis=0) - margin
    xmax, ymax = arr.max(axis=0) + margin
    return float(xmin), float(ymin), float(xmax), float(ymax)


def build_grid(sc: Scenario, resolution: float = RESOLUTION) -> tuple[np.ndarray, tuple[float, float]]:
    """返回 (grid (H,W) uint8, (xmin, ymin))。

    grid 已是 PGM 行序（row 0 = y_max，标准 ROS 地图约定）：
    cell 中心 = (xmin+(i+0.5)res, ymin+(h-1-row+0.5)res)。
    """
    xmin, ymin, xmax, ymax = scenario_bounds(sc)
    w = max(2, int(np.ceil((xmax - xmin) / resolution)))
    h = max(2, int(np.ceil((ymax - ymin) / resolution)))
    xs = xmin + (np.arange(w) + 0.5) * resolution
    ys = ymin + (np.arange(h) + 0.5) * resolution
    X, Y = np.meshgrid(xs, ys)  # (h, w)，行 j ↔ ys[j]（自下而上）
    occ = np.zeros((h, w), dtype=bool)
    for o in sc.obstacles:
        if not o.dynamic:
            occ |= (X - o.p0[0]) ** 2 + (Y - o.p0[1]) ** 2 <= o.radius**2
    grid = np.where(occ, OCCUPIED, FREE).astype(np.uint8)
    return np.flipud(grid), (xmin, ymin)


def write_map(sc: Scenario, out_dir: Path, resolution: float = RESOLUTION) -> Path:
    """写 out_dir/{name}/{name}.pgm + {name}.yaml，返回 yaml 路径。"""
    out = Path(out_dir) / sc.name
    out.mkdir(parents=True, exist_ok=True)
    grid, (xmin, ymin) = build_grid(sc, resolution)
    h, w = grid.shape
    pgm = out / f"{sc.name}.pgm"
    with open(pgm, "wb") as f:
        f.write(f"P5\n{w} {h}\n255\n".encode("ascii"))
        f.write(grid.tobytes())
    yaml_path = out / f"{sc.name}.yaml"
    yaml_path.write_text(
        f"image: {sc.name}.pgm\n"
        f"resolution: {resolution}\n"
        f"origin: [{xmin:.4f}, {ymin:.4f}, 0.0]\n"
        "negate: 0\n"
        "occupied_thresh: 0.65\n"
        "free_thresh: 0.25\n"
        "mode: trinary\n"
    )
    return yaml_path


def write_all(scenarios: list[Scenario], out_dir: Path, resolution: float = RESOLUTION) -> list[Path]:
    return [write_map(sc, out_dir, resolution) for sc in scenarios]


# ---- 自检（写盘后回读，连同 y 翻转一起验证） ----


def _read_pgm(path: Path) -> np.ndarray:
    with open(path, "rb") as f:
        assert f.readline().strip() == b"P5", f"{path} 不是 P5 PGM"
        line = f.readline()
        while line.startswith(b"#"):
            line = f.readline()
        w, h = (int(v) for v in line.split())
        f.readline()  # maxval
        return np.frombuffer(f.read(w * h), dtype=np.uint8).reshape(h, w)


def _read_yaml(path: Path) -> tuple[float, float, float]:
    text = path.read_text()
    m = re.search(r"origin:\s*\[([-\d.eE+]+),\s*([-\d.eE+]+)", text)
    res = float(re.search(r"resolution:\s*([\d.]+)", text).group(1))
    return float(m.group(1)), float(m.group(2)), res


def occupied_at(name: str, out_dir: Path, x: float, y: float) -> bool:
    out = Path(out_dir) / name
    data = _read_pgm(out / f"{name}.pgm")
    xmin, ymin, res = _read_yaml(out / f"{name}.yaml")
    h, w = data.shape
    col = int((x - xmin) / res)
    row = h - 1 - int((y - ymin) / res)  # PGM row 0 = y_max
    assert 0 <= col < w and 0 <= row < h, f"{name} ({x},{y}) 出界"
    return int(data[row, col]) < 128


def self_check(out_dir: Path) -> None:
    cases = [
        ("S1", 4.0, 0.0, True),   # 静障中心
        ("S1", 0.0, 0.0, False),  # 起点
        ("S1", 8.0, 0.0, False),  # 目标
        ("S3", 5.0, 0.8, True),   # 静障中心
        ("S3", 10.5, -0.5, False),  # 动态障碍初位——不进图
        ("S3", 10.0, 0.0, False),  # 目标
    ]
    for name, x, y, expect in cases:
        got = occupied_at(name, out_dir, x, y)
        assert got == expect, (
            f"{name} ({x},{y}) 期望{'占据' if expect else '自由'}，实得{'占据' if got else '自由'}"
        )
    s2 = _read_pgm(Path(out_dir) / "S2" / "S2.pgm")
    assert (s2 == FREE).all(), "S2 应全自由（仅含动态障碍，永不进图）"


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description="W3 场景地图栅格化（同源几何，动态障碍不进图）")
    ap.add_argument("--out", default="out/maps")
    ap.add_argument("--resolution", type=float, default=RESOLUTION)
    args = ap.parse_args()
    paths = write_all([f() for f in ALL], Path(args.out), args.resolution)
    for p in paths:
        print("map →", p)
    self_check(Path(args.out))
    print("mapping 自检 PASS（静态占据正确、动态障碍未进图）")
