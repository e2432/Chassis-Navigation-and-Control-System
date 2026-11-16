"""按契约清单（docs/scenarios/instances.yaml）生成评分用 Isaac 场景。

为什么不用 `robot_sim/scripts/scene_from_scenario.py`：
  * 那个脚本必须 `from isaacsim import SimulationApp` 起一整个 Kit 才能建 USD，
    生成一个场景约 40 s；9-2 官方采集要 5 场景 × 20 seed = 100 个场景，
    光生成就要一小时以上，而且每个场景都要占一次 GPU。
  * 本模块直接写 USD 的**文本格式** `.usda`：不需要 pxr、不需要 Kit、毫秒级，
    场景内容与 `scene_from_scenario.build_scene()` 对齐（几何、物理场景、光照、
    机器人引用、Start/Goal 标记、layer 自定义数据里的 scenario/seed/run）。
  * 文本格式还有一个工程好处：场景能进 diff，评审能直接看出每个 seed 放了什么障碍。

口径（与契约 v1.4 / 9-1 一致）：
  * 只有 `docs/scenarios/instances.yaml` 是评分清单；生成文件名为
    `<scenario>_<run:02d>.usda`，与清单的 (scenario, run) 一一对应。
  * 行人在这里只冻结 t=0 位姿：静态 USD 无法做时间动画，动态场景的推进必须由
    Isaac 侧按契约公式 `pedestrian_position()` 驱动（见 docs/9-2 的"未完成"节）。
"""

from __future__ import annotations

import json
import math
import os
from pathlib import Path

from .scenario_gen import pedestrian_position

# 与 scene_from_scenario.py 保持一致的视觉/物理常量
OBSTACLE_HEIGHT = 0.6
OBSTACLE_COLOR = (0.55, 0.42, 0.3)
WALL_HEIGHT = 1.0
WALL_COLOR = (0.7, 0.7, 0.66)
PED_HEIGHT = 1.7
PED_COLOR = (0.8, 0.25, 0.25)
GROUND_COLOR = (0.35, 0.38, 0.4)
MARKER_COLOR = (0.2, 0.5, 0.9)
GOAL_COLOR = (0.2, 0.8, 0.3)


def _vec3(x: float, y: float, z: float) -> str:
    return "(%.6f, %.6f, %.6f)" % (x, y, z)


def _xform_lines(translate=None, rotate_z_deg=None, scale=None) -> list:
    ops, lines = [], []
    if translate is not None:
        ops.append("xformOp:translate")
        lines.append("double3 xformOp:translate = %s" % _vec3(*translate))
    if rotate_z_deg is not None:
        ops.append("xformOp:rotateZ")
        lines.append("double xformOp:rotateZ = %.6f" % rotate_z_deg)
    if scale is not None:
        ops.append("xformOp:scale")
        lines.append("double3 xformOp:scale = %s" % _vec3(*scale))
    if ops:
        lines.append("uniform token[] xformOpOrder = [%s]"
                     % ", ".join('"%s"' % o for o in ops))
    return lines


def _prim(kind: str, name: str, attrs: list, indent: str, collision: bool) -> str:
    api = ""
    if collision:
        api = (" (\n" + indent + '    prepend apiSchemas = ["PhysicsCollisionAPI"]\n'
               + indent + ")")
    inner = indent + "    "
    body = "\n".join(inner + a for a in attrs)
    return '%sdef %s "%s"%s\n%s{\n%s\n%s}\n' % (indent, kind, name, api, indent, body, indent)


def _cylinder(name, xyz, radius, height, color, indent, collision=True) -> str:
    attrs = [
        "double radius = %.6f" % radius,
        "double height = %.6f" % height,
        'uniform token axis = "Z"',
        "color3f[] primvars:displayColor = [(%.3f, %.3f, %.3f)]" % color,
    ] + _xform_lines(translate=xyz)
    return _prim("Cylinder", name, attrs, indent, collision)


def _cube(name, xyz, scale, color, indent, collision=True) -> str:
    attrs = [
        "double size = 1.0",
        "color3f[] primvars:displayColor = [(%.3f, %.3f, %.3f)]" % color,
    ] + _xform_lines(translate=xyz, scale=scale)
    return _prim("Cube", name, attrs, indent, collision)


def scene_usda(inst: dict, world_pose: dict, robot_href: str) -> str:
    """返回一个场景的 .usda 文本；robot_href 是相对场景文件所在目录的机器人 USD 路径。"""
    wx = float(world_pose.get("x", 0.0))
    wy = float(world_pose.get("y", 0.0))
    yaw_deg = math.degrees(float(world_pose.get("yaw", 0.0)))
    has_yaw = abs(yaw_deg) > 1e-9

    frame_head = '    def Xform "ScenarioFrame"\n    {\n'
    frame_ops = "\n".join("        " + s for s in
                          _xform_lines(translate=(wx, wy, 0.0),
                                       rotate_z_deg=yaw_deg if has_yaw else None))
    children = []
    # 地面：长 ≥ 路径长度 + 6，宽 20（与 build_scene 一致）
    children.append(_cube("Ground", (inst["goal"]["x"] / 2.0, 0.0, -0.03),
                          (inst["path_length_m"] + 6.0, 20.0, 0.05),
                          GROUND_COLOR, "        "))
    for i, o in enumerate(inst.get("obstacles") or []):
        children.append(_cylinder("Obstacle_%02d" % i, (o["x"], o["y"], OBSTACLE_HEIGHT / 2.0),
                                  o["r"], OBSTACLE_HEIGHT, OBSTACLE_COLOR, "        "))
    for i, w in enumerate(inst.get("walls") or []):
        children.append(_cube("Wall_%02d" % i,
                              ((w["x0"] + w["x1"]) / 2.0, (w["y0"] + w["y1"]) / 2.0, 0.5),
                              (w["x1"] - w["x0"], w["y1"] - w["y0"], WALL_HEIGHT),
                              WALL_COLOR, "        "))
    for i, p in enumerate(inst.get("pedestrians") or []):
        (px, py) = pedestrian_position(p, 0.0)      # 冻结 t=0 位姿
        children.append(_cylinder("Pedestrian_%02d" % i, (px, py, PED_HEIGHT / 2.0),
                                  p["r"], PED_HEIGHT, PED_COLOR, "        "))
    children.append(_cylinder("Start", (0.0, 0.0, 0.015), 0.25, 0.02,
                              MARKER_COLOR, "        ", collision=False))
    children.append(_cylinder("Goal", (inst["goal"]["x"], inst["goal"]["y"], 0.015),
                              0.25, 0.02, GOAL_COLOR, "        ", collision=False))

    # 行人参数以 JSON 塞进 layer 自定义数据：Isaac 侧按契约公式推进位姿
    # y = dir · speed · (t − t_meet)，x = x_cross（匀速直线、无停住、无钳位）。
    peds_json = json.dumps(inst.get("pedestrians") or [], ensure_ascii=False)
    layer_data = (
        "    customLayerData = {\n"
        '        string scenario = "%s"\n'
        "        int seed = %d\n"
        "        int run = %d\n"
        '        string generator_version = "2.0.0"\n'
        '        string coordinate_frame = "path"\n'
        '        string pedestrians_json = "%s"\n'
        "    }\n" % (inst["scenario"], inst["seed"], inst["run"],
                    peds_json.replace("\\", "\\\\").replace('"', '\\\"')))

    robot_block = (
        '    def Xform "robot_a" (\n'
        "        prepend references = @%s@\n"
        "    )\n"
        "    {\n"
        "        double3 xformOp:translate = %s\n"
        "        double xformOp:rotateZ = %.6f\n"
        '        uniform token[] xformOpOrder = ["xformOp:translate", "xformOp:rotateZ"]\n'
        "    }\n" % (robot_href, _vec3(wx, wy, 0.0), yaw_deg))

    return (
        "#usda 1.0\n"
        "(\n"
        '    defaultPrim = "World"\n'
        "    metersPerUnit = 1\n"
        '    upAxis = "Z"\n'
        "%s"
        ")\n\n"
        'def Xform "World"\n'
        "{\n"
        '    def PhysicsScene "PhysicsScene"\n'
        "    {\n"
        "        float physics:gravityMagnitude = 0.0\n"
        "        vector3f physics:gravityDirection = (0, 0, -1)\n"
        "    }\n\n"
        '    def DomeLight "Dome"\n'
        "    {\n"
        "        float inputs:intensity = 700.0\n"
        "    }\n\n"
        "%s"
        "%s"
        "    }\n\n"
        "%s"
        "%s"
        "}\n"
    ) % (layer_data, frame_head, frame_ops + "\n", "".join(children), robot_block)


def write_scene(inst: dict, world_pose: dict, out_path, robot_usd) -> str:
    """写出场景文件，返回路径字符串。robot_usd 会写成相对 out_path 目录的引用。"""
    out_path = Path(out_path)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    href = os.path.relpath(Path(robot_usd), out_path.parent).replace(os.sep, "/")
    out_path.write_text(scene_usda(inst, world_pose, href), encoding="utf-8")
    return str(out_path)


def scene_path_for(out_root, inst: dict) -> Path:
    """约定路径：<out_root>/scenes/<scenario>_<run:02d>.usda"""
    return Path(out_root) / "scenes" / ("%s_%02d.usda" % (inst["scenario"], inst["run"]))


# --------------------------------------------------------------------------- #
# 评分用静态地图（供 Nav2 global_costmap 的 static_layer）
# --------------------------------------------------------------------------- #
def build_occupancy(inst: dict, world_pose: dict, res: float = 0.05,
                    half_extent: float = 20.0) -> dict:
    """按契约实例生成 OccupancyGrid 内容（map 系，轴向与 Nav2 一致）。

    为什么需要静态地图层：Isaac 的射线雷达每帧只有约 500 个点（pointid_step=12），
    标出来的障碍是**带孔洞的细弧**；SmacPlanner2D 会从孔洞里穿过去，DWB 到了跟前
    就找不到可行轨迹，机器人停在原地直到 progress_checker 报失败（2026-11-14 实测
    反复停在 x≈4.5 与 x≈7.96）。评分场景由 docs/scenarios/instances.yaml 完全确定，
    所以全局规划层直接用清单几何，**局部避障仍然用实时 /lidar/points**。

    返回 {width, height, resolution, origin_x, origin_y, data}（data 为 int8 列表：
    100=占用，0=空闲）。行人不进静态地图（会动）。
    """
    n = int(round(2.0 * half_extent / res))
    ox = oy = -half_extent
    data = [0] * (n * n)

    def mark_cell(ix: int, iy: int):
        if 0 <= ix < n and 0 <= iy < n:
            data[iy * n + ix] = 100

    # map 系 → path frame（实例几何是 path frame）
    wx = float(world_pose.get("x", 0.0))
    wy = float(world_pose.get("y", 0.0))
    wyaw = float(world_pose.get("yaw", 0.0))
    c, s = math.cos(-wyaw), math.sin(-wyaw)

    def to_path(x: float, y: float):
        dx, dy = x - wx, y - wy
        return c * dx - s * dy, s * dx + c * dy

    # 关键：几何是 **path frame** 的，而代价地图是 **map frame**，两者差一个 world_pose
    # （可能带旋转）。迭代范围必须在 map 系里算：先把几何变换到 map 系取包围盒，
    # 再用"格心反变换回 path 系"做精确判定。
    #
    # 2026-11-15 实测的两个坑（都出在这里）：
    #   1) 迭代范围正好等于几何包围盒 + 只判格心 → 比一格的几何一个格心都落不进去；
    #   2) world_pose.yaw != 0 时按未旋转包围盒迭代 → dead_end 5 个种子里 4 个、
    #      narrow_passage 4 个的静态图**一格都标不上**，机器人拿着"没有墙的地图"直接
    #      穿墙，13/15 失败、间距恒为 0。这就是 dead_end 成功率的真正瓶颈。
    c_w, s_w = math.cos(wyaw), math.sin(wyaw)

    def to_map(lx: float, ly: float):
        return c_w * lx - s_w * ly + wx, s_w * lx + c_w * ly + wy

    PAD_CELLS = 2

    def mark_range(pts):
        """把 map 系多边形 pts 的包围盒内、且格心落在 path 系多边形内的格子标脏。"""
        xs = [p[0] for p in pts]
        ys = [p[1] for p in pts]
        ix0 = max(0, int((min(xs) - ox) / res) - PAD_CELLS)
        ix1 = min(n - 1, int((max(xs) - ox) / res) + 1 + PAD_CELLS)
        iy0 = max(0, int((min(ys) - oy) / res) - PAD_CELLS)
        iy1 = min(n - 1, int((max(ys) - oy) / res) + 1 + PAD_CELLS)
        for ix in range(ix0, ix1 + 1):
            for iy in range(iy0, iy1 + 1):
                lx, ly = to_path(ox + (ix + 0.5) * res, oy + (iy + 0.5) * res)
                yield ix, iy, lx, ly

    for ob in inst.get("obstacles") or []:
        r = float(ob["r"])
        (cxm, cym) = to_map(float(ob["x"]), float(ob["y"]))
        box = [(cxm - r, cym - r), (cxm + r, cym + r)]
        for ix, iy, lx, ly in mark_range(box):
            if (lx - ob["x"]) ** 2 + (ly - ob["y"]) ** 2 <= r * r:
                mark_cell(ix, iy)
    for w in inst.get("walls") or []:
        x0, x1 = float(w["x0"]), float(w["x1"])
        y0, y1 = float(w["y0"]), float(w["y1"])
        box = [to_map(x0, y0), to_map(x1, y0), to_map(x1, y1), to_map(x0, y1)]
        for ix, iy, lx, ly in mark_range(box):
            if x0 <= lx <= x1 and y0 <= ly <= y1:
                mark_cell(ix, iy)
    return {"width": n, "height": n, "resolution": res,
            "origin_x": ox, "origin_y": oy, "data": data}


def write_nav2_map(inst: dict, world_pose: dict, out_stem) -> dict:
    """按契约实例写出 Nav2 的 .pgm + .yaml（供 nav2_map_server 加载）。

    为什么不用 driver 里自建 OccupancyGrid 直接发 /map：实测在 dead_end 这类
    带旋转 world_pose 的实例上，全局代价地图的 static_layer 收到了地图（日志有
    `Resizing costmap to 800 X 800`）却不产生致命格，机器人照样穿墙。
    map_server 是 Nav2 官方配套的发布者，走它的标准链路更可靠。

    返回 {"pgm": 路径, "yaml": 路径, "occupied_cells": 数量}
    """
    occ = build_occupancy(inst, world_pose)
    n = occ["width"]
    res = occ["resolution"]
    # P5 灰度：0=占用（黑），254=空闲（白）。本实验没有未知区域。
    pixels = bytearray(n * n)
    n_occ = 0
    for i, v in enumerate(occ["data"]):
        if v:
            pixels[i] = 0
            n_occ += 1
        else:
            pixels[i] = 254
    stem = Path(out_stem)
    pgm = stem.with_suffix(".pgm")
    yml = stem.with_suffix(".yaml")
    pgm.parent.mkdir(parents=True, exist_ok=True)
    # PGM 的行序：图像第一行是 map 的**最大 y**，所以按行翻转写
    with open(pgm, "wb") as f:
        f.write(b"P5\n# generated from docs/scenarios/instances.yaml\n")
        f.write(("%d %d\n255\n" % (n, n)).encode("ascii"))
        for iy in range(n - 1, -1, -1):
            f.write(bytes(pixels[iy * n:(iy + 1) * n]))
    yml.write_text(
        "image: %s\n"
        "resolution: %.4f\n"
        "origin: [%.4f, %.4f, 0.0]\n"
        "negate: 0\n"
        "occupied_thresh: 0.65\n"
        "free_thresh: 0.196\n"
        "mode: trinary\n"
        % (os.path.basename(pgm), res, occ["origin_x"], occ["origin_y"]),
        encoding="utf-8")
    return {"pgm": str(pgm), "yaml": str(yml), "occupied_cells": n_occ}
