#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""把 E14 的场景实例搭成 **Isaac Sim 场景**，用来"亲眼检查"五类场景。

它读 `scenarios/instances/<场景>_instances.yaml`（已冻结的实例），
在 Isaac 里生成：地面 + 机器人 + 静态障碍（圆柱/方块）+ 行人代理（会按真值轨迹动）
+ 起点标记 + 目标标记（绿色圆环）+ 灯光，并把视口相机摆到能一眼看全的位置。

常用用法：

    # 看某一个实例（开窗口）
    python3 src/robot_sim/robot_sim/scripts/scene_from_scenario.py --scenario static_avoid --run 1 --gui

    # 五类场景各看第一个实例，每 10 秒自动换下一个（自动巡览）
    python3 src/robot_sim/robot_sim/scripts/scene_from_scenario.py --all --cycle 10 --gui

    # 不开窗口，只导出成 USD（之后用 File > Open 打开，或给 E15 复用）
    python3 src/robot_sim/robot_sim/scripts/scene_from_scenario.py --all --save --headless

    # 想让它一直开着：--duration 0（Ctrl-C 退出）

注意：
  * 机器人用 assets/robot_a_chassis_agl_chs03_f01.usd 的**相对引用**（和 E05 场景一致）；
  * 场景只做可视化 + 碰撞体，不接 ROS、不驱动底盘；
  * 行人用圆柱代理（半径 0.3 m、高 1.7 m），按实例里的 start/velocity/start_time_s 匀速直线运动，
    到达 stop_at_y 后停住（与需求清单"不折返不停止"一致）。
"""

import argparse
import math
import os
import sys

ap = argparse.ArgumentParser()
ap.add_argument("--scenario", default="static_avoid",
                choices=["static_avoid", "dynamic_pedestrian", "narrow_passage",
                         "dead_end", "mixed"])
ap.add_argument("--run", type=int, default=1, help="实例序号 1~20")
ap.add_argument("--all", action="store_true", help="依次预览五类场景（各取 --run 指定的实例）")
ap.add_argument("--cycle", type=float, default=0.0,
                help="配合 --all：每 N 秒切换一个场景（0 = 只看第一个，手动切换需重跑）")
ap.add_argument("--scenario-dir", default="scenarios")
ap.add_argument("--rich", action="store_true",
                help="按 E14 要素富化场景（加更多障碍/行人，仍保证起点→目标可达）；"
                     "评分用的 instances/*.yaml 不受影响")
ap.add_argument("--save", action="store_true",
                help="把每个场景另存为 assets/scenes/scenarios/<场景>_<run>.usd")
ap.add_argument("--out-dir", default="assets/scenes/scenarios")
ap.add_argument("--duration", type=float, default=0.0, help="仿真播放时长（秒），0 = 一直开着")
ap.add_argument("--headless", action="store_true", default=False)
ap.add_argument("--gui", action="store_true", help="打开 Isaac 窗口（默认就是开窗口）")
args = ap.parse_args()

HEADLESS = args.headless and not args.gui
if not HEADLESS:
    os.environ.setdefault("DISPLAY", ":1")
else:
    os.environ.pop("DISPLAY", None)      # headless 带 DISPLAY 会卡在 GLFW 窗口插件初始化

if args.all and args.save and args.cycle <= 0:
    args.cycle = 4.0        # 导出模式：每 4 秒切一个场景，五个都会写到磁盘

seq = []
if args.all:
    for key in ("static_avoid", "dynamic_pedestrian", "narrow_passage", "dead_end", "mixed"):
        seq.append(key)
else:
    seq.append(args.scenario)

HERE = os.path.dirname(os.path.abspath(__file__))


def _repo_root():
    """从脚本位置往上找含 assets/ 的目录（脚本可能在 install/share 里）。"""
    cur = os.getcwd()
    for base in (cur, HERE):
        p = os.path.abspath(base)
        for _ in range(6):
            if os.path.isdir(os.path.join(p, "assets")):
                return p
            p = os.path.dirname(p)
    return cur


ROOT = _repo_root()
SCEN_DIR = os.path.join(ROOT, args.scenario_dir)
ROBOT_USD = os.path.join(ROOT, "assets/robot_a_chassis_agl_chs03_f01.usd")
OUT_DIR = os.path.join(ROOT, args.out_dir)

print("[SCENE] repo=%s headless=%s" % (ROOT, HEADLESS), flush=True)

import yaml  # noqa: E402

from isaacsim import SimulationApp  # noqa: E402

sim_app = SimulationApp({"headless": HEADLESS})

import numpy as np  # noqa: E402
import omni.kit.app  # noqa: E402
import omni.timeline  # noqa: E402
import omni.usd  # noqa: E402
from pxr import Gf, Usd, UsdGeom, UsdLux, UsdPhysics, UsdShade  # noqa: E402

STAGE = "/World"
DEFAULT_COLORS = {
    "obstacle": (0.55, 0.42, 0.30),
    "wall": (0.72, 0.70, 0.66),
    "pedestrian": (0.85, 0.25, 0.25),
    "goal": (0.15, 0.85, 0.25),
    "start": (0.25, 0.55, 0.95),
}


def set_translate(xformable, value):
    """设置（必要时新增）translate 操作。**引用的 USD 根可能已有 xformOp**，不能盲目 Add。"""
    xf = UsdGeom.Xformable(xformable)
    for op in xf.GetOrderedXformOps():
        if op.GetOpName() == "xformOp:translate":
            op.Set(Gf.Vec3d(*value))
            return
    xf.AddTranslateOp().Set(Gf.Vec3d(*value))


def add_box(stage, path, center, w, h, height=1.0, color=(0.7, 0.7, 0.7)):
    cube = UsdGeom.Cube.Define(stage, path)
    cube.CreateSizeAttr(1.0)
    xf = UsdGeom.Xformable(cube)
    xf.AddTranslateOp().Set(Gf.Vec3d(*center))
    xf.AddScaleOp().Set(Gf.Vec3f(w, h, height))
    cube.CreateDisplayColorAttr([Gf.Vec3f(*color)])
    UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
    return cube


def add_cylinder(stage, path, center, radius, height, color=(0.55, 0.42, 0.30)):
    cyl = UsdGeom.Cylinder.Define(stage, path)
    cyl.CreateRadiusAttr(radius)
    cyl.CreateHeightAttr(height)
    cyl.CreateAxisAttr("Z")
    xf = UsdGeom.Xformable(cyl)
    xf.AddTranslateOp().Set(Gf.Vec3d(*center))
    cyl.CreateDisplayColorAttr([Gf.Vec3f(*color)])
    UsdPhysics.CollisionAPI.Apply(cyl.GetPrim())
    return cyl


def add_ring(stage, path, center, radius, color, height=0.01):
    """目标/起点标记：薄圆盘（比 Cylinder 更好认）。"""
    cyl = UsdGeom.Cylinder.Define(stage, path)
    cyl.CreateRadiusAttr(radius)
    cyl.CreateHeightAttr(height)
    cyl.CreateAxisAttr("Z")
    UsdGeom.Xformable(cyl).AddTranslateOp().Set(Gf.Vec3d(center[0], center[1], center[2] + height))
    cyl.CreateDisplayColorAttr([Gf.Vec3f(*color)])
    return cyl


def _richify(scenario, inst):
    """调用基准包里的富化函数（同一 seed 结果确定）。"""
    try:
        sys.path.insert(0, os.path.join(ROOT, "src/robot_benchmark"))
        from robot_benchmark.scenario_gen import richify  # noqa: E402
        return richify(scenario, inst)
    except Exception as exc:
        print("[SCENE] 富化失败，使用原实例: %r" % (exc,), flush=True)
        return inst


def build_scene(scenario, inst):
    omni.usd.get_context().new_stage()
    for _ in range(3):
        sim_app.update()
    stage = omni.usd.get_context().get_stage()

    # ---- 物理场景 + 灯光 ----
    UsdPhysics.Scene.Define(stage, STAGE + "/PhysicsScene")
    dome = UsdLux.DomeLight.Define(stage, STAGE + "/Dome")
    dome.CreateIntensityAttr(800.0)
    sun = UsdLux.DistantLight.Define(stage, STAGE + "/Sun")
    sun.CreateIntensityAttr(2500.0)
    UsdGeom.Xformable(sun).AddRotateXYZOp().Set(Gf.Vec3f(-45.0, 0.0, 30.0))

    # ---- 地面（按场景范围裁剪，够看就行）----
    goal = inst["goal"]
    start = inst["robot_start"]
    span_x = max(abs(goal["x"]), abs(start["x"])) + 6.0
    ground = UsdGeom.Cube.Define(stage, STAGE + "/Ground")
    ground.CreateSizeAttr(1.0)
    gxf = UsdGeom.Xformable(ground)
    gxf.AddTranslateOp().Set(Gf.Vec3d(span_x / 2.0 - 2.0, 0.0, -0.02))
    gxf.AddScaleOp().Set(Gf.Vec3f(span_x * 2.0, 16.0, 0.02))
    ground.CreateDisplayColorAttr([Gf.Vec3f(0.42, 0.44, 0.46)])
    UsdPhysics.CollisionAPI.Apply(ground.GetPrim())

    # ---- 机器人（相对引用的 USD 资产）----
    robot = UsdGeom.Xform.Define(stage, STAGE + "/robot_a")
    robot.GetPrim().GetReferences().AddReference(ROBOT_USD)
    set_translate(robot, (start["x"], start["y"], 0.0))

    # ---- 静态障碍 ----
    n_cyl = n_box = 0
    for i, obs in enumerate(inst["obstacles"]):
        if obs["type"] == "circle":
            add_cylinder(stage, "%s/Obstacle_%02d" % (STAGE, i),
                         (obs["x"], obs["y"], 0.30), obs["r"], 0.60,
                         DEFAULT_COLORS["obstacle"])
            n_cyl += 1
        else:
            add_box(stage, "%s/Wall_%02d" % (STAGE, i),
                    (obs["x"], obs["y"], 0.50), obs["w"], obs["h"], 1.0,
                    DEFAULT_COLORS["wall"])
            n_box += 1

    # ---- 行人代理（按真值轨迹运动，由脚本每帧推移）----
    peds = []
    for i, d in enumerate(inst.get("dynamic", [])):
        prim = add_cylinder(stage, "%s/Pedestrian_%02d" % (STAGE, i),
                            (d["start"]["x"], d["start"]["y"], 0.85),
                            d["radius"], 1.70, DEFAULT_COLORS["pedestrian"])
        peds.append({"prim": prim, "spec": d, "y": d["start"]["y"]})

    # ---- 起点 / 目标标记 ----
    add_ring(stage, STAGE + "/StartMarker",
             (start["x"], start["y"], 0.0), 0.45, DEFAULT_COLORS["start"])
    add_ring(stage, STAGE + "/GoalMarker",
             (goal["x"], goal["y"], 0.0), 0.45, DEFAULT_COLORS["goal"])
    # 目标点上方一根细柱，远处也能看见
    pole = UsdGeom.Cylinder.Define(stage, STAGE + "/GoalPole")
    pole.CreateRadiusAttr(0.04)
    pole.CreateHeightAttr(1.2)
    UsdGeom.Xformable(pole).AddTranslateOp().Set(Gf.Vec3d(goal["x"], goal["y"], 0.6))
    pole.CreateDisplayColorAttr([Gf.Vec3f(*DEFAULT_COLORS["goal"])])

    return {"stage": stage, "peds": peds, "n_cyl": n_cyl, "n_box": n_box,
            "start": start, "goal": goal, "scenario": scenario, "inst": inst}


def set_view(scene):
    """把视口相机摆到能看全起点→目标的斜俯视角。"""
    if HEADLESS:
        return
    try:
        import omni.kit.viewport.utility as vp_util
        vp = vp_util.get_active_viewport()
        st, gl = scene["start"], scene["goal"]
        mid = ((st["x"] + gl["x"]) / 2.0, (st["y"] + gl["y"]) / 2.0, 0.4)
        dist = max(6.0, 0.9 * abs(gl["x"] - st["x"]) + 4.0)
        eye = (mid[0] - 0.45 * dist, mid[1] - 0.75 * dist, 0.85 * dist)
        vp_util.set_camera_view(eye=eye, target=mid, camera=vp.camera_path)
        print("[SCENE] 视口相机: eye=%s target=%s" % (tuple(round(v, 2) for v in eye),
                                                     tuple(round(v, 2) for v in mid)), flush=True)
    except Exception as exc:
        print("[SCENE] 设置视口相机失败（不影响查看）: %r" % (exc,), flush=True)


def load_instance(scenario, run):
    path = os.path.join(SCEN_DIR, "instances", "%s_instances.yaml" % scenario)
    with open(path, encoding="utf-8") as f:
        doc = yaml.safe_load(f)
    for inst in doc["instances"]:
        if inst["scenario_id"] == "%s_%02d" % (scenario, run):
            return inst
    raise SystemExit("找不到实例 %s_%02d（%s 里只有 %d 个）"
                     % (scenario, run, path, len(doc["instances"])))


def describe(scene):
    inst = scene["inst"]
    print("=" * 72, flush=True)
    print("[SCENE] 场景 %s｜实例 %s｜seed=%s" % (scene["scenario"], inst["scenario_id"], inst["seed"]), flush=True)
    print("[SCENE] 起点 (%.2f, %.2f) → 目标 (%.2f, %.2f)，时限 %ss，容差 %.2f m"
          % (inst["robot_start"]["x"], inst["robot_start"]["y"],
             inst["goal"]["x"], inst["goal"]["y"], inst["timeout_s"],
             inst["goal_tolerance"]["pos_m"]), flush=True)
    if inst.get("rich"):
        r = inst["rich"]
        print("[SCENE] 富化：障碍 %d→%d，行人 %d→%d（已重新校验可达性）"
              % (r["obstacles_base"], r["obstacles_total"],
                 r["dynamic_base"], r["dynamic_total"]), flush=True)
    print("[SCENE] 静态障碍 %d（圆柱 %d / 方块 %d），行人 %d"
          % (len(inst["obstacles"]), scene["n_cyl"], scene["n_box"], len(inst["dynamic"])), flush=True)
    if inst.get("passage"):
        p = inst["passage"]
        print("[SCENE] 窄通道：净宽 %.2f m（长 %.2f m），单侧余量 %.2f m（要求 ≥0.30）"
              % (p["net_width_m"], p["length_m"], p["margin_per_side_m"]), flush=True)
    if inst.get("dead_end"):
        d = inst["dead_end"]
        print("[SCENE] 死角：袋口 %.2f m、袋深 %.2f m，需要倒车退出"
              % (d["mouth_width_m"], d["depth_m"]), flush=True)
    for p in scene["peds"]:
        s = p["spec"]
        print("[SCENE] 行人：起点 (%.2f, %.2f)，速度 (%.2f, %.2f) m/s，启动 %.2f s，停于 y=%.2f"
              % (s["start"]["x"], s["start"]["y"], s["velocity"]["vx"], s["velocity"]["vy"],
                 s["start_time_s"], s["stop_at_y"]), flush=True)


def animate(scene, t):
    """行人匀速直线运动（真值轨迹）；到 stop_at_y 停住。"""
    for p in scene["peds"]:
        s = p["spec"]
        dt = t - s["start_time_s"]
        if dt <= 0.0:
            y = s["start"]["y"]
        else:
            y = s["start"]["y"] + s["velocity"]["vy"] * dt
            if s["velocity"]["vy"] > 0:
                y = min(y, s["stop_at_y"])
            else:
                y = max(y, s["stop_at_y"])
        if abs(y - p["y"]) < 1e-6:
            continue
        p["y"] = y
        UsdGeom.Xformable(p["prim"]).GetTranslateOp().Set(
            Gf.Vec3d(s["start"]["x"], y, 0.85))


def main():
    i = 0
    scene = None
    t_switch = 0.0
    timeline = omni.timeline.get_timeline_interface()

    def load_next():
        nonlocal i, scene, t_switch
        key = seq[i % len(seq)]
        inst = load_instance(key, args.run)
        if args.rich:
            inst = _richify(key, inst)
        scene = build_scene(key, inst)
        describe(scene)
        set_view(scene)
        if not timeline.is_playing():
            timeline.play()
        sim_app.update()
        if args.save:
            os.makedirs(OUT_DIR, exist_ok=True)
            path = os.path.join(OUT_DIR, "%s_%02d.usd" % (key, args.run))
            # Keep runtime references absolute, but author the saved layer relative to its destination.
            from pxr import Sdf
            saved_layer = Sdf.Layer.CreateAnonymous()
            saved_layer.TransferContent(scene["stage"].GetRootLayer())
            relative_robot = os.path.relpath(ROBOT_USD, os.path.dirname(path)).replace(os.sep, "/")
            saved_layer.UpdateExternalReference(ROBOT_USD, relative_robot)
            saved_layer.Export(path)
            print("[SCENE] 已导出 USD: %s" % path, flush=True)
        t_switch = 0.0
        i += 1

    load_next()
    import time
    t0 = time.time()
    last_print = 0.0
    while True:
        sim_app.update()
        wall = time.time() - t0
        try:
            t = float(timeline.get_current_time())
        except Exception:
            t = 0.0
        if scene is not None:
            animate(scene, t if t > 0 else wall)
        if wall - last_print > 5.0:
            last_print = wall
            print("[SCENE] 运行中 t=%.1fs（%s）" % (wall, scene["scenario"] if scene else "-"), flush=True)
        if args.all and args.cycle > 0 and wall - t_switch > args.cycle and i < len(seq):
            load_next()
        if args.duration > 0 and wall > args.duration:
            break
    print("[SCENE] 结束。Ctrl-C 之前看到的就是该实例的真实几何", flush=True)
    sim_app.close()
    return 0



if __name__ == "__main__":
    sys.exit(main())
