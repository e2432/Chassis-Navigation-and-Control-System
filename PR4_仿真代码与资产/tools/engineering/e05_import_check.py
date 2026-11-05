#!/usr/bin/env python3
"""E05：把最小底盘模型导入 Isaac Sim 5.1，并做静态稳定性验收。

做四件事：
  1) 打印 URDF 导入器配置项（不同 Kit 版本属性名不同，先探明再设）；
  2) 空场景 + 地面 + 物理场景，导入 src/robot_description/urdf/agl_chs_03_f01.urdf；
  3) 播放 2 s 仿真时间，采样底盘位姿，检查：初始不穿透、静置不漂移、不持续抖动；
  4) 把机器人导出成 USD 资产（assets/ 下），供后续 GUI 打开与场景复用。

判定阈值（相对 PDF 参数）：
  * 底盘离地间隙 25 mm：稳定后底盘底面 z ∈ [0.015, 0.035] m
  * base_link 静置高度应≈轮半径 0.105 m（±10 mm）
  * 2 s 内水平漂移 < 10 mm；z 抖动峰峰值 < 5 mm；俯仰/横滚 < 2°
"""

import csv
import os
import sys
import time

OUT_DIR = os.path.abspath(sys.argv[1] if len(sys.argv) > 1 else "results/w2/e05")
os.makedirs(OUT_DIR, exist_ok=True)
SAMPLES = os.path.join(OUT_DIR, "settle_samples.csv")
REPORT = os.path.join(OUT_DIR, "import_check.txt")

XACRO = os.path.abspath("src/robot_description/urdf/agl_chs_03_f01.urdf.xacro")
# Isaac Sim 的 URDF 导入器不认 xacro：先展开成纯 URDF，展开结果作为证据保留
URDF = os.path.join(OUT_DIR, "agl_chs_03_f01_expanded.urdf")
USD_OUT = os.path.abspath("assets/robot_a_chassis_agl_chs03_f01.usd")

lines = []


def say(msg):
    print(msg, flush=True)
    lines.append(msg)


T0 = time.time()
say("# E05 最小底盘模型导入与静态稳定性检查")
say("时间: %s" % time.strftime("%Y-%m-%dT%H:%M:%S"))
say("xacro 源文件: %s" % XACRO)

# 展开 xacro（xacro 在 /opt/ros/humble/bin，e01_run_sim_startup.sh 已 source 过 ROS 2）
import subprocess  # noqa: E402

xacro_bin = "/opt/ros/humble/bin/xacro"
# 关键：Isaac 的 URDF 导入器无法解析 package://（工作空间没 source 时直接跳过网格，
# 车体会“消失”）。这里把 mesh_dir 覆盖成绝对 file:// URI，导入才拿得到车体网格。
MESH_URI = "file://" + os.path.abspath("src/robot_description/meshes")
with open(URDF, "w") as fh:
    proc = subprocess.run([xacro_bin, XACRO, "mesh_dir:=%s" % MESH_URI],
                          stdout=fh, stderr=subprocess.PIPE, text=True)
if proc.returncode != 0:
    say("FATAL: xacro 展开失败: %s" % proc.stderr.strip()[:500])
    sys.exit(3)
import hashlib  # noqa: E402

raw = open(URDF, "rb").read()
say("展开后 URDF: %s（%d 字节, sha256=%s）"
    % (URDF, len(raw), hashlib.sha256(raw).hexdigest()[:16]))
say("xacro 命令: %s %s mesh_dir:=%s" % (xacro_bin, os.path.relpath(XACRO), MESH_URI))
if "package://" in raw.decode("utf-8", "ignore"):
    say("WARN: 展开后的 URDF 仍含 package:// 路径，导入器可能解析失败")

from isaacsim import SimulationApp  # noqa: E402

sim_app = SimulationApp({"headless": True})
say("SimulationApp ready in %.1f s" % (time.time() - T0))

import omni.kit.app  # noqa: E402
import omni.kit.commands  # noqa: E402
import omni.timeline  # noqa: E402
import omni.usd  # noqa: E402
from pxr import Gf, UsdGeom, UsdPhysics  # noqa: E402

app = omni.kit.app.get_app()
ext_mgr = app.get_extension_manager()
for ext in ("isaacsim.asset.importer.urdf",):
    if not ext_mgr.is_extension_enabled(ext):
        ext_mgr.set_extension_enabled_immediate(ext, True)
        sim_app.update()
    say("扩展 %s enabled=%s" % (ext, ext_mgr.is_extension_enabled(ext)))

stage = omni.usd.get_context().get_stage()
say("初始 stage: %s" % stage.GetRootLayer().identifier)


def ensure_world_scaffold():
    """空场景里补 ground plane 与物理场景（缺什么补什么）。"""
    if not stage.GetPrimAtPath("/World"):
        UsdGeom.Xform.Define(stage, "/World")
    if not stage.GetPrimAtPath("/World/PhysicsScene"):
        scene = UsdPhysics.Scene.Define(stage, "/World/PhysicsScene")
        scene.CreateGravityDirectionAttr(Gf.Vec3f(0, 0, -1))
        scene.CreateGravityMagnitudeAttr(9.81)
    if not stage.GetPrimAtPath("/World/GroundPlane"):
        plane = UsdGeom.Plane.Define(stage, "/World/GroundPlane")
        plane.CreateAxisAttr("Z")
        plane.CreateExtentAttr([Gf.Vec3f(-50, -50, 0), Gf.Vec3f(50, 50, 0)])
        prim = plane.GetPrim()
        UsdPhysics.CollisionAPI.Apply(prim)
    return True


ensure_world_scaffold()
say("场景骨架: /World/PhysicsScene + /World/GroundPlane 就绪")

# World 负责物理场景/sim view 的初始化；单独用 Articulation.initialize() 会因为
# simulation_view 还没建好而报 AttributeError: 'NoneType' object has no attribute ...
from isaacsim.core.api import World  # noqa: E402

world = World(stage_units_in_meters=1.0, physics_dt=1.0 / 60.0, rendering_dt=1.0 / 60.0)
say("World 已创建（physics_dt=1/60）")

# ---- 1. 导入器配置项探测 ----
# 注意：omni.kit.commands.execute 返回 (status, result)，result 才是 ImportConfig。
# 直接取 [0] 会拿到 bool，随后 parse_urdf 会报 "incompatible function arguments"。
_cfg_res = omni.kit.commands.execute("URDFCreateImportConfig")
cfg = _cfg_res[-1] if isinstance(_cfg_res, (tuple, list)) else _cfg_res
attrs = [a for a in dir(cfg) if not a.startswith("_") and not a.startswith("set_")]
say("导入器配置项(%d 个): %s" % (len(attrs), ", ".join(attrs)))

WANTED = {
    "fix_base": False,                 # 底盘可自由落地（不留固定基座）
    "merge_fixed_joints": False,       # 保留 base_footprint / 雷达 / IMU 坐标帧
    "collision_from_visuals": False,   # 使用 URDF 里显式的碰撞几何
    "self_collision": False,
    "create_physics_scene": False,     # 场景已有物理场景
    "import_inertia_tensor": False,    # 用 URDF 里写的惯量，不重新计算
    "make_default_prim": True,
    "replace_cylinders_with_capsules": False,  # 轮子保持圆柱，不要换成胶囊
    # 关键：不实例化。实例代理(instance proxy)不允许逐图元改属性，
    # 会导致“无法关闭底盘碰撞”，进而运动学模式写位姿时 PhysX 报穿透错误。
    "make_instanceable": False,
}
applied = []
for key, value in WANTED.items():
    if hasattr(cfg, key):
        try:
            setattr(cfg, key, value)
            applied.append("%s=%s" % (key, value))
        except Exception as exc:  # pragma: no cover
            applied.append("%s 设置失败(%s)" % (key, exc))
    else:
        applied.append("%s(不存在)" % key)
say("已应用的配置: %s" % ", ".join(applied))

# ---- 2. 导入 ----
status, robot_path = omni.kit.commands.execute(
    "URDFParseAndImportFile",
    urdf_path=URDF,
    import_config=cfg,
    dest_path="",
    get_articulation_root=True,
)
say("导入 status=%s articulation_root=%s" % (status, robot_path))
if not robot_path:
    say("FATAL: 导入失败")
    sim_app.close()
    sys.exit(2)

robot_prim = stage.GetPrimAtPath(robot_path)
say("robot prim valid=%s type=%s" % (robot_prim.IsValid(), robot_prim.GetTypeName()))


def list_tree(path, depth=0, max_depth=3):
    prim = stage.GetPrimAtPath(path)
    if not prim.IsValid() or depth > max_depth:
        return
    say("  " * depth + "- %s (%s)" % (prim.GetName(), prim.GetTypeName()))
    for child in prim.GetChildren():
        list_tree(child.GetPath().pathString, depth + 1, max_depth)


say("prim 结构（≤3 层）:")
list_tree(robot_path)

# 平面包络（图纸 726 × 726，含轮）：用导入后的轮子实际世界位姿解析计算
# 说明：URDF 导入器不写 extents，USD BBoxCache 在这里会返回空包围盒（FLT_MAX），
#       所以改成从各轮的连杆世界位姿 + 已知轮径/轮宽反算包络 —— 这也更贴近“模型真的在哪”。
from isaacsim.core.prims import XFormPrim  # noqa: E402  （必须在导入后、取位姿前）

# 模型关键尺寸（与 xacro 默认值一致；用于解析计算平面包络）
wheel_radius = 0.105      # 8 寸 = 210 mm
wheel_width = 0.060       # 假设值

wheel_world = {}
for tag in ("fl", "fr", "rl", "rr"):
    pth = robot_path.replace("/base_footprint", "") + "/wheel_%s_link" % tag
    xf = XFormPrim(pth)
    pos, ori = xf.get_world_poses()
    wheel_world[tag] = (pos[0].tolist(), ori[0].tolist())

xs, ys = [], []
for tag, (c, q) in wheel_world.items():
    # 轮轴方向 = 轮连杆相对机体的 45° 方向（由 URDF 定义），与轮径/轮宽合成平面矩形
    import math as _m
    ang = {"fl": 45.0, "fr": -45.0, "rl": 135.0, "rr": -135.0}[tag]
    ax = _m.radians(ang)
    n = (_m.cos(ax), _m.sin(ax))                      # 轮轴水平投影
    u = (-n[1], n[0])                                  # 轮面内的水平方向
    for su in (1.0, -1.0):
        for sv in (1.0, -1.0):
            xs.append(c[0] + su * wheel_radius * u[0] + sv * (wheel_width / 2.0) * n[0])
            ys.append(c[1] + su * wheel_radius * u[1] + sv * (wheel_width / 2.0) * n[1])
size_x, size_y = max(xs) - min(xs), max(ys) - min(ys)
# 跨轮距离（垂直于 45° 斜面，对应图纸上的 726 尺寸线）
import math as _mm
across = 2.0 * (abs(wheel_world["fl"][0][0]) * _mm.sqrt(2.0) + wheel_width / 2.0)
say("轮心世界位姿与包络:")
for tag, (c, q) in wheel_world.items():
    say("  wheel_%s: (%.4f, %+0.4f, %.4f) m" % (tag, c[0], c[1], c[2]))
say("含轮 X/Y 外接包络: %.1f × %.1f mm（由 跨轮726 与轮宽推导得 662）" % (size_x * 1000, size_y * 1000))
say("跨轮距离（垂直 45° 斜面）: %.1f mm（图纸 726）" % (across * 1000))

# 网格加载检查（车体是 mesh，这一项之前漏了，导致“车体消失”没被发现）
# 注意：USD 实例代理默认不会被 stage.Traverse() 走过，必须用 TraverseAll()
_all = stage.TraverseAll() if hasattr(stage, "TraverseAll") else stage.Traverse()
mesh_prims = [p for p in _all if p.GetTypeName() == "Mesh"]
mesh_files = sorted({p.GetAttribute("filePath").Get() for p in mesh_prims
                     if p.GetAttribute("filePath") and p.GetAttribute("filePath").Get()})
say("Mesh prim 数: %d；引用的网格文件: %s" % (len(mesh_prims), mesh_files[:3]))
# 车体网格尺寸核对（应≈ 643 × 656 × 226 mm）
chassis_ok = False
if mesh_prims:
    cache = UsdGeom.BBoxCache(0, ["default"])
    for mp in mesh_prims:
        b = cache.ComputeWorldBound(mp).ComputeAlignedRange()
        lo, hi = b.GetMin(), b.GetMax()
        if lo[0] > 1e30:
            continue
        ex, ey, ez = (hi[0] - lo[0]) * 1000, (hi[1] - lo[1]) * 1000, (hi[2] - lo[2]) * 1000
        say("  网格 %s: %.1f × %.1f × %.1f mm" % (mp.GetName(), ex, ey, ez))
        if abs(ex - 643) < 5 and abs(ey - 656) < 5:
            chassis_ok = True

# 关节数量（URDF 里 4 个连续关节）
joint_prims = [p for p in stage.Traverse() if p.GetTypeName() == "PhysicsRevoluteJoint"]
say("物理旋转关节数: %d（期望 4：四个轮）" % len(joint_prims))

# ---- 3. 物理稳定性 ----
from isaacsim.core.prims import Articulation, XFormPrim  # noqa: E402

timeline = omni.timeline.get_timeline_interface()
# world.reset() 建立 physics simulation view；之后 articulation 才能 initialize
world.reset()
art = Articulation(prim_paths_expr=robot_path)
art.initialize()
say("articulation num_dof=%d body_count=%d" % (art.num_dof, art.get_articulation_body_count()))

# 位姿必须从 base_link（真实刚体）读，而不是 articulation root（base_footprint 是静态帧）。
# update_fabric=True 让物理结果同步到 fabric，XFormPrim 才能读到步进后的世界位姿。
base_link_path = robot_path.replace("/base_footprint", "") + "/base_link"
base_xform = XFormPrim(base_link_path)
say("位姿采样点: %s" % base_link_path)

# 物理要真正推进必须播放时间轴；否则 world.step() 只是刷新界面，位姿恒定不变
timeline.play()
for _ in range(5):
    sim_app.update()
say("timeline.is_playing=%s sim_time=%.3f" % (timeline.is_playing(), timeline.get_current_time()))

records = []


def step():
    world.step(render=False, update_fabric=True)
    sim_t = timeline.get_current_time()
    pos, ori = base_xform.get_world_poses()
    xyz = pos[0].tolist()
    wxyz = ori[0].tolist()
    records.append((sim_t, xyz[0], xyz[1], xyz[2], wxyz[0], wxyz[1], wxyz[2], wxyz[3]))


# 预热 0.2 s，让接触建立
for _ in range(12):
    step()
settle_start = len(records)
for _ in range(120):   # 2 s @ 60 Hz
    step()

with open(SAMPLES, "w", newline="") as f:
    w = csv.writer(f)
    w.writerow(["sim_time_s", "x", "y", "z", "qw", "qx", "qy", "qz"])
    w.writerows([(round(r[0], 5),) + tuple(round(v, 6) for v in r[1:]) for r in records])
say("采样写入 %s（%d 行）" % (SAMPLES, len(records)))

tail = records[settle_start:]
zs = [r[3] for r in tail]
xs = [r[1] for r in tail]
ys = [r[2] for r in tail]


import math  # noqa: E402


def tilt_deg_from_quat(qw, qx, qy, qz):
    # 世界系到机体：取机体 z 轴与重力方向的夹角
    # R 的第三列 = 机体 z 轴在世界系
    zx = 2 * (qx * qz + qw * qy)
    zy = 2 * (qy * qz - qw * qx)
    zz = 1 - 2 * (qx * qx + qy * qy)
    return math.degrees(math.acos(max(-1.0, min(1.0, zz))))


tilts = [tilt_deg_from_quat(*r[4:8]) for r in tail]
wheel_radius = 0.105
wheel_width = 0.060                 # 假设，与 xacro 的 wheel_width 默认值保持一致
# 八边形车体：底面离地 25 mm，base_link 在轮轴平面（离地 105 mm）
chassis_bottom_rel = 0.025 - wheel_radius      # = -0.080
clearance = [z + chassis_bottom_rel for z in zs]

say("")
say("== 静置结果（预热后 2 s 窗口，%d 帧）==" % len(tail))
say("base_link 高度 z: 首 %.4f → 末 %.4f m（期望≈轮半径 %.3f）" % (zs[0], zs[-1], wheel_radius))
say("底盘底面离地: 首 %.4f → 末 %.4f m（期望≈0.025，PDF 离地间隙 25 mm）" % (clearance[0], clearance[-1]))
say("底盘底面最小离地: %.4f m（<0 表示穿透）" % min(clearance))
say("水平漂移: dx=%.4f m dy=%.4f m" % (xs[-1] - xs[0], ys[-1] - ys[0]))
say("z 抖动峰峰值: %.4f m" % (max(zs) - min(zs)))
say("姿态倾角最大: %.2f°" % max(tilts))

checks = {
    "导入成功": bool(robot_path),
    "四轮旋转关节": len(joint_prims) == 4,
    "车体网格已加载": len(mesh_prims) >= 1 and chassis_ok,
    "跨轮距离≈726mm": abs(across - 0.726) <= 0.005,
    "外接包络≈662mm": abs(size_x - 0.662) <= 0.006 and abs(size_y - 0.662) <= 0.006,
    "DOF=4": art.num_dof == 4,
    "无穿透(底面>=0)": min(clearance) >= -0.001,
    "离地间隙接近25mm": 0.015 <= clearance[-1] <= 0.035,
    "静置高度≈轮半径": abs(zs[-1] - wheel_radius) <= 0.010,
    "无水平漂移(<10mm)": abs(xs[-1] - xs[0]) < 0.010 and abs(ys[-1] - ys[0]) < 0.010,
    "无持续抖动(z峰峰<5mm)": (max(zs) - min(zs)) < 0.005,
    "姿态水平(<2°)": max(tilts) < 2.0,
}
say("")
say("== 验收 ==")
ok = True
for name, passed in checks.items():
    say("  [%s] %s" % ("PASS" if passed else "FAIL", name))
    ok = ok and passed
# ---- 3b. 落体试验：证明底盘是自由体（不是被固定的），且能稳定着地 ----
import numpy as np  # noqa: E402

say("")
say("== 落体试验（抬到 z=0.30 m 自由落体）==")
try:
    art.set_world_poses(
        positions=np.array([[0.0, 0.0, 0.30]]),
        orientations=np.array([[1.0, 0.0, 0.0, 0.0]]),
    )
    art.set_velocities(np.zeros((1, 6)))
    world.step(render=False, update_fabric=True)
    start_z = base_xform.get_world_poses()[0][0].tolist()[2]
    expected_start = 0.30 + wheel_radius
    drop_zs = []
    for _ in range(90):            # 1.5 s
        world.step(render=False, update_fabric=True)
        drop_zs.append(base_xform.get_world_poses()[0][0].tolist()[2])
    end_z = drop_zs[-1]
    say("base_link 起始 z=%.4f（设置 0.30 + 轮半径 %.3f = %.3f）→ 落地 z=%.4f m（期望 %.3f）"
        % (start_z, wheel_radius, expected_start, end_z, wheel_radius))
    say("落体过程最低 z=%.4f m（<0 表示穿透地面）" % min(drop_zs))
    # set_world_poses 作用在铰接根（base_footprint），base_link 在其上 0.105 m
    free_base = abs(start_z - expected_start) < 0.02
    landed = abs(end_z - wheel_radius) <= 0.015
    no_sink = min(drop_zs) >= -0.005
    for name, passed in (("底盘是自由体(可被抬起)", free_base),
                         ("自由落体后回到轮子着地高度", landed),
                         ("落地无穿透", no_sink)):
        say("  [%s] %s" % ("PASS" if passed else "FAIL", name))
        ok = ok and passed
except Exception as exc:  # pragma: no cover
    say("落体试验执行失败: %r" % (exc,))
    ok = False

say("")
say("总体: %s" % ("PASS" if ok else "FAIL"))

timeline.stop()

# ---- 4. 导出 USD 供 GUI 使用 ----
try:
    omni.usd.get_context().new_stage()
    stage2 = omni.usd.get_context().get_stage()
    status2, path2 = omni.kit.commands.execute(
        "URDFParseAndImportFile",
        urdf_path=URDF,
        import_config=cfg,
        dest_path=USD_OUT,
        get_articulation_root=False,
    )
    sim_app.update()
    say("")
    say("USD 导出: status=%s path=%s 大小=%s 字节"
        % (status2, USD_OUT, os.path.getsize(USD_OUT) if os.path.exists(USD_OUT) else "缺失"))
    say("USD 目录清单: %s" % sorted(os.listdir(os.path.dirname(USD_OUT))))

    # 回读校验：确认导出的 USD 能重新打开且结构完整（GUI 使用前的前置检查）
    omni.usd.get_context().open_stage(USD_OUT)
    sim_app.update()
    stage3 = omni.usd.get_context().get_stage()
    link_names = {"base_link", "base_footprint", "wheel_fl_link", "wheel_fr_link",
                  "wheel_rl_link", "wheel_rr_link", "lidar_front_link", "lidar_rear_link", "imu_link"}
    found = {p.GetName() for p in stage3.Traverse()} & link_names
    joint_cnt = sum(1 for p in stage3.Traverse()
                    if p.GetTypeName() in ("PhysicsRevoluteJoint", "PhysicsFixedJoint"))
    art_roots = [p.GetPath().pathString for p in stage3.Traverse()
                 if p.HasAPI("PhysicsArticulationRootAPI")]
    say("USD 回读: 默认 prim=%s, 找到 link %d/9, 关节 %d, 铰接根 %s"
        % (stage3.GetDefaultPrim().GetPath() if stage3.GetDefaultPrim() else None,
           len(found), joint_cnt, art_roots))
    usd_ok = len(found) == 9 and joint_cnt >= 8
    say("  [%s] 导出的 USD 可回读且结构完整" % ("PASS" if usd_ok else "FAIL"))
    ok = ok and usd_ok
except Exception as exc:  # pragma: no cover
    say("USD 导出失败: %r" % (exc,))

with open(REPORT, "w") as f:
    f.write("\n".join(lines) + "\n")
say("")
say("报告写入 %s" % REPORT)
sim_app.close()
sys.exit(0 if ok else 1)
