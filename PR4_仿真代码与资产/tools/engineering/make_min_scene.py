#!/usr/bin/env python3
"""生成 E05 最小场景：底盘 + 地面 + 一面墙 + 一个箱体 + 灯光。

产物：assets/scenes/e05_min_scene.usd（用**相对引用**指向 ../robot_a_chassis_agl_chs03_f01.usd，
所以整个 assets/ 目录可以整体拷贝，不会丢引用）。

用法（模拟侧 Python 3.11，需能 import isaacsim）：
  ./tools/engineering/e01_run_sim_startup.sh results/w2/e05 tools/engineering/make_min_scene.py
生成后可用 GUI 打开该 usd 并直接按 Play。
"""

import os
import sys

from isaacsim import SimulationApp

sim_app = SimulationApp({"headless": True})

import omni.usd  # noqa: E402
from pxr import Gf, Sdf, Usd, UsdGeom, UsdLux, UsdPhysics  # noqa: E402

OUT = os.path.abspath("assets/scenes/e05_min_scene.usd")
ROBOT_REF = "../robot_a_chassis_agl_chs03_f01.usd"   # 相对 assets/scenes/ 解析
os.makedirs(os.path.dirname(OUT), exist_ok=True)


def cube_collider(stage, path, translate, scale, color=None):
    """带碰撞的长方体（墙 / 箱体）。"""
    cube = UsdGeom.Cube.Define(stage, path)
    cube.CreateSizeAttr(1.0)
    xf = UsdGeom.Xformable(cube.GetPrim())
    xf.AddTranslateOp().Set(Gf.Vec3d(*translate))
    xf.AddScaleOp().Set(Gf.Vec3f(*scale))
    if color:
        cube.CreateDisplayColorAttr([Gf.Vec3f(*color)])
    UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
    UsdPhysics.RigidBodyAPI.Apply(cube.GetPrim())      # 墙/箱体参与物理（箱体可被推动）
    mass = UsdPhysics.MassAPI.Apply(cube.GetPrim())
    mass.CreateMassAttr(20.0)
    return cube


stage = Usd.Stage.CreateNew(OUT)
UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
UsdGeom.SetStageMetersPerUnit(stage, 1.0)

world = UsdGeom.Xform.Define(stage, "/World")
stage.SetDefaultPrim(world.GetPrim())

scene = UsdPhysics.Scene.Define(stage, "/World/PhysicsScene")
scene.CreateGravityDirectionAttr(Gf.Vec3f(0, 0, -1))
scene.CreateGravityMagnitudeAttr(9.81)

# 地面（50 m × 50 m）
plane = UsdGeom.Plane.Define(stage, "/World/GroundPlane")
plane.CreateAxisAttr("Z")
plane.CreateExtentAttr([Gf.Vec3f(-25, -25, 0), Gf.Vec3f(25, 25, 0)])
plane.CreateDisplayColorAttr([Gf.Vec3f(0.55, 0.55, 0.58)])
UsdPhysics.CollisionAPI.Apply(plane.GetPrim())

# 底盘（相对引用）
robot = UsdGeom.Xform.Define(stage, "/World/robot_a")
robot.GetPrim().GetReferences().AddReference(ROBOT_REF)

# 一面墙：位于 +x 3 m 处，宽 4 m、高 1.2 m、厚 0.1 m
cube_collider(stage, "/World/Wall", (3.0, 0.0, 0.6), (0.1, 4.0, 1.2), (0.75, 0.72, 0.68))
# 一个箱体：0.4 m 立方，放在前方 1.5 m 处（E05 要求的“单个箱体”）
cube_collider(stage, "/World/Box", (1.5, 0.0, 0.2), (0.4, 0.4, 0.4), (0.6, 0.45, 0.3))

# 光照
sun = UsdLux.DistantLight.Define(stage, "/World/Sun")
sun.CreateIntensityAttr(900.0)
sun.CreateAngleAttr(0.5)
UsdGeom.Xformable(sun.GetPrim()).AddRotateXYZOp().Set(Gf.Vec3f(35.0, 0.0, 45.0))
dome = UsdLux.DomeLight.Define(stage, "/World/Dome")
dome.CreateIntensityAttr(180.0)

stage.Save()
print("已写出 %s（%d 字节）" % (OUT, os.path.getsize(OUT)), flush=True)

# ---- 回读校验：确认场景能打开、引用生效、铰接根存在 ----
sim_app.update()
omni.usd.get_context().open_stage(OUT)
for _ in range(3):
    sim_app.update()
st = omni.usd.get_context().get_stage()
paths = [p.GetPath().pathString for p in st.Traverse()]
arts = [p.GetPath().pathString for p in st.Traverse() if p.HasAPI("PhysicsArticulationRootAPI")]
joints = [p for p in st.Traverse() if p.GetTypeName() in ("PhysicsRevoluteJoint", "PhysicsFixedJoint")]
links = {p.GetName() for p in st.Traverse()}
want = {"base_link", "wheel_fl_link", "wheel_fr_link", "wheel_rl_link", "wheel_rr_link",
        "lidar_front_link", "lidar_rear_link", "imu_link", "mount_plate_link"}
found = links & want
print("回读：prim 总数 %d" % len(paths), flush=True)
print("  /World 下的对象: %s" % sorted({p.split('/')[2] for p in paths if p.startswith('/World/') and p.count('/') >= 2}), flush=True)
print("  铰接根: %s" % arts, flush=True)
print("  关节数: %d" % len(joints), flush=True)
print("  底盘 link 命中 %d/%d: %s" % (len(found), len(want), sorted(found)), flush=True)
ok = len(found) == len(want) and len(arts) == 1 and len(joints) >= 9
print("校验: %s" % ("PASS" if ok else "FAIL"), flush=True)
sim_app.close()
sys.exit(0 if ok else 1)
