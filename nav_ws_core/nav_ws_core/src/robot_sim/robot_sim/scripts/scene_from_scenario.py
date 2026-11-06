"""Preview the frozen contract table; pedestrians use the shared contract formula."""
from __future__ import annotations
import argparse
import math
import os
from pathlib import Path
import sys

def core_root():
    candidates = [Path(os.environ['NAV_WS'])] if os.environ.get('NAV_WS') else []
    candidates += [Path(__file__).resolve().parent, Path.cwd()]
    for base in candidates:
        for parent in (base, *base.parents):
            if (parent / 'assets/robot_a_chassis_agl_chs03_f01.usd').is_file():
                return parent
    raise FileNotFoundError('找不到 nav_ws_core/assets；请设置 NAV_WS 或从 nav_ws_core 运行')
ROOT = core_root()
sys.path.insert(0, str(ROOT / 'src/robot_benchmark'))
from robot_benchmark.scenario_gen import SCENARIO_KEYS, default_manifest_path, find_instance, pedestrian_position, transform_pose

def set_translate(prim, xyz):
    from pxr import Gf, UsdGeom
    xf = UsdGeom.Xformable(prim)
    for op in xf.GetOrderedXformOps():
        if op.GetOpName() == 'xformOp:translate':
            op.Set(Gf.Vec3d(*xyz))
            return
    xf.AddTranslateOp().Set(Gf.Vec3d(*xyz))

def add_cylinder(stage, path, xyz, radius, height, color, collision=True):
    from pxr import Gf, UsdGeom, UsdPhysics
    prim = UsdGeom.Cylinder.Define(stage, path)
    prim.CreateRadiusAttr(radius)
    prim.CreateHeightAttr(height)
    prim.CreateAxisAttr('Z')
    set_translate(prim, xyz)
    prim.CreateDisplayColorAttr([Gf.Vec3f(*color)])
    if collision:
        UsdPhysics.CollisionAPI.Apply(prim.GetPrim())
    return prim.GetPrim()

def build_scene(stage, inst, world_pose, robot_usd):
    from pxr import Gf, Sdf, UsdGeom, UsdLux, UsdPhysics
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    world = UsdGeom.Xform.Define(stage, '/World')
    stage.SetDefaultPrim(world.GetPrim())
    physics = UsdPhysics.Scene.Define(stage, '/World/PhysicsScene')
    physics.CreateGravityMagnitudeAttr(0.0)
    dome = UsdLux.DomeLight.Define(stage, '/World/Dome')
    dome.CreateIntensityAttr(700.0)
    frame = UsdGeom.Xform.Define(stage, '/World/ScenarioFrame')
    set_translate(frame, (world_pose['x'], world_pose['y'], 0.0))
    frame.AddRotateZOp().Set(math.degrees(world_pose['yaw']))
    base = '/World/ScenarioFrame'
    floor = UsdGeom.Cube.Define(stage, base + '/Ground')
    floor.CreateSizeAttr(1.0)
    set_translate(floor, (inst['goal']['x'] / 2, 0.0, -0.03))
    UsdGeom.Xformable(floor).AddScaleOp().Set(Gf.Vec3f(inst['path_length_m'] + 6.0, 20.0, 0.05))
    floor.CreateDisplayColorAttr([Gf.Vec3f(0.35, 0.38, 0.4)])
    UsdPhysics.CollisionAPI.Apply(floor.GetPrim())
    robot = UsdGeom.Xform.Define(stage, '/World/robot_a')
    robot.GetPrim().GetReferences().AddReference(str(robot_usd))
    set_translate(robot, (world_pose['x'], world_pose['y'], 0.0))
    robot.AddRotateZOp().Set(math.degrees(world_pose['yaw']))
    for prim in stage.Traverse():
        if str(prim.GetPath()).startswith('/World/robot_a') and prim.HasAPI(UsdPhysics.RigidBodyAPI):
            UsdPhysics.RigidBodyAPI(prim).CreateKinematicEnabledAttr(True)
    for (i, o) in enumerate(inst['obstacles']):
        add_cylinder(stage, base + f'/Obstacle_{i:02d}', (o['x'], o['y'], 0.3), o['r'], 0.6, (0.55, 0.42, 0.3))
    for (i, w) in enumerate(inst['walls']):
        cube = UsdGeom.Cube.Define(stage, base + f'/Wall_{i:02d}')
        cube.CreateSizeAttr(1.0)
        set_translate(cube, ((w['x0'] + w['x1']) / 2, (w['y0'] + w['y1']) / 2, 0.5))
        UsdGeom.Xformable(cube).AddScaleOp().Set(Gf.Vec3f(w['x1'] - w['x0'], w['y1'] - w['y0'], 1.0))
        cube.CreateDisplayColorAttr([Gf.Vec3f(0.7, 0.7, 0.66)])
        UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
    peds = []
    for (i, p) in enumerate(inst['pedestrians']):
        (x, y) = pedestrian_position(p, 0.0)
        prim = add_cylinder(stage, base + f'/Pedestrian_{i:02d}', (x, y, 0.85), p['r'], 1.7, (0.8, 0.25, 0.25))
        peds.append({'prim': prim, 'spec': p})
    add_cylinder(stage, base + '/Start', (0.0, 0.0, 0.015), 0.25, 0.02, (0.2, 0.5, 0.9), False)
    add_cylinder(stage, base + '/Goal', (inst['goal']['x'], inst['goal']['y'], 0.015), 0.25, 0.02, (0.2, 0.8, 0.3), False)
    stage.GetRootLayer().customLayerData = {'scenario': inst['scenario'], 'seed': inst['seed'], 'generator_version': '2.0.0'}
    return {'stage': stage, 'inst': inst, 'peds': peds, 'world_pose': world_pose}

def animate(scene, t):
    for ped in scene['peds']:
        (x, y) = pedestrian_position(ped['spec'], t)
        set_translate(ped['prim'], (x, y, 0.85))

def export_scene(scene, path, robot_usd):
    from pxr import Sdf
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    layer = Sdf.Layer.CreateAnonymous()
    layer.TransferContent(scene['stage'].GetRootLayer())
    layer.UpdateExternalReference(str(robot_usd), os.path.relpath(robot_usd, path.parent).replace(os.sep, '/'))
    layer.Export(str(path))

def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument('--manifest', type=Path, default=default_manifest_path())
    parser.add_argument('--scenario', choices=SCENARIO_KEYS, default='static_avoid')
    parser.add_argument('--run', type=int, choices=range(1, 21), default=1)
    parser.add_argument('--all', action='store_true')
    parser.add_argument('--cycle', type=float, default=10.0)
    parser.add_argument('--duration', type=float, default=0.0)
    parser.add_argument('--save', action='store_true')
    parser.add_argument('--out-dir', type=Path, default=ROOT / 'assets/scenes/scenarios')
    parser.add_argument('--headless', action='store_true')
    parser.add_argument('--gui', action='store_true')
    parser.add_argument('--path-frame', action='store_true', help='忽略世界摆放，仅查看契约 path frame')
    args = parser.parse_args(argv)
    if args.headless and (not args.gui):
        os.environ.pop('DISPLAY', None)
    import yaml
    document = yaml.safe_load(args.manifest.read_text(encoding='utf-8'))
    from isaacsim import SimulationApp
    app = SimulationApp({'headless': args.headless and (not args.gui)})
    import omni.timeline
    import omni.usd
    timeline = omni.timeline.get_timeline_interface()
    keys = SCENARIO_KEYS if args.all else (args.scenario,)
    robot_usd = ROOT / 'assets/robot_a_chassis_agl_chs03_f01.usd'
    idx = 0

    def load(index):
        timeline.stop()
        omni.usd.get_context().new_stage()
        inst = find_instance(document, keys[index], args.run)
        pose = {'x': 0.0, 'y': 0.0, 'yaw': 0.0} if args.path_frame else document.get('world_poses', {}).get(str(inst['seed']), {'x': 0.0, 'y': 0.0, 'yaw': 0.0})
        scene = build_scene(omni.usd.get_context().get_stage(), inst, pose, robot_usd)
        if args.save:
            export_scene(scene, args.out_dir / f"{inst['scenario']}_{inst['run']:02d}.usd", robot_usd)
        try:
            from omni.kit.viewport.utility import get_active_viewport, set_camera_view
            mid = transform_pose(inst['goal']['x'] / 2, 0.0, 0.0, pose)
            distance = max(8.0, inst['path_length_m'])
            set_camera_view(eye=(mid[0] - 0.4 * distance, mid[1] - 0.8 * distance, distance), target=(mid[0], mid[1], 0.0), camera=get_active_viewport().camera_path)
        except Exception:
            pass
        print(f"{inst['scenario']} run={inst['run']} seed={inst['seed']} pose={pose}", flush=True)
        timeline.play()
        app.update()
        return (scene, float(timeline.get_current_time()))
    try:
        (scene, t0) = load(idx)
        while app.is_running():
            app.update()
            t = max(0.0, float(timeline.get_current_time()) - t0)
            animate(scene, t)
            if args.all and args.cycle > 0 and (t >= args.cycle):
                if idx + 1 < len(keys):
                    idx += 1
                    (scene, t0) = load(idx)
                elif args.save:
                    break
            if args.duration > 0 and t >= args.duration:
                break
    finally:
        app.close()
    return 0
if __name__ == '__main__':
    raise SystemExit(main())
