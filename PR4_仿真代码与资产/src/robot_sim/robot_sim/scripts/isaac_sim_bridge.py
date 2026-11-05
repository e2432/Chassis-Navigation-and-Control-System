#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Isaac Sim ↔ ROS 2 统一桥接（E09/E10 的仿真侧入口）。

一个脚本同时提供 E06/E08/E09 三件事，用开关选择：

  --cmd-vel  订阅 `/cmd_vel` 驱动四轮（physics 用速度驱动；kinematic 用理想运动学推进）
  --sensors  发布 `/lidar/points`（射线投射 3D 雷达）与 `/imu`（IMUSensor，含重力）
  --truth    发布 `/sim/ground_truth`（nav_msgs/Odometry，**世界系**位姿与速度）
  --demo     内置运动剧本（静止→前进→侧移→原地转→复位到原点→静止），用于不需要外部指令的验证

发布节奏：`/clock` 每 tick；`/lidar/points` 10 Hz（仿真时间）；`/imu` 与 `/sim/ground_truth`
每个物理步一次。三者都用 `OnCustomEvent` 边沿触发——直接挂 OnPlaybackTick 会"每帧重发当前值"
（E08 实测会变成 ~84 Hz 且时间戳每帧都变，见 docs/engineering/sensors.md 第 6 节）。

用法：
  # E09：真值 + 传感器 + 内置剧本（配合 robot_bringup/truth_tf.launch.py 起 TF 节点）
  "$ISAAC_PYTHON" src/robot_sim/robot_sim/scripts/isaac_sim_bridge.py --truth --sensors --demo --duration 25

  # E10：由 launch 拉起（见 robot_bringup/simulation.launch.py）
  ros2 launch robot_bringup simulation.launch.py use_sim_time:=true

产出：仿真侧统计写入 --out 目录（默认 results/w3/e09）。
"""

import argparse
import math
import os
import sys
import time

ap = argparse.ArgumentParser()
ap.add_argument("--scene", default="assets/scenes/e05_min_scene.usd")
ap.add_argument("--out", default="results/w3/e09")
ap.add_argument("--duration", type=float, default=25.0,
                help="仿真时间长度（秒）；<=0 表示不限时长（Ctrl-C 才停）")
ap.add_argument("--mode", choices=["physics", "kinematic"], default="physics")
ap.add_argument("--headless", action="store_true", default=True)
ap.add_argument("--gui", action="store_true", help="开窗口（默认 headless）")
ap.add_argument("--cloud-alias", default="",
                help="额外把点云再发一份到该话题（过渡期兼容旧名 /points_raw）")
ap.add_argument("--draw", action="store_true",
                help="在 Isaac 视口里把点云画成绿色点（/World/LidarDebug/points），仅可视化")
ap.add_argument("--cmd-vel", action="store_true", help="订阅 /cmd_vel 驱动")
ap.add_argument("--sensors", action="store_true", help="发布 /lidar/points 与 /imu")
ap.add_argument("--truth", action="store_true", help="发布 /sim/ground_truth")
ap.add_argument("--demo", action="store_true", help="使用内置运动剧本")
ap.add_argument("--lidar-hz", type=float, default=10.0)
ap.add_argument("--h-rays", type=int, default=360)
ap.add_argument("--v-rays", type=int, default=8)
ap.add_argument("--vfov", nargs=2, type=float, default=[-7.0, 52.0])
ap.add_argument("--max-range", type=float, default=40.0)
ap.add_argument("--imu-hz", type=float, default=200.0)
ap.add_argument("--target-rtf", type=float, default=1.0)
args, _unknown = ap.parse_known_args()

if args.gui:
    args.headless = False
else:
    # headless 不需要 X。反过来，带着 DISPLAY 跑 headless 时 Kit 会卡在
    # carb.windowing-glfw.plugin 初始化（本机实测：卡 80 s 无进展），所以这里主动清掉。
    os.environ.pop("DISPLAY", None)

OUT = os.path.abspath(args.out)
os.makedirs(OUT, exist_ok=True)
SUMMARY = os.path.join(OUT, "sim_bridge_summary.txt")
lines = []


def say(msg):
    print("[SIM] %s" % msg, flush=True)
    lines.append(msg)


say("== 桥接参数 ==")
say("scene=%s mode=%s headless=%s draw=%s" % (args.scene, args.mode, args.headless, args.draw))
say("cmd_vel=%s sensors=%s truth=%s demo=%s duration=%.1fs"
    % (args.cmd_vel, args.sensors, args.truth, args.demo, args.duration))

from isaacsim import SimulationApp  # noqa: E402

sim_app = SimulationApp({"headless": args.headless})
import carb.events  # noqa: E402
import numpy as np  # noqa: E402
import omni.graph.core as og  # noqa: E402
import omni.kit.app  # noqa: E402
import omni.physx  # noqa: E402
import omni.timeline  # noqa: E402
import omni.usd  # noqa: E402
from isaacsim.core.prims import Articulation  # noqa: E402
from pxr import Gf, UsdGeom, UsdPhysics  # noqa: E402

app = omni.kit.app.get_app()
mgr = app.get_extension_manager()
for ext in ("isaacsim.ros2.bridge", "isaacsim.sensors.physics"):
    if not mgr.is_extension_enabled(ext):
        mgr.set_extension_enabled_immediate(ext, True)
        sim_app.update()
say("扩展: ros2.bridge=%s sensors.physics=%s"
    % (mgr.is_extension_enabled("isaacsim.ros2.bridge"),
       mgr.is_extension_enabled("isaacsim.sensors.physics")))

scene = args.scene if os.path.isabs(args.scene) else os.path.join(
    os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))), args.scene)
if not os.path.exists(scene):
    # 安装在 share/ 里时的兜底：按工作空间根目录找
    scene = os.path.join(os.getcwd(), args.scene)
omni.usd.get_context().open_stage(os.path.abspath(scene))
for _ in range(6):
    sim_app.update()
stage = omni.usd.get_context().get_stage()
say("场景: %s" % stage.GetRootLayer().identifier)

ROBOT_ROOT = "/World/robot_a/base_footprint"
LIDAR_FRAME = "lidar_front_link"
IMU_FRAME = "imu_link"
LIDAR_XY = 0.13
LIDAR_Z_REL = 0.146 + 0.04 + 0.065 / 2.0      # 0.2185
WHEEL_R = 0.105
WHEEL_C = 0.23546
PHI = {"fl": 45.0, "fr": -45.0, "rl": 135.0, "rr": 225.0}
R_I = {"fl": (WHEEL_C, WHEEL_C), "fr": (WHEEL_C, -WHEEL_C),
       "rl": (-WHEEL_C, WHEEL_C), "rr": (-WHEEL_C, -WHEEL_C)}
U = {k: (math.sin(math.radians(v)), -math.cos(math.radians(v))) for k, v in PHI.items()}

# ---- 重力：physics 要保持正常重力（IMU 才有重力读数）----
scene_prim = stage.GetPrimAtPath("/World/PhysicsScene")
if args.mode == "kinematic" and scene_prim.IsValid():
    UsdPhysics.Scene(scene_prim).GetGravityMagnitudeAttr().Set(0.0)
    say("kinematic 模式：重力置 0（IMU 会退化为 0，只用于位姿/点云验证）")
elif scene_prim.IsValid():
    g = UsdPhysics.Scene(scene_prim).GetGravityMagnitudeAttr().Get()
    if g is None or g <= 1e-6:
        UsdPhysics.Scene(scene_prim).GetGravityMagnitudeAttr().Set(9.81)
    say("physics 模式：重力=%s（负值会让车起飞）"
        % UsdPhysics.Scene(scene_prim).GetGravityMagnitudeAttr().Get())

# ---- IMU ----
imu = None
if args.sensors:
    from isaacsim.sensors.physics import IMUSensor
    imu = IMUSensor(prim_path="/World/robot_a/base_link/ImuBridge", frequency=int(args.imu_hz),
                    translation=np.array([LIDAR_XY, LIDAR_XY, LIDAR_Z_REL]))
    say("IMU: %s freq=%d Hz" % (imu.prim_path, imu.get_frequency()))

# ---- 动作图 ----
GRAPH = "/SimBridge"
nodes = [
    ("Tick", "omni.graph.action.OnPlaybackTick"),
    ("Ctx", "isaacsim.ros2.bridge.ROS2Context"),
    ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
    ("PubClock", "isaacsim.ros2.bridge.ROS2PublishClock"),
    ("CntTick", "omni.graph.action.Counter"),
]
values = [("ReadSimTime.inputs:resetOnStop", False),
          ("PubClock.inputs:topicName", "/clock")]
conns = [("Tick.outputs:tick", "PubClock.inputs:execIn"),
         ("Tick.outputs:tick", "CntTick.inputs:execIn"),
         ("Ctx.outputs:context", "PubClock.inputs:context"),
         ("ReadSimTime.outputs:simulationTime", "PubClock.inputs:timeStamp")]
events = {}
if args.sensors:
    nodes += [("PubCloud", "isaacsim.ros2.bridge.ROS2PublishPointCloud"),
              ("PubImu", "isaacsim.ros2.bridge.ROS2PublishImu"),
              ("EvCloud", "omni.graph.action.OnCustomEvent"),
              ("EvImu", "omni.graph.action.OnCustomEvent")]
    values += [("PubCloud.inputs:topicName", "/lidar/points"),
               ("PubCloud.inputs:frameId", LIDAR_FRAME),
               ("PubCloud.inputs:queueSize", 5),
               ("PubCloud.inputs:qosProfile", '{"history": "keepLast", "depth": 5, "reliability": "bestEffort", "durability": "volatile", "deadline": 0.0, "lifespan": 0.0, "liveliness": "systemDefault", "leaseDuration": 0.0}'),
               ("PubImu.inputs:topicName", "/imu"),
               ("PubImu.inputs:frameId", IMU_FRAME),
               ("PubImu.inputs:queueSize", 50),
               ("PubImu.inputs:qosProfile", '{"history": "keepLast", "depth": 50, "reliability": "bestEffort", "durability": "volatile", "deadline": 0.0, "lifespan": 0.0, "liveliness": "systemDefault", "leaseDuration": 0.0}'),
               ("PubImu.inputs:publishOrientation", True),
               ("PubImu.inputs:publishLinearAcceleration", True),
               ("PubImu.inputs:publishAngularVelocity", True),
               ("EvCloud.inputs:eventName", "sim_pub_cloud"),
               ("EvImu.inputs:eventName", "sim_pub_imu"),
               ("EvCloud.inputs:onlyPlayback", True),
               ("EvImu.inputs:onlyPlayback", True)]
    conns += [("Ctx.outputs:context", "PubCloud.inputs:context"),
              ("Ctx.outputs:context", "PubImu.inputs:context"),
              ("ReadSimTime.outputs:simulationTime", "PubCloud.inputs:timeStamp"),
              ("ReadSimTime.outputs:simulationTime", "PubImu.inputs:timeStamp"),
              ("EvCloud.outputs:execOut", "PubCloud.inputs:execIn"),
              ("EvImu.outputs:execOut", "PubImu.inputs:execIn")]
    events["cloud"] = "omni.graph.action.sim_pub_cloud"
    events["imu"] = "omni.graph.action.sim_pub_imu"
    if args.cloud_alias:
        nodes += [("PubCloudAlias", "isaacsim.ros2.bridge.ROS2PublishPointCloud")]
        values += [("PubCloudAlias.inputs:topicName", args.cloud_alias),
                   ("PubCloudAlias.inputs:frameId", LIDAR_FRAME),
                   ("PubCloudAlias.inputs:queueSize", 5),
               ("PubCloudAlias.inputs:qosProfile", '{"history": "keepLast", "depth": 5, "reliability": "bestEffort", "durability": "volatile", "deadline": 0.0, "lifespan": 0.0, "liveliness": "systemDefault", "leaseDuration": 0.0}')]
        conns += [("Ctx.outputs:context", "PubCloudAlias.inputs:context"),
                  ("ReadSimTime.outputs:simulationTime", "PubCloudAlias.inputs:timeStamp"),
                  ("EvCloud.outputs:execOut", "PubCloudAlias.inputs:execIn")]
        say("点云别名话题（过渡期）：%s" % args.cloud_alias)
if args.truth:
    nodes += [("PubTruth", "isaacsim.ros2.bridge.ROS2PublishOdometry"),
              ("EvTruth", "omni.graph.action.OnCustomEvent")]
    values += [("PubTruth.inputs:topicName", "/sim/ground_truth"),
               ("PubTruth.inputs:odomFrameId", "world"),
               ("PubTruth.inputs:chassisFrameId", "base_footprint"),
               # 关键：世界系速度原样发布，机体系转换在 ROS 侧的真值适配节点做（E09 检查项 3）
               ("PubTruth.inputs:publishRawVelocities", True),
               ("PubTruth.inputs:robotFront", [1.0, 0.0, 0.0]),
               ("EvTruth.inputs:eventName", "sim_pub_truth"),
               ("EvTruth.inputs:onlyPlayback", True)]
    conns += [("Ctx.outputs:context", "PubTruth.inputs:context"),
              ("ReadSimTime.outputs:simulationTime", "PubTruth.inputs:timeStamp"),
              ("EvTruth.outputs:execOut", "PubTruth.inputs:execIn")]
    events["truth"] = "omni.graph.action.sim_pub_truth"
if args.cmd_vel:
    nodes += [("SubCmd", "isaacsim.ros2.bridge.ROS2SubscribeTwist")]
    values += [("SubCmd.inputs:topicName", "/cmd_vel"), ("SubCmd.inputs:queueSize", 10)]
    conns += [("Tick.outputs:tick", "SubCmd.inputs:execIn"),
              ("Ctx.outputs:context", "SubCmd.inputs:context")]

if stage.GetPrimAtPath(GRAPH).IsValid():
    stage.RemovePrim(GRAPH)
og.Controller.edit({"graph_path": GRAPH, "evaluator_name": "execution"},
                   {og.Controller.Keys.CREATE_NODES: nodes,
                    og.Controller.Keys.SET_VALUES: values,
                    og.Controller.Keys.CONNECT: conns})
A = lambda n: og.Controller.attribute("%s/%s" % (GRAPH, n))  # noqa: E731
sim_t_attr = A("ReadSimTime.outputs:simulationTime")
cnt_tick = A("CntTick.outputs:count")
cloud_data = A("PubCloud.inputs:data") if args.sensors else None
cloud_alias_data = (A("PubCloudAlias.inputs:data") if (args.sensors and args.cloud_alias) else None)
imu_lacc = A("PubImu.inputs:linearAcceleration") if args.sensors else None
imu_avel = A("PubImu.inputs:angularVelocity") if args.sensors else None
imu_ori = A("PubImu.inputs:orientation") if args.sensors else None
truth_pos = A("PubTruth.inputs:position") if args.truth else None
truth_ori = A("PubTruth.inputs:orientation") if args.truth else None
truth_lin = A("PubTruth.inputs:linearVelocity") if args.truth else None
truth_ang = A("PubTruth.inputs:angularVelocity") if args.truth else None
cmd_lin = A("SubCmd.outputs:linearVelocity") if args.cmd_vel else None
cmd_ang = A("SubCmd.outputs:angularVelocity") if args.cmd_vel else None
_bus = omni.kit.app.get_app().get_message_bus_event_stream()
EV = {k: carb.events.type_from_string(v) for k, v in events.items()}
say("动作图已建立: %s" % (", ".join(sorted(events.keys())) or "仅 /clock"))


def push(kind):
    _bus.push(EV[kind], payload={})


# ---- 雷达射线 ----
if args.sensors:
    el = np.radians(np.linspace(args.vfov[0], args.vfov[1], args.v_rays))
    az = np.radians(np.arange(args.h_rays) * 360.0 / args.h_rays)
    AZ, EL = np.meshgrid(az, el)
    DIR_LOCAL = np.stack([np.cos(EL) * np.cos(AZ), np.cos(EL) * np.sin(AZ), np.sin(EL)],
                         axis=-1).reshape(-1, 3)
    say("射线/帧: %d" % len(DIR_LOCAL))
    q_int = omni.physx.get_physx_scene_query_interface()

# ---- 点云可视化（Isaac 视口里看得到）----
pts_geom = None
if args.draw and args.sensors:
    stage.DefinePrim("/World/LidarDebug", "Xform")
    prim = stage.DefinePrim("/World/LidarDebug/points", "Points")
    pts_geom = UsdGeom.Points(prim)
    pts_geom.CreateDisplayColorAttr([Gf.Vec3f(0.1, 0.9, 0.2)])
    pts_geom.CreateWidthsAttr([0.03])
    say("视口点云可视化: /World/LidarDebug/points（绿色，随雷达刷新）")

# ---- 驱动 ----
timeline = omni.timeline.get_timeline_interface()
timeline.play()
for _ in range(5):
    sim_app.update()
art = Articulation(prim_paths_expr=ROBOT_ROOT)
art.initialize()
dof = list(art.dof_names or [])
order = [next((i for i, n in enumerate(dof) if n.startswith("base_wheel_%s" % k)), i)
         for i, k in enumerate(R_I)]
if args.mode == "physics":
    for i in range(art.num_dof):
        art.switch_dof_control_mode("velocity", i)
    art.set_gains(kps=np.zeros(art.num_dof), kds=np.full(art.num_dof, 1.0e4))
    art.set_max_efforts(np.full(art.num_dof, 80.0))
say("articulation DOF=%d 关节=%s 驱动=%s" % (art.num_dof, dof, args.mode))


def ik(vx, vy, wz):
    return {k: ((vx - wz * ry) * U[k][0] + (vy + wz * rx) * U[k][1]) / WHEEL_R
            for k, (rx, ry) in R_I.items()}


DEMO = [
    ("still", 0.0, 0.0, 0.0, 3.0),
    ("forward", 0.15, 0.0, 0.0, 3.0),
    ("still", 0.0, 0.0, 0.0, 1.5),
    ("lateral_left", 0.0, 0.15, 0.0, 3.0),
    ("still", 0.0, 0.0, 0.0, 1.5),
    ("rot_ccw_90", 0.0, 0.0, 0.5, math.pi / 2 / 0.5),
    ("still", 0.0, 0.0, 0.0, 1.5),
    ("reset", 0.0, 0.0, 0.0, 0.2),
    ("still", 0.0, 0.0, 0.0, 2.0),
]
say("内置剧本: %s" % [d[0] for d in DEMO])
if args.mode == "physics":
    say("提示：physics 模式受圆柱轮（无辊子）限制，平移只有指令的 30~45% 且会偏约 45 度，"
        "旋转可达 90%；要指令多快就多快、方向也对，请用 --mode kinematic（见 docs/engineering/motion_check.md）")


def apply_motion(vx, vy, wz, dt):
    if args.mode == "physics":
        wheels = ik(vx, vy, wz)
        wv = np.zeros(art.num_dof)
        for i, k in enumerate(R_I):
            wv[order[i]] = wheels[k]
        art.set_joint_velocity_targets(wv.reshape(1, -1))
        return
    # kinematic：积分位姿并写回（理想运动学，不验证接触）
    p = state["pose"]
    c, s = math.cos(p[2]), math.sin(p[2])
    p[0] += (vx * c - vy * s) * dt
    p[1] += (vx * s + vy * c) * dt
    p[2] += wz * dt
    q = [math.cos(p[2] / 2), 0.0, 0.0, math.sin(p[2] / 2)]
    art.set_world_poses(positions=np.array([[p[0], p[1], 0.0]]), orientations=np.array([q]))
    wheels = ik(vx, vy, wz)
    qj = (state["joint_q"] + np.array([wheels[k] for k in R_I]) * dt + math.pi) % (2 * math.pi) - math.pi
    state["joint_q"] = qj
    art.set_joint_positions(np.array([[qj[i] for i in order]]))


state = {"pose": np.zeros(3), "joint_q": np.zeros(4), "prev_t": 0.0,
         "next_cloud": 0.0, "phase": 0, "phase_t0": 0.0, "reset_done": False,
         "n_cloud": 0, "n_imu": 0, "n_truth": 0, "skipped": 0, "rays_ms": [],
         "prev_xy": None, "prev_yaw": None, "v_world": (0.0, 0.0, 0.0)}

T0 = time.time()
UNLIMITED = args.duration <= 0.0
if UNLIMITED:
    say("开始循环（不限时长：Ctrl-C 退出；launch 里用 duration:=0）")
else:
    say("开始循环（仿真时间 %0.1f s 后自动结束并打印统计 —— 这是正常收尾，不是闪退）"
        % args.duration)
while True:
    t_loop = time.time()
    sim_app.update()
    try:
        t = float(sim_t_attr.get())
    except Exception:
        continue
    if t <= 0.0:
        continue
    dt = 0.0 if state["prev_t"] == 0.0 else max(0.0, min(0.2, t - state["prev_t"]))
    state["prev_t"] = t

    if args.cmd_vel:
        try:
            lin = [float(v) for v in cmd_lin.get()]
            ang = [float(v) for v in cmd_ang.get()]
        except Exception:
            lin, ang = [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]
        vx, vy, wz = lin[0], lin[1], ang[2]
    elif args.demo:
        name, vx, vy, wz, hold = DEMO[state["phase"] % len(DEMO)]
        if t - state["phase_t0"] >= hold:
            state["phase"] += 1
            state["phase_t0"] = t
            name, vx, vy, wz, hold = DEMO[state["phase"] % len(DEMO)]
            say("剧本 -> %s（t=%.2f s）" % (name, t))
        if name == "reset" and not state["reset_done"]:
            art.set_world_poses(positions=np.array([[0.0, 0.0, 0.0]]),
                                orientations=np.array([[1.0, 0.0, 0.0, 0.0]]))
            state["pose"][:] = 0.0        # kinematic 模式的积分位姿也要清，否则下一帧又被写回原处
            state["reset_done"] = True
            say("复位到原点（用于验证重置后 TF/里程计一致）")
        vx, vy, wz = (vx, vy, wz) if name != "reset" else (0.0, 0.0, 0.0)
    else:
        vx = vy = wz = 0.0

    apply_motion(vx, vy, wz, dt)

    pos, ori = art.get_world_poses()
    px, py, pz = float(pos[0][0]), float(pos[0][1]), float(pos[0][2])
    qw, qx, qy, qz = [float(v) for v in ori[0]]
    yaw = math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))

    # ---- 真值速度：对真值位姿做差分（世界系）----
    if args.truth and state["prev_xy"] is not None and dt > 1e-6:
        vxw = (px - state["prev_xy"][0]) / dt
        vyw = (py - state["prev_xy"][1]) / dt
        dyaw = math.atan2(math.sin(yaw - state["prev_yaw"]), math.cos(yaw - state["prev_yaw"]))
        state["v_world"] = (vxw, vyw, dyaw / dt)
    state["prev_xy"] = (px, py)
    state["prev_yaw"] = yaw

    # ---- 雷达 ----
    if args.sensors and t >= state["next_cloud"]:
        state["next_cloud"] = t + 1.0 / args.lidar_hz
        t_ray = time.time()
        cz, sz = math.cos(yaw), math.sin(yaw)
        ox = px + LIDAR_XY * cz - LIDAR_XY * sz
        oy = py + LIDAR_XY * sz + LIDAR_XY * cz
        oz = LIDAR_Z_REL + WHEEL_R
        origin = Gf.Vec3d(ox, oy, oz)
        Rw = np.array([[cz, -sz, 0.0], [sz, cz, 0.0], [0.0, 0.0, 1.0]])
        dw = DIR_LOCAL @ Rw.T
        pts = []
        for d in dw:
            hits = []

            def _cb(h, _hits=hits):
                _hits.append(h)
                return True

            q_int.raycast_all(origin, Gf.Vec3d(float(d[0]), float(d[1]), float(d[2])),
                              args.max_range, _cb)
            for h in hits:
                if str(getattr(h, "collision", "")).startswith("/World/robot_a"):
                    continue
                p = getattr(h, "position", None)
                if p is not None:
                    pts.append((p[0], p[1], p[2]))
                break
        state["rays_ms"].append((time.time() - t_ray) * 1e3)
        if pts:
            local = (np.array(pts, dtype=float) - np.array([ox, oy, oz])) @ Rw
            cloud_data.set(local.astype(np.float32).tolist())
            if cloud_alias_data is not None:
                cloud_alias_data.set(local.astype(np.float32).tolist())
            push("cloud")
            state["n_cloud"] += 1
            if pts_geom is not None:
                pts_geom.GetPointsAttr().Set([Gf.Vec3f(float(p[0]), float(p[1]), float(p[2]))
                                              for p in pts])
        else:
            state["skipped"] += 1

    # ---- IMU + 真值：每物理步 ----
    if dt > 0.0:
        if args.sensors and imu is not None:
            fr = imu.get_current_frame(read_gravity=True)
            if fr:
                la, av = fr.get("lin_acc"), fr.get("ang_vel")
                if la is not None and av is not None:
                    imu_lacc.set([float(v) for v in np.asarray(la).reshape(-1)[:3]])
                    imu_avel.set([float(v) for v in np.asarray(av).reshape(-1)[:3]])
                    o = fr.get("orientation")
                    if o is not None:
                        o = [float(v) for v in np.asarray(o).reshape(-1)[:4]]
                        imu_ori.set([o[1], o[2], o[3], o[0]])
                    push("imu")
                    state["n_imu"] += 1
        if args.truth:
            truth_pos.set([px, py, pz])
            truth_ori.set([qx, qy, qz, qw])       # IJKR
            vxw, vyw, wz_w = state["v_world"]
            truth_lin.set([vxw, vyw, 0.0])
            truth_ang.set([0.0, 0.0, wz_w])
            push("truth")
            state["n_truth"] += 1

    if args.target_rtf > 0:
        spent = time.time() - t_loop
        target = 1.0 / (60.0 * args.target_rtf)
        if target > spent:
            time.sleep(min(0.03, target - spent))
    if not UNLIMITED and t >= args.duration:
        break

say("")
say("== 统计 ==")
say("点云 %d 帧（空帧 %d）、IMU %d 条、真值 %d 条；最终位姿 x=%.3f y=%.3f yaw=%.1f°"
    % (state["n_cloud"], state["skipped"], state["n_imu"], state["n_truth"],
       state["prev_xy"][0] if state["prev_xy"] else 0.0,
       state["prev_xy"][1] if state["prev_xy"] else 0.0, math.degrees(state["prev_yaw"] or 0.0)))
if state["rays_ms"]:
    say("单帧射线耗时: 均值 %.1f ms / 最大 %.1f ms" % (float(np.mean(state["rays_ms"])),
                                             float(np.max(state["rays_ms"]))))
timeline.stop()
open(SUMMARY, "w").write("\n".join(lines) + "\n")
say("正常收尾：关闭 Isaac（日志会出现 Simulation App Shutting Down / process has finished cleanly，"
    "这不是闪退；要一直跑就用 duration:=0）")
sim_app.close()
