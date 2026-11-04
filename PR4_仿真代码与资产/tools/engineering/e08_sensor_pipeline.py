#!/usr/bin/env python3
"""E08：激光雷达 + IMU 接入（仿真侧）。

做三件事：
  1) 3D 雷达：用 PhysX 射线投射构造一个与 Mid-360 同 FOV/量程/频率的**通用 3D 雷达模型**
     （官方 18 个 RTX 雷达配置里没有 Livox Mid-360，且本机未安装 Isaac 传感器 USD 资产包），
     发布 /lidar/points（PointCloud2，frame_id=lidar_front_link）；
  2) IMU：用 isaacsim.sensors.physics 的 IMUSensor（挂在 base_link，与雷达同位置，符合
     “IMU 集成在雷达内”的规格），发布 /imu（含重力，属正常）；
  3) 按剧本让底盘原地转 / 前进，验证点云随运动正确变化，并记录发布频率与丢帧。

产出：<out>/sensors_log.csv、<out>/sensor_summary.txt
用法：./tools/engineering/e01_run_sim_startup.sh results/w2/e08 tools/engineering/e08_sensor_pipeline.py --out results/w2/e08
"""

import argparse
import csv
import math
import os
import sys
import time

ap = argparse.ArgumentParser()
ap.add_argument("--out", required=True)
ap.add_argument("--scene", default="assets/scenes/e05_min_scene.usd")
ap.add_argument("--duration", type=float, default=30.0)
ap.add_argument("--config", default="src/robot_sim/config/sensors.yaml",
                help="传感器参数配置；命令行显式给出的参数优先")
ap.add_argument("--lidar-hz", type=float, default=None)
ap.add_argument("--imu-hz", type=float, default=None)
ap.add_argument("--h-rays", type=int, default=None)
ap.add_argument("--v-rays", type=int, default=None)
ap.add_argument("--vfov", nargs=2, type=float, default=None)
ap.add_argument("--max-range", type=float, default=None)
ap.add_argument("--target-rtf", type=float, default=1.0)
ap.add_argument("--mode", choices=["physics", "kinematic"], default="physics",
                help="physics=底盘真在物理里（IMU 有真实动态）；kinematic=瞬移推进（IMU 退化为 0）")
args, _ = ap.parse_known_args()

OUT = os.path.abspath(args.out)
os.makedirs(OUT, exist_ok=True)
CSV = os.path.join(OUT, "sensors_log.csv")
SUMMARY = os.path.join(OUT, "sensor_summary.txt")
lines = []


def say(m):
    print("[E08] %s" % m, flush=True)
    lines.append(m)


# 底盘几何（与 URDF 一致）
WHEEL_R = 0.105
LIDAR_XY = 0.13                      # 前雷达对角位置 |x|=|y|
BODY_TOP_Z_REL = 0.146               # base_link 之上的车体顶面
LIDAR_MAST, LIDAR_SIZE = 0.04, 0.065
LIDAR_Z_REL = BODY_TOP_Z_REL + LIDAR_MAST + LIDAR_SIZE / 2.0    # 0.2185 → 世界 z=0.3235
ROBOT_ROOT = "/World/robot_a/base_footprint"
LIDAR_FRAME = "lidar_front_link"

# ---- 读传感器配置（CLI > 配置文件 > 代码内默认值）----
LIDAR_TOPIC, IMU_TOPIC, IMU_FRAME = "/lidar/points", "/imu", "imu_link"
CFG = {}
if args.config and os.path.exists(args.config):
    import yaml
    CFG = yaml.safe_load(open(args.config))
    lid_sim = CFG["sensors"]["lidar"]["sim"]
    lid_ros = CFG["sensors"]["lidar"]["ros"]
    imu_cfg = CFG["sensors"]["imu"]
    LIDAR_TOPIC = lid_ros["topic"]
    LIDAR_FRAME = lid_ros["frame_id"]
    IMU_TOPIC = imu_cfg["ros"]["topic"]
    IMU_FRAME = imu_cfg["ros"]["frame_id"]
    args.lidar_hz = args.lidar_hz if args.lidar_hz is not None else float(lid_ros["publish_rate_hz"]["nominal"])
    args.h_rays = args.h_rays or int(lid_sim["h_rays"])
    args.v_rays = args.v_rays or int(lid_sim["v_rays"])
    args.vfov = args.vfov or [float(v) for v in CFG["sensors"]["lidar"]["spec"]["vfov_deg"]]
    args.max_range = args.max_range if args.max_range is not None else float(lid_sim["max_range_m"])
    args.imu_hz = args.imu_hz if args.imu_hz is not None else float(imu_cfg["sim"]["sensor_frequency_hz"])
else:
    say("警告：未找到传感器配置 %s，使用代码内默认值" % args.config)
    args.lidar_hz = args.lidar_hz or 10.0
    args.v_rays = args.v_rays or 8
    args.h_rays = args.h_rays or 360
    args.vfov = args.vfov or [-7.0, 52.0]
    args.max_range = args.max_range or 40.0
    args.imu_hz = args.imu_hz or 200.0

say("== 参数 ==")
say("配置: %s" % (args.config if CFG else "<内置默认>"))
say("雷达: %d×%d 射线, vfov=%s°, max_range=%.1f m, %.0f Hz" % (args.h_rays, args.v_rays, args.vfov, args.max_range, args.lidar_hz))
say("IMU : %.0f Hz（isaacsim.sensors.physics，含重力；ROS 发布按物理步，实测约 60 Hz）" % args.imu_hz)

from isaacsim import SimulationApp  # noqa: E402

sim_app = SimulationApp({"headless": True})
import carb.events  # noqa: E402
import numpy as np  # noqa: E402
import omni.graph.core as og  # noqa: E402
import omni.kit.app  # noqa: E402
import omni.physx  # noqa: E402
import omni.timeline  # noqa: E402
import omni.usd  # noqa: E402
from isaacsim.core.prims import Articulation  # noqa: E402
from pxr import Gf, UsdPhysics  # noqa: E402

app = omni.kit.app.get_app()
mgr = app.get_extension_manager()
for ext in ("isaacsim.ros2.bridge", "isaacsim.sensors.physics"):
    if not mgr.is_extension_enabled(ext):
        mgr.set_extension_enabled_immediate(ext, True)
        sim_app.update()
    say("扩展 %s enabled=%s" % (ext, mgr.is_extension_enabled(ext)))

omni.usd.get_context().open_stage(os.path.abspath(args.scene))
for _ in range(5):
    sim_app.update()
stage = omni.usd.get_context().get_stage()
say("场景: %s" % stage.GetRootLayer().identifier)

# kinematic 运动学：关重力（与 E06 一致）
if args.mode == "kinematic":
    scene_prim = stage.GetPrimAtPath("/World/PhysicsScene")
    if scene_prim.IsValid():
        UsdPhysics.Scene(scene_prim).GetGravityMagnitudeAttr().Set(0.0)
        say("kinematic 模式：已关闭重力（不做轮地支撑；IMU 将退化为 0，只用于点云验证）")
else:
    say("physics 模式：保留重力并驱动轮子（IMU 有真实动态；平移只到指令的 ~30%，见 E06）")

# ---- IMU ----
from isaacsim.sensors.physics import IMUSensor  # noqa: E402

imu = IMUSensor(prim_path="/World/robot_a/base_link/ImuE08", frequency=int(args.imu_hz),
                translation=np.array([LIDAR_XY, LIDAR_XY, LIDAR_Z_REL]))
say("IMU 已创建: %s (freq=%d Hz)" % (imu.prim_path, imu.get_frequency()))

# ---- Action Graph ----
GRAPH = "/E08Sensors"
sim_t_attr, cloud_attrs, imu_attrs = None, {}, {}
og.Controller.edit(
    {"graph_path": GRAPH, "evaluator_name": "execution"},
    {
        og.Controller.Keys.CREATE_NODES: [
            ("Tick", "omni.graph.action.OnPlaybackTick"),
            ("Ctx", "isaacsim.ros2.bridge.ROS2Context"),
            ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
            ("PubClock", "isaacsim.ros2.bridge.ROS2PublishClock"),
            ("PubCloud", "isaacsim.ros2.bridge.ROS2PublishPointCloud"),
            ("PubImu", "isaacsim.ros2.bridge.ROS2PublishImu"),
            # 发布节点若直接挂 OnPlaybackTick，会每帧把"当前值"再发一次（实测消息数 =
            # tick 数，约为物理步的 1.4 倍）。改用 OnCustomEvent：Python 侧每要发一帧
            # 就推一个 carb 事件，边沿触发 → 一次事件恰好一次发布（已用 Counter 验证 1:1）。
            ("EvCloud", "omni.graph.action.OnCustomEvent"),
            ("EvImu", "omni.graph.action.OnCustomEvent"),
            ("CntTick", "omni.graph.action.Counter"),
        ],
        og.Controller.Keys.SET_VALUES: [
            ("ReadSimTime.inputs:resetOnStop", False),
            ("PubClock.inputs:topicName", "/clock"),
            ("PubCloud.inputs:topicName", LIDAR_TOPIC),
            ("PubCloud.inputs:frameId", LIDAR_FRAME),
            ("PubCloud.inputs:queueSize", 5),
               ("PubCloud.inputs:qosProfile", '{"history": "keepLast", "depth": 5, "reliability": "bestEffort", "durability": "volatile", "deadline": 0.0, "lifespan": 0.0, "liveliness": "systemDefault", "leaseDuration": 0.0}'),
            ("PubImu.inputs:topicName", IMU_TOPIC),
            ("PubImu.inputs:frameId", IMU_FRAME),
            ("PubImu.inputs:queueSize", 50),
               ("PubImu.inputs:qosProfile", '{"history": "keepLast", "depth": 50, "reliability": "bestEffort", "durability": "volatile", "deadline": 0.0, "lifespan": 0.0, "liveliness": "systemDefault", "leaseDuration": 0.0}'),
            ("PubImu.inputs:publishOrientation", True),
            ("PubImu.inputs:publishLinearAcceleration", True),
            ("PubImu.inputs:publishAngularVelocity", True),
            ("EvCloud.inputs:eventName", "e08_pub_cloud"),
            ("EvImu.inputs:eventName", "e08_pub_imu"),
            ("EvCloud.inputs:onlyPlayback", True),
            ("EvImu.inputs:onlyPlayback", True),
        ],
        og.Controller.Keys.CONNECT: [
            ("Tick.outputs:tick", "PubClock.inputs:execIn"),
            ("Ctx.outputs:context", "PubClock.inputs:context"),
            ("Ctx.outputs:context", "PubCloud.inputs:context"),
            ("Ctx.outputs:context", "PubImu.inputs:context"),
            ("ReadSimTime.outputs:simulationTime", "PubClock.inputs:timeStamp"),
            ("Tick.outputs:tick", "CntTick.inputs:execIn"),
            ("EvCloud.outputs:execOut", "PubCloud.inputs:execIn"),
            ("EvImu.outputs:execOut", "PubImu.inputs:execIn"),
            ("ReadSimTime.outputs:simulationTime", "PubCloud.inputs:timeStamp"),
            ("ReadSimTime.outputs:simulationTime", "PubImu.inputs:timeStamp"),
        ],
    },
)
A = lambda n: og.Controller.attribute("%s/%s" % (GRAPH, n))  # noqa: E731
sim_t_attr = A("ReadSimTime.outputs:simulationTime")
cloud_data = A("PubCloud.inputs:data")
imu_lacc, imu_avel, imu_ori = (A("PubImu.inputs:linearAcceleration"),
                               A("PubImu.inputs:angularVelocity"),
                               A("PubImu.inputs:orientation"))
cnt_tick = A("CntTick.outputs:count")
_bus = omni.kit.app.get_app().get_message_bus_event_stream()
EV_CLOUD = carb.events.type_from_string("omni.graph.action.e08_pub_cloud")
EV_IMU = carb.events.type_from_string("omni.graph.action.e08_pub_imu")


def _push(ev):
    """推一个事件让发布节点边沿触发一次（一次事件=一次发布，无重复）。"""
    _bus.push(ev, payload={})

say("动作图已建立: %s, %s, /clock" % (LIDAR_TOPIC, IMU_TOPIC))

# ---- 雷达射线方向（机体/雷达坐标系内固定） ----
el = np.radians(np.linspace(args.vfov[0], args.vfov[1], args.v_rays))
az = np.radians(np.arange(args.h_rays) * 360.0 / args.h_rays)
AZ, EL = np.meshgrid(az, el)                        # (V, H)
DIR_LOCAL = np.stack([np.cos(EL) * np.cos(AZ), np.cos(EL) * np.sin(AZ), np.sin(EL)], axis=-1).reshape(-1, 3)
say("射线总数/帧: %d" % len(DIR_LOCAL))

# ---- 运动/驱动（复用 E06 的运动学模型） ----
timeline = omni.timeline.get_timeline_interface()
timeline.play()
for _ in range(5):
    sim_app.update()
art = Articulation(prim_paths_expr=ROBOT_ROOT)
art.initialize()
say("articulation DOF=%d" % art.num_dof)
if args.mode == "physics":
    for i in range(art.num_dof):
        art.switch_dof_control_mode("velocity", i)
    art.set_gains(kps=np.zeros(art.num_dof), kds=np.full(art.num_dof, 1.0e4))
    art.set_max_efforts(np.full(art.num_dof, 80.0))
    say("已配置轮子速度驱动（kps=0, kds=1e4, maxEffort=80）")

phi = {"fl": 45.0, "fr": -45.0, "rl": 135.0, "rr": 225.0}
U = {k: (math.sin(math.radians(v)), -math.cos(math.radians(v))) for k, v in phi.items()}
C = 0.23546                       # 轮心 |x|=|y|，与 URDF 一致
R_I = {"fl": (C, C), "fr": (C, -C), "rl": (-C, C), "rr": (-C, -C)}
dof = list(art.dof_names or [])
order = [next((i for i, n in enumerate(dof) if n.startswith("base_wheel_%s" % k)), i) for i, k in enumerate(R_I)]


def ik(vx, vy, wz):
    return {k: ((vx - wz * ry) * U[k][0] + (vy + wz * rx) * U[k][1]) / WHEEL_R
            for k, (rx, ry) in R_I.items()}


# 剧本：静止 → 原地逆时针 90° → 前进 → 原地顺时针 90° → 侧移
PHASES = [
    ("still", 0.0, 0.0, 0.0, 3.0),
    ("rot_ccw_90", 0.0, 0.0, 0.5, math.pi / 2 / 0.5),
    ("forward", 0.15, 0.0, 0.0, 3.0),
    ("rot_cw_90", 0.0, 0.0, -0.5, math.pi / 2 / 0.5),
    ("lateral_left", 0.0, 0.15, 0.0, 3.0),
    ("still2", 0.0, 0.0, 0.0, 2.0),
]
say("运动剧本: %s" % [p[0] for p in PHASES])

f = open(CSV, "w", newline="")
w = csv.writer(f)
w.writerow(["sim_t", "phase", "cmd_vx", "cmd_vy", "cmd_wz", "x", "y", "yaw",
            "cloud_pub", "cloud_points", "cloud_r_min", "cloud_r_max",
            "wall_in_body_x", "wall_in_body_y",
            "imu_ax", "imu_ay", "imu_az", "imu_wx", "imu_wy", "imu_wz",
            "lidar_frame_ms", "imu_pub", "cloud_skipped", "tick_count"])

q_int = omni.physx.get_physx_scene_query_interface()
T0 = time.time()
pose = np.zeros(3)
joint_q = np.zeros(4)
prev_t = 0.0
next_cloud, next_imu = 0.0, 0.0
n_cloud = n_imu = 0
skipped = 0
last_stats = {}
frame_times = []


def lidar_rays(px, py, yaw):
    """在世界系发射线，返回雷达坐标系内的命中点 + 是否命中墙体/箱体（用于方向核对）。"""
    cz, sz = math.cos(yaw), math.sin(yaw)
    origin = Gf.Vec3d(px + LIDAR_XY * cz - LIDAR_XY * sz,
                      py + LIDAR_XY * sz + LIDAR_XY * cz,
                      LIDAR_Z_REL + WHEEL_R)
    # 雷达坐标系 -> 世界
    Rw = np.array([[cz, -sz, 0.0], [sz, cz, 0.0], [0.0, 0.0, 1.0]])
    dw = DIR_LOCAL @ Rw.T
    pts, wall = [], None
    for i in range(len(dw)):
        d = dw[i]
        direction = Gf.Vec3d(float(d[0]), float(d[1]), float(d[2]))
        hits = []

        def _cb(h, _hits=hits):
            _hits.append(h)
            return True                    # True = 继续找更远的命中

        q_int.raycast_all(origin, direction, args.max_range, _cb)
        hit, pos, path = None, None, ""
        for h in hits:
            try:
                path = str(h.collision)
                pos = h.position
            except AttributeError:
                path = str(getattr(h, "collision", ""))
                pos = getattr(h, "position", None)
            if path.startswith("/World/robot_a"):
                continue                   # 排除机器人自身（−7° 下俯会先打到车体顶面）
            hit = h
            break
        if hit is None or pos is None:
            continue
        p = np.array([pos[0], pos[1], pos[2]], dtype=float)
        pts.append(p)
        if "Wall" in path:
            if wall is None or np.linalg.norm(p - np.array([origin[0], origin[1], origin[2]])) < \
                    np.linalg.norm(wall - np.array([origin[0], origin[1], origin[2]])):
                wall = p                   # 取最近的墙体命中点（方位角应与 -yaw 一致）
    if not pts:
        return None, None, None
    P = np.array(pts)
    local = (P - np.array([origin[0], origin[1], origin[2]])) @ Rw   # 世界->雷达
    r = np.linalg.norm(local, axis=1)
    return local.astype(np.float32), r, wall


def apply_motion(vx, vy, wz, dt):
    global joint_q, pose
    if args.mode == "physics":
        wheels = ik(vx, vy, wz)
        wv = np.zeros(art.num_dof)
        for i, k in enumerate(R_I):
            wv[order[i]] = wheels[k]
        art.set_joint_velocity_targets(wv.reshape(1, -1))
        pos, ori = art.get_world_poses()
        pose[0], pose[1] = float(pos[0][0]), float(pos[0][1])
        qw, qx, qy, qz = [float(v) for v in ori[0]]
        pose[2] = math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))
        joint_q = (joint_q + np.array([wheels[k] for k in R_I]) * dt + math.pi) % (2 * math.pi) - math.pi
        return
    c, s = math.cos(pose[2]), math.sin(pose[2])
    pose[0] += (vx * c - vy * s) * dt
    pose[1] += (vx * s + vy * c) * dt
    pose[2] += wz * dt
    q = [math.cos(pose[2] / 2), 0.0, 0.0, math.sin(pose[2] / 2)]
    art.set_world_poses(positions=np.array([[pose[0], pose[1], 0.0]]), orientations=np.array([q]))
    wheels = ik(vx, vy, wz)
    joint_q = (joint_q + np.array([wheels[k] for k in R_I]) * dt + math.pi) % (2 * math.pi) - math.pi
    art.set_joint_positions(np.array([[joint_q[i] for i in order]]))


phase_i = 0
phase_t0 = 0.0
while time.time() - T0 < args.duration + 40.0:
    t_loop = time.time()
    sim_app.update()
    try:
        t = float(sim_t_attr.get())
    except Exception:
        continue
    if t <= 0.0:
        continue
    dt = 0.0 if prev_t == 0.0 else max(0.0, min(0.2, t - prev_t))
    prev_t = t

    # 阶段推进
    name, vx, vy, wz, hold = PHASES[phase_i % len(PHASES)]
    if t - phase_t0 >= hold:
        phase_i += 1
        phase_t0 = t
        name, vx, vy, wz, hold = PHASES[phase_i % len(PHASES)]
        say("阶段 -> %s" % name)
    apply_motion(vx, vy, wz, dt)

    row = [round(t, 4), name, vx, vy, wz] + [round(v, 4) for v in pose]
    cloud_pub, npts, rmin, rmax, wx, wy = 0, 0, None, None, None, None
    t_lidar = None

    # 雷达按 --lidar-hz 触发
    if t >= next_cloud:
        next_cloud = t + 1.0 / args.lidar_hz
        t_a = time.time()
        local, r, wall = lidar_rays(pose[0], pose[1], pose[2])
        if local is not None and len(local):
            cloud_data.set(local.tolist())
            _push(EV_CLOUD)
            cloud_pub, npts = 1, len(local)
            rmin, rmax = float(r.min()), float(r.max())
            if wall is not None:
                c, s = math.cos(pose[2]), math.sin(pose[2])
                dx, dy = wall[0] - pose[0], wall[1] - pose[1]
                wx, wy = dx * c + dy * s, -dx * s + dy * c
                last_stats["wall"] = (wx, wy)
        else:
            skipped += 1
        t_lidar = (time.time() - t_a) * 1000.0
        frame_times.append(t_lidar)
        n_cloud += cloud_pub

    # IMU：每物理帧发一次（数据由 IMUSensor 在物理步里更新；发布不可能快于帧率）
    imu_pub = 0
    pub_imu_now = dt > 0.0
    fr = imu.get_current_frame(read_gravity=True)
    ax = ay = az = gx = gy = gz = 0.0
    if fr:
        la, av = fr.get("lin_acc"), fr.get("ang_vel")
        ori = fr.get("orientation")
        if la is not None and av is not None:
            ax, ay, az = [float(v) for v in np.asarray(la).reshape(-1)[:3]]
            gx, gy, gz = [float(v) for v in np.asarray(av).reshape(-1)[:3]]
            if pub_imu_now:
                imu_lacc.set([ax, ay, az])
                imu_avel.set([gx, gy, gz])
                if ori is not None:
                    o = [float(v) for v in np.asarray(ori).reshape(-1)[:4]]
                    imu_ori.set([o[1], o[2], o[3], o[0]])   # 输入顺序 IJKR
                _push(EV_IMU)
                imu_pub = 1
                n_imu += 1

    w.writerow(row + [cloud_pub, npts, rmin, rmax, wx, wy,
                      round(ax, 3), round(ay, 3), round(az, 3),
                      round(gx, 4), round(gy, 4), round(gz, 4),
                      round(t_lidar, 2) if t_lidar else None, imu_pub, skipped,
                      int(cnt_tick.get()) if cnt_tick else 0])
    if args.target_rtf > 0:
        spent = time.time() - t_loop
        target = 1.0 / (60.0 * args.target_rtf)
        if target > spent:
            time.sleep(min(0.03, target - spent))
    if t >= args.duration:
        break

f.close()
timeline.stop()

say("")
say("== 统计 ==")
say("点云发布次数=%d，IMU 发布次数=%d，空点云帧=%d" % (n_cloud, n_imu, skipped))
if frame_times:
    say("单帧射线耗时: 均值 %.1f ms / 最大 %.1f ms（%d 条射线）"
        % (np.mean(frame_times), np.max(frame_times), len(DIR_LOCAL)))
if "wall" in last_stats:
    say("墙体在机体坐标系的位置: x=%.2f y=%.2f m（用于核对朝向）" % last_stats["wall"])
say("日志: %s" % CSV)
open(SUMMARY, "w").write("\n".join(lines) + "\n")
sim_app.close()
