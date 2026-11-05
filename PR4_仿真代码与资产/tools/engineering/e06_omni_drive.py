#!/usr/bin/env python3
"""E06：全向底盘速度输入（仿真侧驱动）。

接收机体坐标系速度指令 (vx, vy, ωz)，按 45° 全向轮布局做逆解，驱动四个轮子；
同时把位姿与速度通过 /odom 发出去，并把全过程写成 CSV 供 e06_analyze.py 判定。

两种驱动模式（--mode）：
  kinematic  理想运动学模型（**早期接口模型**）：直接按指令积分机体位姿，再做轮子视觉转动。
             不涉及轮地接触/摩擦，不能当作接触动力学验证（路线书 E06 明确允许此阶段做法）。
  physics    接触驱动试探：把轮角速度下发给关节，让 PhysX 通过轮地摩擦推动底盘。
             圆柱轮没有横向滚子，侧移会被摩擦挡住 —— 本模式就是用来量化这件事的。

速度输入契约（见 docs/接口契约.md 第 3 节）：
  linear.x=+前, linear.y=+左, angular.z=+逆时针(右手)，单位 m/s 与 rad/s；linear.z/angular.x/y 必须为 0。
  超时 0.5 s 未收到新指令 -> 速度归零（活性由 /e06/heartbeat 心跳判定，见 motion_check.md）。
"""

import argparse
import csv
import math
import os
import sys
import time

parser = argparse.ArgumentParser()
parser.add_argument("--out", required=True)
parser.add_argument("--scene", default="assets/scenes/e05_min_scene.usd")
parser.add_argument("--mode", choices=["kinematic", "physics"], default="kinematic")
parser.add_argument("--duration", type=float, default=40.0, help="仿真时长（秒）")
parser.add_argument("--target-rtf", type=float, default=1.0, help="限速到约 1 倍实时，便于外部按仿真时间发指令")
parser.add_argument("--max-vx", type=float, default=2.0)
parser.add_argument("--max-vy", type=float, default=2.0)
parser.add_argument("--max-wz", type=float, default=1.0)
parser.add_argument("--cmd-timeout", type=float, default=0.5)
args, _ = parser.parse_known_args()

OUT = os.path.abspath(args.out)
os.makedirs(OUT, exist_ok=True)
CSV = os.path.join(OUT, "drive_log.csv")

# ---- 底盘几何（与 robot_description/config/robot_geometry.yaml 一致）----
WHEEL_R = 0.105
WHEEL_C = 0.23546          # 轮心 |x| = |y|
PHI = {"fl": 45.0, "fr": -45.0, "rl": 135.0, "rr": 225.0}   # 轮轴水平方位角
R_I = {"fl": (WHEEL_C, WHEEL_C), "fr": (WHEEL_C, -WHEEL_C),
       "rl": (-WHEEL_C, WHEEL_C), "rr": (-WHEEL_C, -WHEEL_C)}
# 轮子滚动方向 u = (sinφ, -cosφ)：正角速度时轮心前进方向（推导见 docs/engineering/motion_check.md）
U = {k: (math.sin(math.radians(v)), -math.cos(math.radians(v))) for k, v in PHI.items()}


def ik(vx, vy, wz):
    """机体速度 -> 四轮角速度（rad/s）。"""
    out = {}
    for k, (rx, ry) in R_I.items():
        vix = vx - wz * ry          # 轮心处机体点速度（ẑ×r = (-ωy, ωx)）
        viy = vy + wz * rx
        out[k] = (vix * U[k][0] + viy * U[k][1]) / WHEEL_R
    return out


def fk(wheels):
    """四轮角速度 -> 机体速度（最小二乘，用于内部一致性自检）。"""
    import numpy as np
    A, b = [], []
    for k, (rx, ry) in R_I.items():
        A.append([U[k][0], U[k][1], rx * U[k][1] - ry * U[k][0]])
        b.append(wheels[k] * WHEEL_R)
    sol, *_ = np.linalg.lstsq(np.array(A), np.array(b), rcond=None)
    return sol


print("[E06-SIM] mode=%s scene=%s out=%s" % (args.mode, args.scene, OUT), flush=True)
# 逆解/正解自检：任意速度 -> 轮速 -> 反解应还原
for probe in ((0.1, 0.0, 0.0), (0.0, 0.1, 0.0), (0.0, 0.0, 0.3), (0.1, 0.05, 0.2)):
    back = fk(ik(*probe))
    print("[E06-SIM] IK/FK 自检 cmd=%s -> %s" % (probe, [round(float(v), 6) for v in back]), flush=True)

from isaacsim import SimulationApp  # noqa: E402

sim_app = SimulationApp({"headless": True})
import numpy as np  # noqa: E402
import omni.graph.core as og  # noqa: E402
import omni.kit.app  # noqa: E402
import omni.timeline  # noqa: E402
import omni.usd  # noqa: E402
from isaacsim.core.prims import Articulation  # noqa: E402
from pxr import Gf, UsdPhysics  # noqa: E402

app = omni.kit.app.get_app()
ext_mgr = app.get_extension_manager()
for ext in ("isaacsim.ros2.bridge",):
    if not ext_mgr.is_extension_enabled(ext):
        ext_mgr.set_extension_enabled_immediate(ext, True)
        sim_app.update()
print("[E06-SIM] bridge enabled=%s" % ext_mgr.is_extension_enabled("isaacsim.ros2.bridge"), flush=True)

# ---- 打开 E05 最终产出场景 ----
omni.usd.get_context().open_stage(os.path.abspath(args.scene))
for _ in range(5):
    sim_app.update()
stage = omni.usd.get_context().get_stage()
print("[E06-SIM] scene opened: %s" % stage.GetRootLayer().identifier, flush=True)

# kinematic 模式关闭重力：这是理想运动学模型，不模拟轮地支撑
scene_prim = stage.GetPrimAtPath("/World/PhysicsScene")
if scene_prim.IsValid() and args.mode == "kinematic":
    UsdPhysics.Scene(scene_prim).GetGravityMagnitudeAttr().Set(0.0)
    print("[E06-SIM] kinematic 模式：重力置 0（不做轮地支撑）", flush=True)

ART_PATH = "/World/robot_a/base_footprint"
art = Articulation(prim_paths_expr=ART_PATH)
# 需要先有 physics view：先播一帧
timeline = omni.timeline.get_timeline_interface()

# ---- Action Graph：订阅指令与心跳，发布 /clock 与 /odom ----
GRAPH = "/E06Drive"
og.Controller.edit(
    {"graph_path": GRAPH, "evaluator_name": "execution"},
    {
        og.Controller.Keys.CREATE_NODES: [
            ("Tick", "omni.graph.action.OnPlaybackTick"),
            ("Ctx", "isaacsim.ros2.bridge.ROS2Context"),
            ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
            ("PubClock", "isaacsim.ros2.bridge.ROS2PublishClock"),
            ("SubCmd", "isaacsim.ros2.bridge.ROS2SubscribeTwist"),
            ("SubBeat", "isaacsim.ros2.bridge.ROS2SubscribeTwist"),
            ("PubOdom", "isaacsim.ros2.bridge.ROS2PublishOdometry"),
        ],
        og.Controller.Keys.SET_VALUES: [
            ("ReadSimTime.inputs:resetOnStop", False),
            ("PubClock.inputs:topicName", "/clock"),
            ("SubCmd.inputs:topicName", "/cmd_vel"),
            ("SubCmd.inputs:queueSize", 10),
            ("SubBeat.inputs:topicName", "/e06/heartbeat"),
            ("SubBeat.inputs:queueSize", 10),
            ("PubOdom.inputs:topicName", "/odom"),
            ("PubOdom.inputs:odomFrameId", "odom"),
            ("PubOdom.inputs:chassisFrameId", "base_link"),
            ("PubOdom.inputs:publishRawVelocities", True),
        ],
        og.Controller.Keys.CONNECT: [
            ("Tick.outputs:tick", "PubClock.inputs:execIn"),
            ("Tick.outputs:tick", "SubCmd.inputs:execIn"),
            ("Tick.outputs:tick", "SubBeat.inputs:execIn"),
            ("Tick.outputs:tick", "PubOdom.inputs:execIn"),
            ("Ctx.outputs:context", "PubClock.inputs:context"),
            ("Ctx.outputs:context", "SubCmd.inputs:context"),
            ("Ctx.outputs:context", "SubBeat.inputs:context"),
            ("Ctx.outputs:context", "PubOdom.inputs:context"),
            ("ReadSimTime.outputs:simulationTime", "PubClock.inputs:timeStamp"),
        ],
    },
)
A = lambda name: og.Controller.attribute("%s/%s" % (GRAPH, name))  # noqa: E731
sim_time_attr = A("ReadSimTime.outputs:simulationTime")   # 与 /clock 同源，作为唯一时间基准
cmd_lin, cmd_ang = A("SubCmd.outputs:linearVelocity"), A("SubCmd.outputs:angularVelocity")
beat_seq = A("SubBeat.outputs:angularVelocity")
odom_pos, odom_ori = A("PubOdom.inputs:position"), A("PubOdom.inputs:orientation")
odom_lin, odom_ang = A("PubOdom.inputs:linearVelocity"), A("PubOdom.inputs:angularVelocity")

timeline.play()
for _ in range(5):
    sim_app.update()
art.initialize()
dof_names = list(art.dof_names)
order = [next(i for i, n in enumerate(dof_names) if n.startswith("base_wheel_%s" % k)) for k in R_I]
print("[E06-SIM] dof_names=%s" % dof_names, flush=True)
print("[E06-SIM] IK 顺序 index=%s (fl,fr,rl,rr)" % order, flush=True)

if args.mode == "physics":
    # URDF 导入的连续关节默认驱动往往刚度/阻尼不足，速度目标不会生效：
    # 显式切成“速度驱动”（stiffness=0、较大 damping、给出最大力），否则轮子根本不转。
    # 5.x 的 Articulation 没有 get_dof_properties；用 set_gains + set_max_efforts 配速度驱动
    try:
        for i in range(4):
            art.switch_dof_control_mode("velocity", i)
        art.set_gains(kps=np.zeros(4), kds=np.full(4, 1.0e4))
        art.set_max_efforts(np.full(4, 80.0))
        print("[E06-SIM] physics 模式：速度驱动已配置 (kps=0, kds=1e4, maxEffort=80)", flush=True)
    except Exception as exc:
        print("[E06-SIM] 配置速度驱动失败: %r" % (exc,), flush=True)

def _actual_wheels():
    """实测四轮角速度（按 fl,fr,rl,rr 顺序）；失败时返回 0。"""
    try:
        v = np.asarray(art.get_joint_velocities()).reshape(-1)
        return [float(v[i]) for i in order]
    except Exception:
        return [0.0, 0.0, 0.0, 0.0]


T0 = time.time()
f = open(CSV, "w", newline="")
w = csv.writer(f)
w.writerow(["sim_t", "cmd_vx", "cmd_vy", "cmd_wz", "app_vx", "app_vy", "app_wz",
            "x", "y", "yaw", "meas_vx", "meas_vy", "meas_wz",
            "w_fl", "w_fr", "w_rl", "w_rr", "actual_w0", "actual_w1", "actual_w2", "actual_w3",
            "cmd_lost_s", "timeout", "beat_seq"])

pose = np.array([0.0, 0.0, 0.0])   # x, y, yaw
joint_q = np.zeros(4)
applied = (0.0, 0.0, 0.0)
last_beat = None
last_beat_t = 0.0
prev_pose = pose.copy()
prev_t = 0.0
rows = 0

while time.time() - T0 < args.duration + 20.0:
    t_loop = time.time()
    sim_app.update()
    try:
        t = float(sim_time_attr.get())
    except Exception:
        continue
    if t <= 0.0:
        continue
    # ---- 读取指令与心跳（心跳用于判断发布方是否还活着）----
    try:
        lin = [float(v) for v in cmd_lin.get()]
        ang = [float(v) for v in cmd_ang.get()]
        beat = [float(v) for v in beat_seq.get()]
    except Exception:
        lin, ang, beat = [0, 0, 0], [0, 0, 0], [0, 0, 0]
    if beat[0] != last_beat:
        last_beat = beat[0]
        last_beat_t = t
    dt_beat = t - last_beat_t
    timed_out = dt_beat > args.cmd_timeout

    vx = max(-args.max_vx, min(args.max_vx, lin[0]))
    vy = max(-args.max_vy, min(args.max_vy, lin[1]))
    wz = max(-args.max_wz, min(args.max_wz, ang[2]))
    if timed_out:
        vx = vy = wz = 0.0
    applied = (vx, vy, wz)
    wheels = ik(vx, vy, wz)

    if args.mode == "kinematic":
        # 机体位姿按指令积分（机体坐标系 -> 世界系）
        d = t - prev_t if prev_t else 0.0
        c, s = math.cos(pose[2]), math.sin(pose[2])
        pose[0] += (vx * c - vy * s) * d
        pose[1] += (vx * s + vy * c) * d
        pose[2] += wz * d
        q = (math.cos(pose[2] / 2.0), 0.0, 0.0, math.sin(pose[2] / 2.0))   # w,x,y,z
        art.set_world_poses(positions=np.array([[pose[0], pose[1], 0.0]]),
                            orientations=np.array([list(q)]))
        joint_q = joint_q + np.array([wheels[k] for k in R_I]) * d
        # 连续关节角度回卷到 [-π, π)：PhysX setDriveTarget 只接受 ±2π 内，
        # 否则会刷 “only supports target angle in range [-2Pi, 2Pi]” 并让仿真停机
        joint_q = (joint_q + math.pi) % (2.0 * math.pi) - math.pi
        art.set_joint_positions(np.array([joint_q[i] for i in order]).reshape(1, 4))
    else:
        # 接触驱动：把轮角速度下发给关节速度目标，位姿由 PhysX 决定
        art.set_joint_velocity_targets(
            np.array([[[wheels[k] for k in R_I][i] for i in order][j] for j in range(4)]).reshape(1, 4))
        pos, ori = art.get_world_poses()
        pose[0], pose[1] = float(pos[0][0]), float(pos[0][1])
        qw, qx, qy, qz = [float(v) for v in ori[0]]
        pose[2] = math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))

    # 实测速度（位姿差分，世界系 -> 机体系）
    if prev_t:
        d = t - prev_t
        wvx, wvy = (pose[0] - prev_pose[0]) / d, (pose[1] - prev_pose[1]) / d
        c, s = math.cos(pose[2]), math.sin(pose[2])
        mvx, mvy = wvx * c + wvy * s, -wvx * s + wvy * c
        mwz = (pose[2] - prev_pose[2]) / d
    else:
        mvx = mvy = mwz = 0.0
    prev_pose = pose.copy()
    prev_t = t

    # 发布 /odom（位姿与机体速度）
    # OG 的属性要按普通序列赋值；orientation 是 quatd[4]，注释里写的是 IJKR 顺序
    odom_pos.set([float(pose[0]), float(pose[1]), 0.0])
    odom_ori.set([0.0, 0.0, float(math.sin(pose[2] / 2.0)), float(math.cos(pose[2] / 2.0))])
    odom_lin.set([float(vx), float(vy), 0.0])
    odom_ang.set([0.0, 0.0, float(wz)])

    w.writerow([round(t, 4)] + [round(v, 5) for v in (*applied, vx, vy, wz)] +
               [round(pose[0], 5), round(pose[1], 5), round(pose[2], 5),
                round(mvx, 5), round(mvy, 5), round(mwz, 5)] +
               [round(wheels[k], 3) for k in R_I] +
               [round(v, 3) for v in _actual_wheels()] +
               [round(dt_beat, 3), int(timed_out), round(last_beat or 0.0, 3)])
    rows += 1

    if args.target_rtf > 0:
        spent = time.time() - t_loop
        target = 1.0 / (60.0 * args.target_rtf)
        if target > spent:
            time.sleep(min(0.03, target - spent))
    if t >= args.duration:
        break

f.close()
timeline.stop()
print("[E06-SIM] 结束：%d 行 -> %s" % (rows, CSV), flush=True)
sim_app.close()
