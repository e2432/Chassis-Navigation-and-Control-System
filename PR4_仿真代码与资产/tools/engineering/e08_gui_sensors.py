"""在 Isaac Sim **GUI 窗口里**验证 E08 的雷达 + IMU（可视化入口）。

用法（先打开场景，再跑本脚本）：
  1. Isaac Sim 里 `File > Open` → `assets/scenes/e05_min_scene.usd`
  2. `Window > Script Editor` → `File > Open` 选中本文件 → 点 `Run`
     —— 立刻开始：底盘按剧本「原地逆时针 90° → 前进 → 顺时针 90° → 侧移」循环，
        **雷达点云会实时画在视口里**（/World/E08Debug/points），
        同时向 ROS 2 发布 /lidar/points、/imu、/clock。
  3. 外部终端（ROS_DOMAIN_ID 必须与 GUI 一致）验证：
        ros2 topic hz /lidar/points          # 期望 ~10 Hz
        ros2 topic hz /imu                 # 期望 ~60 Hz（= 物理步率）
        ros2 topic echo /imu --once
        rviz2 -d config/e08_rviz.rviz      # 或看 results/w2/e08/rviz_points.png 的效果
  4. 想停：`e08_gui_stop()`；想只发话题不掉头：`e08_gui_start(motion=False)`；
     想关掉视口里的点云：`e08_gui_start(draw=False)`。

设计说明：
  * 与 headless 版 `e08_sensor_pipeline.py` 用同一套射线投射雷达、事件触发发布、
    IMU 传感器；差别只是位姿来自 GUI 里真实的 articulation（physics 模式）。
  * **必须 physics 模式**（保留重力、用轮子速度驱动）：IMU 是物理传感器，
    瞬移（kinematic）它看不到动态，读数会退化成 0。
  * 点云用 USD `Points` 图元画，只是可视化，不参与物理、也不进 ROS。
  * 自动化冒烟：`python tools/engineering/e08_gui_sensors.py --headless-test`（不需要 GUI）。
"""

import math
import os
import sys

import numpy as np


def _import_omni():
    """omni.* / carb.* 只能在 Isaac Sim 起来之后导入。

    GUI 里 Run 时它们本来就在；headless 冒烟（--headless-test）要先 create SimulationApp，
    所以这里统一成"函数内导入 + 绑定到模块全局"。
    """
    g = globals()
    import carb
    import carb.events
    import omni
    import omni.graph.core as og_mod
    import omni.kit.app            # noqa: F401  （让 omni.kit.app 属性可用）
    import omni.physx              # noqa: F401
    import omni.timeline           # noqa: F401
    import omni.usd                # noqa: F401
    from pxr import Gf as _Gf, UsdGeom as _UsdGeom, UsdPhysics as _UsdPhysics
    g["carb"] = carb
    g["omni"] = omni
    g["og"] = og_mod
    g["Gf"] = _Gf
    g["UsdGeom"] = _UsdGeom
    g["UsdPhysics"] = _UsdPhysics

# ---------------- 与 URDF / sensors.yaml 一致的参数 ----------------
WHEEL_R = 0.105
WHEEL_C = 0.23546
PHI = {"fl": 45.0, "fr": -45.0, "rl": 135.0, "rr": 225.0}
R_I = {"fl": (WHEEL_C, WHEEL_C), "fr": (WHEEL_C, -WHEEL_C),
       "rl": (-WHEEL_C, WHEEL_C), "rr": (-WHEEL_C, -WHEEL_C)}
U = {k: (math.sin(math.radians(v)), -math.cos(math.radians(v))) for k, v in PHI.items()}

ROBOT_ROOT = "/World/robot_a/base_footprint"
GRAPH = "/E08GuiSensors"
LIDAR_FRAME = "lidar_front_link"
LIDAR_TOPIC, IMU_TOPIC = "/lidar/points", "/imu"
LIDAR_XY = 0.13                      # 前雷达 |x|=|y|
LIDAR_Z_REL = 0.146 + 0.04 + 0.065 / 2.0     # base_link 之上的雷达高度 = 0.2185
POINTS_PATH = "/World/E08Debug/points"

H_RAYS, V_RAYS = 360, 8
VFOV = (-7.0, 52.0)
MAX_RANGE = 40.0
LIDAR_HZ = 10.0
IMU_HZ_SENSOR = 200               # 传感器采样率；ROS 发布按物理步（≈60 Hz）

# 剧本：(说明, vx, vy, wz, 持续秒)
SCRIPT = [
    ("静止 3 s（IMU 应只看到重力 az≈9.81）", 0.0, 0.0, 0.0, 3.0),
    ("原地逆时针 90°（wz=+0.5，IMU wz→+0.46）", 0.0, 0.0, 0.5, math.pi / 2 / 0.5),
    ("前进（点云里墙应保持在正前方 3 m 左右）", 0.15, 0.0, 0.0, 3.0),
    ("原地顺时针 90°（wz=−0.5）", 0.0, 0.0, -0.5, math.pi / 2 / 0.5),
    ("侧移（IMU wz≈0，说明横移不转向）", 0.0, 0.15, 0.0, 3.0),
    ("静止 2 s", 0.0, 0.0, 0.0, 2.0),
]

# 射线方向（雷达坐标系内固定）
_el = np.radians(np.linspace(VFOV[0], VFOV[1], V_RAYS))
_az = np.radians(np.arange(H_RAYS) * 360.0 / H_RAYS)
_AZ, _EL = np.meshgrid(_az, _el)
DIR_LOCAL = np.stack([np.cos(_EL) * np.cos(_AZ), np.cos(_EL) * np.sin(_AZ), np.sin(_EL)],
                     axis=-1).reshape(-1, 3)

_STATE = {}


def ik(vx, vy, wz):
    out = {}
    for k, (rx, ry) in R_I.items():
        vix, viy = vx - wz * ry, vy + wz * rx
        out[k] = (vix * U[k][0] + viy * U[k][1]) / WHEEL_R
    return out


def _log(msg):
    print("[E08-GUI] %s" % msg, flush=True)


def _remove_graph():
    stage = omni.usd.get_context().get_stage()
    if stage.GetPrimAtPath(GRAPH).IsValid():
        try:
            stage.RemovePrim(GRAPH)
            _log("已删除旧的 %s（重复运行前清理）" % GRAPH)
        except Exception as exc:
            _log("删除旧图失败: %r" % (exc,))


def _build_graph():
    """OnCustomEvent 边沿触发发布：一次事件 = 一次发布（不能直接挂 OnPlaybackTick）。"""
    _remove_graph()
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
                ("PubImu.inputs:frameId", "imu_link"),
                ("PubImu.inputs:queueSize", 50),
               ("PubImu.inputs:qosProfile", '{"history": "keepLast", "depth": 50, "reliability": "bestEffort", "durability": "volatile", "deadline": 0.0, "lifespan": 0.0, "liveliness": "systemDefault", "leaseDuration": 0.0}'),
                ("PubImu.inputs:publishOrientation", True),
                ("PubImu.inputs:publishLinearAcceleration", True),
                ("PubImu.inputs:publishAngularVelocity", True),
                ("EvCloud.inputs:eventName", "e08_gui_pub_cloud"),
                ("EvImu.inputs:eventName", "e08_gui_pub_imu"),
                ("EvCloud.inputs:onlyPlayback", True),
                ("EvImu.inputs:onlyPlayback", True),
            ],
            og.Controller.Keys.CONNECT: [
                ("Tick.outputs:tick", "PubClock.inputs:execIn"),
                ("Tick.outputs:tick", "CntTick.inputs:execIn"),
                ("Ctx.outputs:context", "PubClock.inputs:context"),
                ("Ctx.outputs:context", "PubCloud.inputs:context"),
                ("Ctx.outputs:context", "PubImu.inputs:context"),
                ("ReadSimTime.outputs:simulationTime", "PubClock.inputs:timeStamp"),
                ("ReadSimTime.outputs:simulationTime", "PubCloud.inputs:timeStamp"),
                ("ReadSimTime.outputs:simulationTime", "PubImu.inputs:timeStamp"),
                ("EvCloud.outputs:execOut", "PubCloud.inputs:execIn"),
                ("EvImu.outputs:execOut", "PubImu.inputs:execIn"),
            ],
        },
    )
    A = lambda n: og.Controller.attribute("%s/%s" % (GRAPH, n))    # noqa: E731
    return {
        "sim_t": A("ReadSimTime.outputs:simulationTime"),
        "cloud_data": A("PubCloud.inputs:data"),
        "imu_lacc": A("PubImu.inputs:linearAcceleration"),
        "imu_avel": A("PubImu.inputs:angularVelocity"),
        "imu_ori": A("PubImu.inputs:orientation"),
        "ticks": A("CntTick.outputs:count"),
    }


def _make_points_prim():
    stage = omni.usd.get_context().get_stage()
    stage.DefinePrim("/World/E08Debug", "Xform")
    prim = stage.DefinePrim(POINTS_PATH, "Points")
    pts = UsdGeom.Points(prim)
    pts.CreateDisplayColorAttr([Gf.Vec3f(0.1, 0.9, 0.2)])
    pts.CreateWidthsAttr([0.02])
    return pts


def e08_gui_start(mode="physics", motion=True, draw=True, ros=True, reset_pose=False):
    """在已打开场景的 Isaac Sim 里启动 E08 传感器。

    mode="physics"  保留重力 + 轮子速度驱动（IMU 才有真实动态；默认）
    motion=False    只发传感器，不让底盘动（想手动摆位/用其它驱动时）
    draw=True       把点云画到 /World/E08Debug/points（视口可见）
    ros=True        发布 /lidar/points、/imu、/clock
    reset_pose=True 先把底盘放回原点（车被上次的 bug 甩飞了就用这个复位）
    """
    _import_omni()
    e08_gui_stop()

    stage = omni.usd.get_context().get_stage()
    if not stage.GetPrimAtPath(ROBOT_ROOT).IsValid():
        _log("找不到 %s —— 请先打开 assets/scenes/e05_min_scene.usd" % ROBOT_ROOT)
        return False

    _log("ROS_DOMAIN_ID=%s（GUI 进程视角；外部终端必须一致，未设置即默认 0）"
         % os.environ.get("ROS_DOMAIN_ID", "<未设置=0>"))

    have_ros = False
    if ros:
        mgr = omni.kit.app.get_app().get_extension_manager()
        if not mgr.is_extension_enabled("isaacsim.ros2.bridge"):
            mgr.set_extension_enabled_immediate("isaacsim.ros2.bridge", True)
        have_ros = mgr.is_extension_enabled("isaacsim.ros2.bridge")
        if not have_ros:
            _log("ROS 2 bridge 不可用：只做视口可视化")

    attrs = _build_graph() if have_ros else None

    # ---- IMU（物理传感器：必须 gravity on + 真被物理驱动）----
    ix = LIDAR_XY
    from isaacsim.sensors.physics import IMUSensor
    imu = IMUSensor(prim_path="/World/robot_a/base_link/ImuE08",
                    frequency=IMU_HZ_SENSOR,
                    translation=np.array([ix, ix, LIDAR_Z_REL]))
    _log("IMU: %s freq=%d Hz（含重力）" % (imu.prim_path, imu.get_frequency()))

    # ---- 重力：UsdPhysics 的 gravityMagnitude 是"正的标量大小"，方向由 gravityDirection
    #      决定（默认 −z）。写负值等于重力朝上 → 机器人会原地起飞（早期版本的 bug）。
    #      这里只在"确实不是正常重力"时才纠正，并记下原值以便停止时还原。
    grav_attr, grav_saved = None, None
    scene = stage.GetPrimAtPath("/World/PhysicsScene")
    if scene.IsValid():
        grav_attr = UsdPhysics.Scene(scene).GetGravityMagnitudeAttr()
        grav_saved = grav_attr.Get()
        dir_attr = UsdPhysics.Scene(scene).GetGravityDirectionAttr()
        gdir = dir_attr.Get() if dir_attr else Gf.Vec3f(0.0, 0.0, -1.0)
        need_fix = grav_saved is None or grav_saved <= 1e-6
        if mode == "physics" and need_fix:
            grav_attr.Set(9.81)
            _log("重力被改成 9.81（原值 %s —— 之前被关过/写反了，会让车飘或飞）" % grav_saved)
        _log("physics 模式：重力大小=%s 方向=%s（速度驱动 kps=0, kds=1e4, maxEffort=80）"
             % (grav_attr.Get(), tuple(round(v, 2) for v in gdir)))
        if grav_attr.Get() is not None and grav_attr.Get() < 0:
            _log("发现负的重力大小（重力朝上，会导致起飞），已纠正为 9.81")
            grav_attr.Set(9.81)
    else:
        _log("警告：找不到 /World/PhysicsScene，重力未知")
    if mode != "physics":
        _log("警告：mode=%s 不是 physics，IMU 会退化为 0（只适合看点云）" % mode)

    from isaacsim.core.prims import Articulation
    art = Articulation(prim_paths_expr=ROBOT_ROOT)

    pts_geom = _make_points_prim() if draw else None
    if pts_geom is not None:
        _log("点云可视化: %s（绿色点，随雷达更新）" % POINTS_PATH)

    bus = omni.kit.app.get_app().get_message_bus_event_stream()
    _STATE.clear()
    _STATE.update(dict(
        art=art, attrs=attrs, imu=imu, pts=pts_geom, draw=draw, motion=motion,
        grav_attr=grav_attr, grav_saved=grav_saved, z_max=0.0, z_warned=False,
        mode=mode, started=False, order=list(range(4)), joint_q=np.zeros(4),
        prev_t=None, next_cloud=0.0, script_i=0, script_t0=None,
        n_cloud=0, n_imu=0, frames=0, skipped=0,
        bus=bus,
        ev_cloud=carb.events.type_from_string("omni.graph.action.e08_gui_pub_cloud"),
        ev_imu=carb.events.type_from_string("omni.graph.action.e08_gui_pub_imu"),
    ))

    if reset_pose:
        try:
            art.set_world_poses(positions=np.array([[0.0, 0.0, 0.0]]),
                                orientations=np.array([[1.0, 0.0, 0.0, 0.0]]))
            _log("已把底盘复位到原点（reset_pose=True）")
        except Exception as exc:
            _log("复位失败（不影响运行）: %r" % (exc,))

    omni.timeline.get_timeline_interface().play()
    _STATE["sub"] = (omni.kit.app.get_app().get_update_event_stream()
                     .create_subscription_to_pop(_on_update, name="e08_gui_sensors"))
    _log("开始。motion=%s draw=%s ros=%s —— 外部终端可查 ros2 topic hz %s"
         % (motion, draw, have_ros, LIDAR_TOPIC))
    if not have_ros:
        _log("提示：没有 ROS 时只有视口可视化；要发话题请在 ROS 2 环境里启动 Isaac Sim")
    return True


def _publish_cloud(st, local):
    st["attrs"]["cloud_data"].set(local.tolist())
    st["bus"].push(st["ev_cloud"], payload={})
    st["n_cloud"] += 1


def _lidar_local(pose):
    """世界系发射线，返回雷达坐标系内的命中点。"""
    px, py, yaw = pose
    cz, sz = math.cos(yaw), math.sin(yaw)
    ox = px + LIDAR_XY * cz - LIDAR_XY * sz
    oy = py + LIDAR_XY * sz + LIDAR_XY * cz
    oz = LIDAR_Z_REL + WHEEL_R
    origin = Gf.Vec3d(ox, oy, oz)
    Rw = np.array([[cz, -sz, 0.0], [sz, cz, 0.0], [0.0, 0.0, 1.0]])
    dw = DIR_LOCAL @ Rw.T
    q = omni.physx.get_physx_scene_query_interface()
    pts = []
    for d in dw:
        hits = []

        def _cb(h, _hits=hits):
            _hits.append(h)
            return True

        q.raycast_all(origin, Gf.Vec3d(float(d[0]), float(d[1]), float(d[2])), MAX_RANGE, _cb)
        for h in hits:
            path = str(getattr(h, "collision", ""))
            if path.startswith("/World/robot_a"):
                continue                      # 排除自身（−7° 会打到车顶）
            pos = getattr(h, "position", None)
            if pos is not None:
                pts.append((pos[0], pos[1], pos[2]))
            break
    if not pts:
        return None
    P = np.array(pts, dtype=float)
    return (P - np.array([ox, oy, oz])) @ Rw


def _on_update(evt):
    st = _STATE
    if not st or st.get("art") is None:
        return
    timeline = omni.timeline.get_timeline_interface()
    if not timeline.is_playing():
        return
    try:
        t = float(st["attrs"]["sim_t"].get()) if st.get("attrs") else float(timeline.get_current_time())
    except Exception:
        t = float(timeline.get_current_time())
    if t <= 0.0:
        return
    st["frames"] += 1
    if not st["started"]:
        st["art"].initialize()
        dn = list(st["art"].dof_names or [])
        if len(dn) == 4:
            st["order"] = [next((i for i, n in enumerate(dn)
                                 if n.startswith("base_wheel_%s" % k)), i) for i, k in enumerate(R_I)]
        if st["mode"] == "physics":
            for i in range(st["art"].num_dof):
                st["art"].switch_dof_control_mode("velocity", i)
            st["art"].set_gains(kps=np.zeros(st["art"].num_dof),
                                kds=np.full(st["art"].num_dof, 1.0e4))
            st["art"].set_max_efforts(np.full(st["art"].num_dof, 80.0))
        st["started"] = True
        _log("articulation 初始化: DOF=%d 关节=%s" % (st["art"].num_dof, dn))

    dt = 0.0 if st["prev_t"] is None else max(0.0, min(0.2, t - st["prev_t"]))
    st["prev_t"] = t

    # ---- 剧本驱动（physics：给轮子速度目标；位姿由 PhysX 算）----
    if st["motion"]:
        if st["script_t0"] is None:
            st["script_t0"] = t
        name, vx, vy, wz, hold = SCRIPT[st["script_i"] % len(SCRIPT)]
        if t - st["script_t0"] >= hold:
            st["script_i"] += 1
            st["script_t0"] = t
            name, vx, vy, wz, hold = SCRIPT[st["script_i"] % len(SCRIPT)]
            _log("剧本: %s" % name)
        if st["mode"] == "physics":
            wheels = ik(vx, vy, wz)
            wv = np.zeros(st["art"].num_dof)
            for i, k in enumerate(R_I):
                wv[st["order"][i]] = wheels[k]
            st["art"].set_joint_velocity_targets(wv.reshape(1, -1))

    pos, ori = st["art"].get_world_poses()
    px, py = float(pos[0][0]), float(pos[0][1])
    pz = float(pos[0][2])
    st["z_max"] = max(st.get("z_max", 0.0), abs(pz))
    if abs(pz) > 0.3 and not st.get("z_warned"):
        st["z_warned"] = True
        _log("警告：\"base_footprint\" 高度 z=%.2f m（正常应≈0）—— 检查重力是否为负/为 0，"
             "或场景里 PhysicsScene 的 gravityDirection" % pz)
    qw, qx, qy, qz = [float(v) for v in ori[0]]
    yaw = math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))
    pose = (px, py, yaw)

    # ---- 雷达：按 10 Hz 仿真时间触发 ----
    if t >= st["next_cloud"]:
        st["next_cloud"] = t + 1.0 / LIDAR_HZ
        local = _lidar_local(pose)
        if local is not None and len(local):
            if st["draw"] and st["pts"] is not None:
                world = local @ np.array([[math.cos(yaw), -math.sin(yaw), 0.0],
                                          [math.sin(yaw), math.cos(yaw), 0.0],
                                          [0.0, 0.0, 1.0]]) + np.array(
                    [px + LIDAR_XY * math.cos(yaw) - LIDAR_XY * math.sin(yaw),
                     py + LIDAR_XY * math.sin(yaw) + LIDAR_XY * math.cos(yaw),
                     LIDAR_Z_REL + WHEEL_R])
                st["pts"].GetPointsAttr().Set([Gf.Vec3f(float(p[0]), float(p[1]), float(p[2]))
                                               for p in world])
            if st.get("attrs"):
                _publish_cloud(st, local)
            else:
                st["n_cloud"] += 1
        else:
            st["skipped"] += 1

    # ---- IMU：每物理步发一次 ----
    fr = st["imu"].get_current_frame(read_gravity=True) if dt > 0.0 else None
    if fr:
        la, av = fr.get("lin_acc"), fr.get("ang_vel")
        if la is not None and av is not None and st.get("attrs"):
            st["attrs"]["imu_lacc"].set([float(v) for v in np.asarray(la).reshape(-1)[:3]])
            st["attrs"]["imu_avel"].set([float(v) for v in np.asarray(av).reshape(-1)[:3]])
            o = fr.get("orientation")
            if o is not None:
                o = [float(v) for v in np.asarray(o).reshape(-1)[:4]]
                st["attrs"]["imu_ori"].set([o[1], o[2], o[3], o[0]])   # IJKR
            st["bus"].push(st["ev_imu"], payload={})
            st["n_imu"] += 1


def e08_gui_stop():
    st = _STATE
    if not st:
        _log("没有在运行")
        return
    st["sub"] = None
    if st.get("grav_attr") is not None and st.get("grav_saved") is not None:
        try:
            st["grav_attr"].Set(st["grav_saved"])      # 还原原重力（含被关成 0 的情况）
        except Exception:
            pass
    if st.get("started") and st.get("art") is not None:
        try:
            st["art"].set_joint_velocity_targets(np.zeros((1, st["art"].num_dof)))
        except Exception:
            pass
    _log("已停止：点云 %d 帧、IMU %d 条（空帧 %d），期间 |base_footprint.z| 最大 %.3f m"
         % (st.get("n_cloud", 0), st.get("n_imu", 0), st.get("skipped", 0), st.get("z_max", 0.0)))
    _STATE.clear()


# ---------------- 入口：GUI 里 Run 即启动；--headless-test 做自动冒烟 ----------------
def _headless_smoke(duration=10.0):
    from isaacsim import SimulationApp
    app = SimulationApp({"headless": True})
    _import_omni()
    ws = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    scene = os.path.join(ws, "assets/scenes/e05_min_scene.usd")
    omni.usd.get_context().open_stage(scene)
    for _ in range(8):
        app.update()
    if os.environ.get("E08_TEST_BAD_GRAVITY"):
        # 自测：故意把重力写成负值（= 重力朝上，会让机器人起飞），验证脚本能自动纠正
        st_ = omni.usd.get_context().get_stage()
        UsdPhysics.Scene(st_.GetPrimAtPath("/World/PhysicsScene")).GetGravityMagnitudeAttr().Set(-9.81)
        print("[E08-GUI] 自测：已把重力改成 −9.81（模拟旧版本的 bug）", flush=True)
    if not e08_gui_start(motion=True, draw=True, ros=True):
        app.close()
        return 1
    import time
    t0 = time.time()
    while time.time() - t0 < duration:
        app.update()
    st = dict(_STATE)
    n_cloud, n_imu, frames, skipped = (st.get("n_cloud", 0), st.get("n_imu", 0),
                                       st.get("frames", 0), st.get("skipped", 0))
    sim_t = float(st["attrs"]["sim_t"].get()) if st.get("attrs") else 0.0
    e08_gui_stop()
    ok = n_cloud > 0 and n_imu > 0 and skipped == 0
    print("[E08-GUI] 冒烟结果: 仿真时间 %.1f s，点云 %d 帧（%.2f Hz），IMU %d 条（%.2f Hz），空帧 %d → %s"
          % (sim_t, n_cloud, n_cloud / max(1e-6, sim_t), n_imu, n_imu / max(1e-6, sim_t),
             skipped, "PASS" if ok else "FAIL"), flush=True)
    app.close()
    return 0 if ok else 1


if __name__ == "__main__" or True:
    if "--headless-test" in sys.argv:
        sys.exit(_headless_smoke())
    else:
        e08_gui_start()
