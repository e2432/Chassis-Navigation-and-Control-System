"""在 Isaac Sim **GUI 窗口里**让底盘动起来（E06 速度输入的可视化入口）。

用法（在你已经打开场景的 Isaac Sim 里）：
  1. `Window > Script Editor`
  2. 菜单 `File > Open` 打开本文件（或直接粘贴内容），点 `Run`
  —— 打开后默认进入「演示模式」：不需要 ROS，底盘会自己按序列走
     （前进 → 左移 → 原地转 → 斜向 → 停，循环）。
  3. 想用 ROS 话题控制：在同一次会话里再执行
        e06_gui_drive(ros=True)
     然后外部终端发指令即可：
        ros2 topic pub --rate 20 /cmd_vel geometry_msgs/msg/Twist \
          '{linear: {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
  4. 停止与复位： e06_gui_stop()

设计说明：
  * 与 headless 版 `e06_omni_drive.py` 用同一套逆解与限速（4×45° 全向轮）；
  * 驱动方式是「理想运动学模型」：按指令积分位姿后写回铰接根（早期接口模型，
    不代表轮地接触动力学，见 docs/engineering/motion_check.md）；
  * 时间基准取 OG 节点 IsaacReadSimulationTime 的输出（与 /clock 同源）。
"""

import math

import numpy as np

import omni.graph.core as og
import omni.kit.app
import omni.timeline
import omni.usd

# ---------------- 底盘参数（与 src/robot_description 一致） ----------------
WHEEL_R = 0.105
WHEEL_C = 0.23546
PHI = {"fl": 45.0, "fr": -45.0, "rl": 135.0, "rr": 225.0}
R_I = {"fl": (WHEEL_C, WHEEL_C), "fr": (WHEEL_C, -WHEEL_C),
       "rl": (-WHEEL_C, WHEEL_C), "rr": (-WHEEL_C, -WHEEL_C)}
U = {k: (math.sin(math.radians(v)), -math.cos(math.radians(v))) for k, v in PHI.items()}
ROBOT_ROOT = "/World/robot_a/base_footprint"
GRAPH = "/E06GuiDrive"

# 演示序列：(说明, vx, vy, wz, 持续秒)
DEMO = [
    ("前进 vx=+0.12", 0.12, 0.0, 0.0, 2.0),
    ("后退 vx=-0.12", -0.12, 0.0, 0.0, 2.0),
    ("左移 vy=+0.12", 0.0, 0.12, 0.0, 2.0),
    ("右移 vy=-0.12", 0.0, -0.12, 0.0, 2.0),
    ("原地逆时针 wz=+0.5", 0.0, 0.0, 0.5, 3.0),
    ("原地顺时针 wz=-0.5", 0.0, 0.0, -0.5, 3.0),
    ("斜向 vx+vy", 0.12, 0.12, 0.0, 2.5),
    ("绕圈 vx+wz", 0.12, 0.0, 0.4, 3.0),
    ("停车", 0.0, 0.0, 0.0, 1.5),
]

_STATE = {}


def ik(vx, vy, wz):
    out = {}
    for k, (rx, ry) in R_I.items():
        vix, viy = vx - wz * ry, vy + wz * rx
        out[k] = (vix * U[k][0] + viy * U[k][1]) / WHEEL_R
    return out


def _log(msg):
    print("[E06-GUI] %s" % msg, flush=True)


def _remove_graph():
    """删除同名动作图。重复调用驱动（例如先演示、再 ros=True）时，同名图已存在会让
    og.Controller.edit 抛 OmniGraphError: Failed to wrap graph in node。"""
    stage = omni.usd.get_context().get_stage()
    if stage.GetPrimAtPath(GRAPH).IsValid():
        try:
            stage.RemovePrim(GRAPH)
            _log("已删除旧的 %s（重复运行前清理）" % GRAPH)
        except Exception as exc:
            _log("删除旧图失败: %r" % (exc,))


def _set_robot_collision(enable):
    """尽力关闭/恢复机器人碰撞（kinematic 模式不参与接触）。

    注意：底盘 USD 是「引用 + 实例」，其内部图元是 **实例代理**，不允许逐图元改属性
    （硬改会抛 pxr.Tf.ErrorException）。所以这里跳过实例代理、逐项 try/except，
    只做尽力而为；真正的防跑飞保护是下面的「安全区复位」。
    """
    from pxr import Usd, UsdPhysics
    stage = omni.usd.get_context().get_stage()
    root = stage.GetPrimAtPath("/World/robot_a")
    saved, skipped = [], 0
    if not root.IsValid():
        return saved
    for prim in Usd.PrimRange(root):
        try:
            if prim.IsInstanceProxy():
                skipped += 1
                continue
            if not prim.HasAPI("PhysicsCollisionAPI"):
                continue
            api = UsdPhysics.CollisionAPI(prim)
            attr = api.GetCollisionEnabledAttr() or api.CreateCollisionEnabledAttr(True)
            saved.append((attr, attr.Get()))
            attr.Set(bool(enable))
        except Exception:
            skipped += 1
    if skipped:
        _log("有 %d 个图元是实例代理/不可写，未能关闭其碰撞（靠安全区复位兜底）" % skipped)
    return saved


def _build_graph(with_ros):
    """建最小动作图：OnPlaybackTick + ReadSimTime（+ 可选 ROS 订阅 /clock 发布）。"""
    nodes = [
        ("Tick", "omni.graph.action.OnPlaybackTick"),
        ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
    ]
    values = [("ReadSimTime.inputs:resetOnStop", False)]
    conns = []
    if with_ros:
        nodes += [
            ("Ctx", "isaacsim.ros2.bridge.ROS2Context"),
            ("PubClock", "isaacsim.ros2.bridge.ROS2PublishClock"),
            ("SubCmd", "isaacsim.ros2.bridge.ROS2SubscribeTwist"),
            ("SubBeat", "isaacsim.ros2.bridge.ROS2SubscribeTwist"),
        ]
        values += [("PubClock.inputs:topicName", "/clock"),
                   ("SubCmd.inputs:topicName", "/cmd_vel"),
                   ("SubCmd.inputs:queueSize", 10),
                   ("SubBeat.inputs:topicName", "/e06/heartbeat"),
                   ("SubBeat.inputs:queueSize", 10)]
        conns += [("Tick.outputs:tick", "PubClock.inputs:execIn"),
                  ("Tick.outputs:tick", "SubCmd.inputs:execIn"),
                  ("Tick.outputs:tick", "SubBeat.inputs:execIn"),
                  ("Ctx.outputs:context", "SubBeat.inputs:context"),
                  ("Ctx.outputs:context", "PubClock.inputs:context"),
                  ("Ctx.outputs:context", "SubCmd.inputs:context"),
                  ("ReadSimTime.outputs:simulationTime", "PubClock.inputs:timeStamp")]
    edit = {"graph_path": GRAPH, "evaluator_name": "execution"}
    edits = {og.Controller.Keys.CREATE_NODES: nodes,
             og.Controller.Keys.SET_VALUES: values,
             og.Controller.Keys.CONNECT: conns}
    _remove_graph()
    try:
        og.Controller.edit(edit, edits)
    except Exception as exc:
        # 少数情况下新图尚未从 OmniGraph 注册表里退干净，删一次再建一次
        _log("首次建图失败（%s），清理后重试一次" % type(exc).__name__)
        _remove_graph()
        og.Controller.edit({"graph_path": GRAPH, "evaluator_name": "execution"}, edits)
    sim_t = og.Controller.attribute(GRAPH + "/ReadSimTime.outputs:simulationTime")
    if not with_ros:
        return sim_t, None, None
    cmd = (og.Controller.attribute(GRAPH + "/SubCmd.outputs:linearVelocity"),
           og.Controller.attribute(GRAPH + "/SubCmd.outputs:angularVelocity"))
    beat = og.Controller.attribute(GRAPH + "/SubBeat.outputs:angularVelocity")
    return sim_t, cmd, beat


def e06_gui_drive(ros=False, mode="kinematic", max_vx=2.0, max_vy=2.0, max_wz=1.0,
                  timeout=0.5, safe_radius=0.9):
    """启动 GUI 侧驱动。

    ros=False  内置演示序列（不需要 ROS）
    ros=True   跟随 /cmd_vel（外部按同一 ROS_DOMAIN_ID 发布）
    safe_radius 安全边界（米）：运动学模式写位姿不理会碰撞，越界会限位/回原点，
                避免嵌入箱体或墙。想开远一点就调大，例如 safe_radius=2.5。
    """
    e06_gui_stop()

    stage = omni.usd.get_context().get_stage()
    if not stage.GetPrimAtPath(ROBOT_ROOT).IsValid():
        _log("找不到 %s —— 请先打开 assets/scenes/e05_min_scene.usd" % ROBOT_ROOT)
        return False

    import os as _os
    _log("ROS_DOMAIN_ID=%s（GUI 进程视角；外部终端必须与此一致，未设置即默认 0）"
         % _os.environ.get("ROS_DOMAIN_ID", "<未设置=0>"))
    have_ros = False
    mgr = omni.kit.app.get_app().get_extension_manager()
    if True:
        if not mgr.is_extension_enabled("isaacsim.ros2.bridge"):
            mgr.set_extension_enabled_immediate("isaacsim.ros2.bridge", True)
            _log("已启用 isaacsim.ros2.bridge")
        have_ros = mgr.is_extension_enabled("isaacsim.ros2.bridge")
        if not have_ros:
            _log("ROS 2 bridge 不可用：改用 timeline 时间基准（演示模式仍可运行）")
        elif not ros:
            _log("提示：图里会顺带发布 /clock（既让 ReadSimTime 被求值，也方便外部核对）")

    # 只要 bridge 可用就建带 ROS 的图：ReadSimTime 需要有节点消费才会每帧求值
    sim_time_attr, cmd, beat_attr = _build_graph(have_ros)
    cmd_lin, cmd_ang = cmd if have_ros else (None, None)

    from isaacsim.core.prims import Articulation
    art = Articulation(prim_paths_expr=ROBOT_ROOT)
    # 未 initialize 前读 dof_names 会告警，这里给出默认顺序，初始化后再校正一次
    order = list(range(4))

    stop_attr = None
    scene = stage.GetPrimAtPath("/World/PhysicsScene")
    if mode == "kinematic" and scene.IsValid():
        from pxr import UsdPhysics
        stop_attr = UsdPhysics.Scene(scene).GetGravityMagnitudeAttr()
        _STATE["orig_gravity"] = stop_attr.Get()
        stop_attr.Set(0.0)
        _log("kinematic 模式：已临时关闭重力（停止时恢复）")

    try:
        saved_col = _set_robot_collision(False) if mode == "kinematic" else []
    except Exception as exc:
        saved_col = []
        _log("关闭碰撞失败（不影响运行）: %r" % (exc,))
    _log("kinematic 模式：已关闭 %d 个碰撞体；主保护是安全半径复位" % len(saved_col))

    _STATE.update(dict(saved_col=saved_col, safe_radius=float(safe_radius), resets=0,
                       art=art, order=order, pose=np.zeros(3), joint_q=np.zeros(4),
                       prev_t=None, started=False, ros=ros, mode=mode,
                       max=(max_vx, max_vy, max_wz), timeout=timeout,
                       sim_time_attr=sim_time_attr, cmd_lin=cmd_lin, cmd_ang=cmd_ang,
                       beat_attr=beat_attr, beat_seen=False, last_beat_val=None,
                       demo_idx=0, demo_t0=None, last_beat=None, last_beat_t=0.0))
    timeline = omni.timeline.get_timeline_interface()
    timeline.play()
    _log("开始播放。ros=%s mode=%s（演示序列共 %d 段）" % (ros, mode, len(DEMO)))
    if not ros:
        _log("演示模式：不需要 ROS，底盘会自己走。想用话题控制再执行 e06_gui_drive(ros=True)")
    else:
        _log("ROS 模式：指令持续有效；若外部同时发 /e06/heartbeat 则启用 %.1f s 超时停车" % timeout)
        _log("安全边界 %.2f m（到边界会限位；想开远一点：e06_gui_drive(ros=True, safe_radius=2.5)）"
             % safe_radius)
        _log("外部发布示例：")
        _log("  export ROS_DOMAIN_ID=%s" % (_os.environ.get("ROS_DOMAIN_ID") or "0"))
        _log("  ros2 topic pub --rate 20 /cmd_vel geometry_msgs/msg/Twist "
             "'{linear: {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'")

    stream = omni.kit.app.get_app().get_update_event_stream()
    _STATE["sub"] = stream.create_subscription_to_pop(_on_update, name="e06_gui_drive")
    return True


def _on_update(evt):
    st = _STATE
    if not st or st.get("art") is None:
        return
    st["dbg"] = st.get("dbg", 0) + 1
    timeline = omni.timeline.get_timeline_interface()
    if not timeline.is_playing():
        return
    og_t = 0.0
    try:
        og_t = float(st["sim_time_attr"].get())
    except Exception:
        og_t = 0.0
    # OG 时间与 /clock 同源；若它没被求值（例如 bridge 不可用），退回 timeline 时间
    t = max(og_t, float(timeline.get_current_time()))
    if st["dbg"] <= 3:
        _log("cb#%d sim_t=%.3f (og=%.3f)" % (st["dbg"], t, og_t))
    if t <= st.get("last_abs_t", 0.0):
        return
    st["last_abs_t"] = t
    if t <= 0.0:
        return
    if not st["started"]:
        st["art"].initialize()
        dn = list(st["art"].dof_names or [])
        if len(dn) == 4:
            st["order"] = [next((i for i, n in enumerate(dn) if n.startswith("base_wheel_%s" % k)), i)
                           for i, k in enumerate(R_I)]
        st["started"] = True
        _log("articulation 已初始化 (DOF=%d) 关节=%s 顺序=%s"
             % (st["art"].num_dof, dn, st["order"]))

    # ---- 取指令 ----
    if st["ros"]:
        try:
            lin = [float(v) for v in st["cmd_lin"].get()]
            ang = [float(v) for v in st["cmd_ang"].get()]
        except Exception:
            lin, ang = [0, 0, 0], [0, 0, 0]
        # 超时判定：桥接侧无法区分“值不变但仍在发”和“已停发”，
        # 因此只有收到过 /e06/heartbeat 时才启用超时；否则指令持续有效（便于 ros2 topic pub 调试）。
        hb = None
        if st.get("beat_attr") is not None:
            try:
                hb = float(st["beat_attr"].get()[0])
            except Exception:
                hb = None
        if hb is not None:
            if st.get("last_beat_val") is None:
                st["last_beat_val"] = hb        # 只记初始值，不算“收到心跳”
            elif hb != st["last_beat_val"]:
                st["last_beat_val"] = hb
                st["beat_seen"] = True
                st["last_beat_t"] = t
        if st["beat_seen"] and t - st["last_beat_t"] > st["timeout"]:
            lin, ang = [0, 0, 0], [0, 0, 0]
        vx, vy, wz = lin[0], lin[1], ang[2]
    else:
        if st["demo_t0"] is None:
            st["demo_t0"] = t
        name, dvx, dvy, dwz, hold = DEMO[st["demo_idx"] % len(DEMO)]
        if t - st["demo_t0"] >= hold:
            st["demo_idx"] += 1
            st["demo_t0"] = t
            name, dvx, dvy, dwz, hold = DEMO[st["demo_idx"] % len(DEMO)]
            _log("演示: %s" % name)
        vx, vy, wz = dvx, dvy, dwz

    mx, my, mw = st["max"]
    vx = max(-mx, min(mx, vx)); vy = max(-my, min(my, vy)); wz = max(-mw, min(mw, wz))

    # ---- 运动学积分 + 写回 ----
    dt = 0.0 if st["prev_t"] is None else max(0.0, min(0.2, t - st["prev_t"]))
    st["prev_t"] = t
    pose = st["pose"]
    c, s = math.cos(pose[2]), math.sin(pose[2])
    pose[0] += (vx * c - vy * s) * dt
    pose[1] += (vx * s + vy * c) * dt
    pose[2] += wz * dt
    # 安全区保护：运动学模式直接写位姿、不理会碰撞，开进箱体/墙会让 PhysX 连续报穿透错误
    # 并触发 “too many errors, simulation has been stopped”。两种兜底：
    #   演示模式（ros=False）—— 回到原点，保证循环干净；
    #   ROS 模式      —— 在边界限位（相当于隐形墙），不会把用户的车瞬移走。
    if math.hypot(pose[0], pose[1]) > st["safe_radius"]:
        if st["ros"]:
            pose[0], pose[1] = st.get("last_ok_xy", (0.0, 0.0))
            st["clamps"] = st.get("clamps", 0) + 1
            if st["clamps"] % 60 == 1:
                _log("已到安全边界 %.1f m，忽略该方向速度（第 %d 次）" % (st["safe_radius"], st["clamps"]))
        else:
            st["resets"] = st.get("resets", 0) + 1
            _log("超出安全半径 %.1f m（第 %d 次），复位到原点" % (st["safe_radius"], st["resets"]))
            pose[:] = 0.0
    else:
        st["last_ok_xy"] = (pose[0], pose[1])
    q = [math.cos(pose[2] / 2.0), 0.0, 0.0, math.sin(pose[2] / 2.0)]
    try:
        st["art"].set_world_poses(positions=np.array([[pose[0], pose[1], 0.0]]),
                                  orientations=np.array([q]))
        wheels = ik(vx, vy, wz)
        # 连续关节的角度必须回卷到 [-π, π)：PhysX 的 setDriveTarget 对 revolute 关节
        # 只接受 ±2π 以内的目标角，长时间累加会报
        # “only supports target angle in range [-2Pi, 2Pi]” 并最终让仿真停机。
        q_new = st["joint_q"] + np.array([wheels[k] for k in R_I]) * dt
        st["joint_q"] = (q_new + math.pi) % (2.0 * math.pi) - math.pi
        st["art"].set_joint_positions(
            np.array([[st["joint_q"][i] for i in st["order"]]]))
    except Exception as exc:  # pragma: no cover
        _log("驱动失败: %r" % (exc,))


def e06_gui_stop(reset=True):
    """停止驱动；reset=True 时把底盘放回原点并恢复重力。"""
    st = _STATE
    if st.get("sub") is not None:
        try:
            st["sub"] = None
        except Exception:
            pass
    if reset and st.get("art") is not None:
        try:
            st["art"].set_world_poses(positions=np.array([[0.0, 0.0, 0.0]]),
                                      orientations=np.array([[1.0, 0.0, 0.0, 0.0]]))
        except Exception:
            pass
    for attr, val in st.get("saved_col", []) or []:
        try:
            attr.Set(val)
        except Exception:
            pass
    if st.get("orig_gravity") is not None:
        stage = omni.usd.get_context().get_stage()
        scene = stage.GetPrimAtPath("/World/PhysicsScene")
        if scene.IsValid():
            from pxr import UsdPhysics
            UsdPhysics.Scene(scene).GetGravityMagnitudeAttr().Set(st["orig_gravity"])
    _STATE.clear()
    _log("已停止（reset=%s）" % reset)


# 直接 Run 本文件 = 进入演示模式
if __name__ == "__main__" or True:
    e06_gui_drive(ros=False)
