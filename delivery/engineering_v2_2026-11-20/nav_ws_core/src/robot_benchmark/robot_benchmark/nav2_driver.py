"""backend=nav2 的实跑客户端：把一个契约实例变成一个 run 的 ROS 侧驱动 + 采集。

口径（与 docs/接口契约.md v1.4、docs/基线调参日志.md §4 一致）：
  * 轨迹统一记在 **path frame**（契约坐标），/odom 给的是 map/world 系，这里按
    `world_poses[seed]` 反变换回来；judge() 也只接受 path frame。
  * 控制量记 `/cmd_vel`（安全层之后的实际下发），不是 controller 的原始输出——
    9-1 要求"平面合速度执行口径"对四个控制器同等生效，指标必须按实际执行量算。
  * 到达/超时/碰撞的判定**不在这里做**：这里只负责"跑"和"记"，判定交给 judge.py，
    避免驱动和评分两套口径。
  * 采集驱动用 Nav2 的 `compute_path_to_pose` + `follow_path` 两个 action，
    不经过 BT/behavior_server：基线实验不需要恢复行为，超时就是失败。

命令行自检（M2 套件已在线时）：
    python3 -m robot_benchmark.nav2_driver --scenario static_avoid --run 1
"""

from __future__ import annotations

import argparse
import math
import os
import sys
import time
from typing import Dict, List, Optional, Tuple

from .scenario_gen import default_manifest_path, find_instance

RCLPY = None


def _import_ros():
    """延迟导入 rclpy——纯离线场景（replay/dry-run、单元测试）不需要 ROS。"""
    global RCLPY
    if RCLPY is not None:
        return RCLPY
    import rclpy
    from rclpy.action import ActionClient
    from rclpy.node import Node
    from nav2_msgs.action import ComputePathToPose, FollowPath
    from nav_msgs.msg import Odometry
    from geometry_msgs.msg import Twist
    from std_msgs.msg import Bool
    RCLPY = dict(rclpy=rclpy, ActionClient=ActionClient, Node=Node,
                 ComputePathToPose=ComputePathToPose, FollowPath=FollowPath,
                 Odometry=Odometry, Twist=Twist, Bool=Bool)
    return RCLPY


# --------------------------------------------------------------------------- #
# 坐标变换：world(=map, 真值模式 map→odom 为固定恒等) ↔ path frame
# --------------------------------------------------------------------------- #
def world_to_path(x: float, y: float, yaw: float, world_pose: Dict) -> Tuple[float, float, float]:
    wx = float(world_pose.get("x", 0.0))
    wy = float(world_pose.get("y", 0.0))
    wyaw = float(world_pose.get("yaw", 0.0))
    dx, dy = x - wx, y - wy
    c, s = math.cos(-wyaw), math.sin(-wyaw)
    px, py = c * dx - s * dy, s * dx + c * dy
    return px, py, yaw - wyaw


def path_to_world(x: float, y: float, yaw: float, world_pose: Dict) -> Tuple[float, float, float]:
    wx = float(world_pose.get("x", 0.0))
    wy = float(world_pose.get("y", 0.0))
    wyaw = float(world_pose.get("yaw", 0.0))
    c, s = math.cos(wyaw), math.sin(wyaw)
    return c * x - s * y + wx, s * x + c * y + wy, yaw + wyaw


def _yaw_of(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class Recorder:
    """把 /odom 与限速后指令按时间戳配对成 (t, state, control) 三条等长序列。"""

    def __init__(self):
        self.t: List[float] = []
        self.state: List[List[float]] = []
        self.control: List[List[float]] = []
        self.last_cmd: Tuple[float, float, float] = (0.0, 0.0, 0.0)
        self.estops: List[float] = []
        self.cmd_samples = 0
        self.odom_samples = 0
        self.sim_now = 0.0            # 自己订阅 /clock 维护的仿真时间（不依赖 use_sim_time 是否生效）

    def on_odom(self, t: float, x: float, y: float, yaw: float, world_pose: Dict):
        px, py, pyaw = world_to_path(x, y, yaw, world_pose)
        self.t.append(t)
        self.state.append([px, py, pyaw])
        self.control.append([self.last_cmd[0], self.last_cmd[1], self.last_cmd[2]])
        self.odom_samples += 1

    def on_cmd(self, vx: float, vy: float, wz: float):
        self.last_cmd = (vx, vy, wz)
        self.cmd_samples += 1

    def on_clock(self, t: float):
        self.sim_now = t

    def on_estop(self, t: float, stopped: bool, was: bool):
        if stopped and not was:
            self.estops.append(t)

    def trajectory(self) -> Dict:
        return {"t": self.t, "state": self.state, "control": self.control}


class CostmapView:
    """读取 /global_costmap/costmap，用来判断"感知是否已经把障碍标进代价地图"。"""

    def __init__(self):
        self.grid = None
        self.res = 0.0
        self.ox = 0.0
        self.oy = 0.0
        self.w = 0
        self.h = 0

    def update(self, msg):
        self.res = float(msg.info.resolution)
        self.ox = float(msg.info.origin.position.x)
        self.oy = float(msg.info.origin.position.y)
        self.w = int(msg.info.width)
        self.h = int(msg.info.height)
        self.grid = list(msg.data)

    @property
    def ready(self) -> bool:
        return self.grid is not None

    def lethal_count(self) -> int:
        if self.grid is None:
            return 0
        return sum(1 for v in self.grid if v >= 99)

    def is_lethal(self, x: float, y: float) -> bool:
        if self.grid is None:
            return False
        ix = int((x - self.ox) / self.res)
        iy = int((y - self.oy) / self.res)
        if ix < 0 or iy < 0 or ix >= self.w or iy >= self.h:
            return True                     # 地图外一律视为不可通行（不能假装没事）
        return self.grid[iy * self.w + ix] >= 99

    def lethal_hits(self, poses) -> int:
        return sum(1 for p in poses if self.is_lethal(p[0], p[1]))

    def lethal_points(self):
        if self.grid is None:
            return []
        out = []
        w = self.w
        for i, v in enumerate(self.grid):
            if v >= 99:
                iy, ix = divmod(i, w)
                out.append((self.ox + (ix + 0.5) * self.res,
                            self.oy + (iy + 0.5) * self.res))
        return out


def scene_mismatch(inst: Dict, world_pose: Dict, lethal_pts, tol: float = 0.6) -> Dict:
    """检查"仿真里的场景"和"清单里的实例"是不是同一个。

    2026-11-14 踩过的坑：对齐仓里的 static_avoid_01.usd 是**旧版清单**生成的，障碍位置
    与 docs/scenarios/instances.yaml 不一致。机器人一路在躲"另一套场景"的障碍，judge 却
    按新清单算间距，结果反复出现"最小间距 1 m 却卡死"的怪现象，白查了三轮。
    判据：代价地图里的致命格如果远离清单几何（容差 tol），说明场景和清单不是一套。
    """
    def dist_to_contract(x: float, y: float) -> float:
        (px, py, _) = world_to_path(x, y, 0.0, world_pose)
        best = float("inf")
        for o in inst.get("obstacles") or []:
            d = math.hypot(px - o["x"], py - o["y"]) - float(o["r"])
            best = min(best, abs(d))
        for w in inst.get("walls") or []:
            dx = max(float(w["x0"]) - px, 0.0, px - float(w["x1"]))
            dy = max(float(w["y0"]) - py, 0.0, py - float(w["y1"]))
            best = min(best, math.hypot(dx, dy))
        return best if best != float("inf") else 0.0

    far = [p for p in lethal_pts if dist_to_contract(p[0], p[1]) > tol]
    n = len(lethal_pts)
    return {"lethal_cells": n, "off_contract_cells": len(far),
            "off_contract_frac": (len(far) / n) if n else 0.0}


def check_scene_identity(inst: Dict, scene_file) -> Dict:
    """精确校验：场景文件的 customLayerData 必须就是清单里的这个实例。

    为什么不用"代价地图致命格离清单几何多远"这种几何判据：代价地图里有 inflation
    （半径 0.55 m），窄走廊（净宽 1.46 m）两侧墙的 inflation 会互相叠加，把整条走廊
    都推过 99，于是几何判据会把正确的场景判成"100% 偏离"（2026-11-14 实测 narrow_passage）。
    场景文件是我们自己按清单写出来的文本 USD，直接读它的 scenario/seed 是最可靠的。
    """
    import re
    p = str(scene_file or "")
    if not p or not os.path.isfile(p):
        return {"checked": False, "reason": "未提供场景文件路径"}
    try:
        text = open(p, encoding="utf-8", errors="replace").read()
    except Exception as e:
        return {"checked": False, "reason": "读取失败：%s" % e}
    m_sc = re.search(r'string\s+scenario\s*=\s*"([^"]+)"', text)
    m_sd = re.search(r'int\s+seed\s*=\s*(\d+)', text)
    got_sc = m_sc.group(1) if m_sc else None
    got_sd = int(m_sd.group(1)) if m_sd else None
    ok = (got_sc == inst["scenario"]) and (got_sd == inst["seed"])
    return {"checked": True, "scene": os.path.basename(p), "scene_scenario": got_sc,
            "scene_seed": got_sd, "expect_scenario": inst["scenario"],
            "expect_seed": inst["seed"], "ok": bool(ok)}


def run_episode(inst: Dict,
                world_pose: Dict,
                timeout_s: float,
                *,
                controller_id: str = "",
                goal_checker_id: str = "general_goal_checker",
                planner_id: str = "",
                warmup_s: float = 40.0,
                min_lethal_cells: int = 0,      # 0=只用静态图占用格的一半做门限
                stable_s: float = 2.5,
                stable_tol: float = 0.02,
                replan_period_s: float = 1.5,
                max_replans: int = 30,
                path_deadline_s: float = 20.0,
                max_path_attempts: int = 5,
                scene_file=None,
                log=None) -> Dict:
    """跑一个 episode，返回 {trajectory, estop_times, diag}。调用方负责起栈与判分。"""
    ros = _import_ros()
    rclpy = ros["rclpy"]
    Node = ros["Node"]
    ActionClient = ros["ActionClient"]
    ComputePathToPose = ros["ComputePathToPose"]
    FollowPath = ros["FollowPath"]
    Odometry = ros["Odometry"]
    Twist = ros["Twist"]
    Bool = ros["Bool"]

    def say(msg):
        if log:
            log(msg)
        else:
            print(msg, flush=True)

    own_context = not rclpy.ok()
    if own_context:
        rclpy.init()

    node = Node("benchmark_nav2_driver")
    node.set_parameters([rclpy.parameter.Parameter("use_sim_time", value=True)])
    rec = Recorder()
    cm = CostmapView()
    state = {"estop_active": False, "result": None, "aborted": None,
             "safety_status": None}

    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
    from builtin_interfaces.msg import Time as builtin_interfaces_msg_Time
    from nav2_msgs.srv import ClearEntireCostmap
    from rosgraph_msgs.msg import Clock
    from nav_msgs.msg import OccupancyGrid
    from std_msgs.msg import String

    def odom_cb(msg):
        p = msg.pose.pose
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        rec.on_odom(t, p.position.x, p.position.y, _yaw_of(p.orientation), world_pose)
        ped_tick = ped_hooks.get("fn")
        if ped_tick is not None and (t - ped_hooks.get("last", 0.0)) >= 0.2:
            ped_hooks["last"] = t
            try:
                ped_tick()
            except Exception:
                pass

    def cmd_cb(msg):
        rec.on_cmd(msg.linear.x, msg.linear.y, msg.angular.z)

    def estop_cb(msg):
        rec.on_estop(rec.sim_now, bool(msg.data), state["estop_active"])
        state["estop_active"] = bool(msg.data)

    def clock_cb(msg):
        rec.on_clock(msg.clock.sec + msg.clock.nanosec * 1e-9)

    def costmap_cb(msg):
        cm.update(msg)

    def safety_cb(msg):
        state["safety_status"] = msg.data

    node.create_subscription(Odometry, "/odom", odom_cb, 50)
    # 控制量记 `/cmd_vel`（安全层之后的**实际下发**），不是限速层输出：
    # 安全层会把"碰撞临近/里程计失联/上游违规"的指令清零，指标必须按真正执行量算。
    node.create_subscription(Twist, "/cmd_vel", cmd_cb, 50)
    node.create_subscription(Bool, "/safety/stop", estop_cb, 10)
    # 安全层诊断：哪几条安全条件触发过、触发多少次（落 run.json 的 diag，便于追责与复现）
    node.create_subscription(String, "/safety/status", safety_cb, 10)
    node.create_subscription(Clock, "/clock", clock_cb, 10)
    node.create_subscription(
        OccupancyGrid, "/global_costmap/costmap", costmap_cb,
        QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                   durability=DurabilityPolicy.TRANSIENT_LOCAL,
                   history=HistoryPolicy.KEEP_LAST))
    # 行人 → 动态障碍物：走自研 costmap 层 `robot_bringup::PedCorridorLayer`
    # （nav2_common.yaml 里挂在局部代价地图的 voxel 之后、inflation 之前）。
    # 为什么不用 Nav2 的 static_layer 补丁通道（/map_updates）：2026-11-17 隔离实验证实
    # 那条通道在本环境是死的——按官方类型 map_msgs/OccupancyGridUpdate 发、QoS 与订阅端
    # 一致，局部与全局代价地图都毫无反应（写 100 的格子读回来还是 0）。
    upd_pub = node.create_publisher(
        OccupancyGrid, "/ped/corridor",
        QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                   durability=DurabilityPolicy.VOLATILE,
                   history=HistoryPolicy.KEEP_LAST))
    ped_hooks = {"fn": None, "last": 0.0}
    reset_pub = node.create_publisher(Twist, "/sim/reset", 10)
    # 本 run 的 t=0：行人相位锚点（见 isaac_sim_bridge.py 的 /sim/episode_start）
    start_pub = node.create_publisher(Twist, "/sim/episode_start", 10)
    map_pub = node.create_publisher(
        OccupancyGrid, "/map",
        QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                   durability=DurabilityPolicy.TRANSIENT_LOCAL,
                   history=HistoryPolicy.KEEP_LAST))
    clear_local = node.create_client(ClearEntireCostmap,
                                     "/local_costmap/clear_entirely_local_costmap")
    clear_global = node.create_client(ClearEntireCostmap,
                                      "/global_costmap/clear_entirely_global_costmap")
    planner = ActionClient(node, ComputePathToPose, "/compute_path_to_pose")
    follower = ActionClient(node, FollowPath, "/follow_path")

    def spin(seconds: float):
        end = time.monotonic() + seconds
        while time.monotonic() < end and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)

    def spin_until(pred, limit_s: float) -> bool:
        end = time.monotonic() + limit_s
        while time.monotonic() < end and rclpy.ok():
            if pred():
                return True
            rclpy.spin_once(node, timeout_sec=0.05)
        return pred()

    def now_msg():
        """用自己订阅的 /clock 生成消息时间戳（不赌 use_sim_time 是否生效）。"""
        sec = int(rec.sim_now)
        return builtin_interfaces_msg_Time(sec=sec, nanosec=int((rec.sim_now - sec) * 1e9))

    def spin_until_stable(limit_s: float, stable_s: float, min_cells: int,
                          tol: float, counter) -> bool:
        """等到某个计数(>=min_cells)在 stable_s 内相对变化小于 tol。"""
        end = time.monotonic() + limit_s
        last = counter()
        last_change = time.monotonic()
        while time.monotonic() < end and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.4)
            cur = counter()
            if cur and cur >= min_cells and last and abs(cur - last) <= tol * max(1, last):
                if time.monotonic() - last_change >= stable_s:
                    return True
            else:
                last_change = time.monotonic()
            last = cur
        return False

    def verify_single_stack(wait_s: float = 12.0) -> Dict:
        """确保"只有一个 M2 栈"在跑。

        2026-11-14 踩过的坑：前面几次 `ros2 launch` 的孤儿进程没清干净，结果同时有
        2 个 flat_speed_limiter、2 个安全层、3 个 Isaac。多个节点抢着发 `/cmd_vel`
        （互相把对方的指令清成 0），机器人一动不动，日志里只看到
        "Failed to make progress"，非常难查。这里把"单栈"做成硬不变量。
        """
        deadline = time.monotonic() + wait_s
        counts = {}
        # 话题发布者 + action 服务器数量都要是 1。只查话题是不够的：2026-11-14 实测出现过
        # 栈叠起来但 /cmd_vel、/clock、/odom 各只有 1 个发布者（旧栈的节点被 launch 杀掉、
        # 但 planner_server 存活），driver 收到
        # "more than one action server for the action '/compute_path_to_pose'" 后拿错结果。
        topics = ("/cmd_vel", "/clock", "/odom",
                  "/compute_path_to_pose/_action/status", "/follow_path/_action/status")
        if "mpc" in (controller_id or ""):
            # 算法侧 G5：MPC 的动态障碍输入必须**恰好 1 个发布者**
            # （要么 launch 的空数组心跳，要么 obstacle_feeder.py；两者同时在场 = 脏数据）
            topics = topics + ("/mpc/obstacles",)
        while time.monotonic() < deadline and rclpy.ok():
            counts = {t: len(node.get_publishers_info_by_topic(t)) for t in topics}
            if all(v >= 1 for v in counts.values()):
                break
            rclpy.spin_once(node, timeout_sec=0.2)
        bad = {t: c for t, c in counts.items() if c != 1}
        if bad:
            raise RuntimeError(
                "M2 套件不唯一或未就绪：%s（每个话题必须恰好 1 个发布者）。"
                "请先清理残留进程：pkill -f isaac_sim_bridge.py / flat_speed_limiter.py / "
                "safety_supervisor.py / controller_server / planner_server"
                % ", ".join("%s=%d" % kv for kv in bad.items()))
        return counts

    diag = {"path_attempts": 0, "path_len_m": None, "follow_result": None,
            "warmup_odom": 0, "warmup_cmd": 0, "replans": 0, "replan_failed": 0}

    # ---- 自研 MPC 诊断（E-P0-6 / G7）：类型来自 ASCII 镜像构建的 mpc_controller，
    #      没有该包时静默跳过（三个基线的行为完全不变）。
    #      采集内容：逐拍 solve_ms（→ run.json 的 qp_timing）与最大连续回退拍数
    #      （→ 连续 5 拍判 solver_fail，见 runner.remap_failure）。
    mpc_diag = {"solve_ms": [], "max_streak": 0, "cur_streak": 0, "fallback_frames": 0}
    try:
        from mpc_controller.msg import MpcDiagnostics as _MpcDiag

        def _on_mpc_diag(m):
            try:
                if getattr(m, "solve_ms", None) is not None:
                    mpc_diag["solve_ms"].append(float(m.solve_ms))
                fb = bool(getattr(m, "fallback", False))
                streak = int(getattr(m, "fallback_streak", 0) or 0)
                if fb:
                    mpc_diag["fallback_frames"] += 1
                mpc_diag["cur_streak"] = streak if fb else 0
                mpc_diag["max_streak"] = max(mpc_diag["max_streak"], streak, mpc_diag["cur_streak"])
            except Exception:
                pass

        node.create_subscription(_MpcDiag, "/mpc/diagnostics", _on_mpc_diag, 50)
        say("[nav2] 已订阅 /mpc/diagnostics（MPC 计时/回退诊断）")
    except Exception:
        pass

    try:
        # 0) 单栈不变量：/cmd_vel(安全层) /clock(桥接) /odom(真值适配) 各恰好 1 个发布者
        cnt = verify_single_stack()
        diag["publisher_counts"] = cnt
        say("[nav2] 单栈检查通过：%s" % cnt)

        # 1) 等栈就绪：action server 上线 + /odom、/cmd_vel（安全层之后）真的在流动
        say("[nav2] 等待 /compute_path_to_pose 与 /follow_path ...")
        if not planner.wait_for_server(timeout_sec=path_deadline_s):
            raise RuntimeError("planner_server 的 /compute_path_to_pose 未上线")
        if not follower.wait_for_server(timeout_sec=path_deadline_s):
            raise RuntimeError("controller_server 的 /follow_path 未上线")
        got = spin_until(lambda: rec.odom_samples > 3 and rec.cmd_samples > 0, path_deadline_s)
        diag["warmup_odom"] = rec.odom_samples
        diag["warmup_cmd"] = rec.cmd_samples
        if not got:
            raise RuntimeError("等 /odom 与 /cmd_vel 超时（odom=%d cmd=%d）"
                               % (rec.odom_samples, rec.cmd_samples))

        # 2) 复位到底盘起始位姿（每个 run 必须同起点；靠重开 Isaac 太慢，会污染对比）
        #    约定见 isaac_sim_bridge.py：/sim/reset 上发 Twist，linear.x>=0.5 = 请求复位
        say("[nav2] 发 /sim/reset 复位到场景起点")
        _rst = Twist()
        _rst.linear.x = 1.0
        reset_pub.publish(_rst)
        spin(0.8)
        reset_pub.publish(Twist())
        spin(1.2)

        # 2a) 复位会让里程计跳变，局部代价地图（rolling window 跟着 odom 走）里会残留
        #     上一轮位置的"幽灵障碍"，机器人一启动就被围死（实测：停在原地 60 s、
        #     最小间距 2.79 m 却 v≈0）。复位后必须把两张代价地图清掉再重新感知。
        def clear_costmap(client, name):
            if not client.wait_for_service(timeout_sec=5.0):
                say("[nav2] 警告：%s 清图服务未上线" % name)
                return False
            done = {"ok": None}

            def cb(fut):
                done["ok"] = fut.result() is not None
            client.call_async(ClearEntireCostmap.Request()).add_done_callback(cb)
            spin_until(lambda: done["ok"] is not None, 5.0)
            return bool(done["ok"])

        # 复位校验：/sim/reset 之后机器人必须真的回到起点附近。
        # 2026-11-15 实测漏了一条脏数据：机器人上一轮停在终点、复位没生效，本轮
        # "到达时间 = 0.0 s" 直接判成功（间距 6 m）。这种数据必须当场拦掉。
        if not spin_until(lambda: bool(rec.state), 5.0):
            raise RuntimeError("复位后没有 /odom 样本")
        home = world_to_path(*path_to_world(0.0, 0.0, 0.0, world_pose), world_pose) \
            if False else (0.0, 0.0)
        _d = math.hypot(rec.state[-1][0] - home[0], rec.state[-1][1] - home[1])
        diag["reset_offset_m"] = round(_d, 4)
        if _d > 0.6:
            raise RuntimeError(
                "复位没生效：机器人仍在 (%.2f, %.2f)，离起点 %.2f m（>0.6 m）。"
                "本轮数据无效，请检查 /sim/reset 通道与场景起点。"
                % (rec.state[-1][0], rec.state[-1][1], _d))

        ok_l = clear_costmap(clear_local, "local_costmap")
        ok_g = clear_costmap(clear_global, "global_costmap")
        say("[nav2] 代价地图清除：local=%s global=%s" % (ok_l, ok_g))
        diag["reset"] = True
        diag["costmap_cleared"] = bool(ok_l and ok_g)

        # 2b) 发布评分用静态地图（契约清单几何 → /map → costmap 的 static_layer）
        #     如果外部已经有 /map 发布者（launch 里用 map:= 起了 nav2_map_server），就让位，
        #     避免同一话题两个发布者互相覆盖。
        _ext_map = len(node.get_publishers_info_by_topic("/map")) > 0
        diag["external_map_publisher"] = _ext_map
        from .scenes import build_occupancy
        occ = build_occupancy(inst, world_pose)
        grid = OccupancyGrid()
        grid.header.frame_id = "map"
        grid.header.stamp = now_msg()
        grid.info.resolution = float(occ["resolution"])
        grid.info.width = int(occ["width"])
        grid.info.height = int(occ["height"])
        grid.info.origin.position.x = float(occ["origin_x"])
        grid.info.origin.position.y = float(occ["origin_y"])
        grid.info.origin.orientation.w = 1.0
        grid.data = occ["data"]
        n_static = sum(1 for v in occ["data"] if v)
        # 不能只发几次就指望送达：800x800 的地图约 640 KB，transient-local 大样本对
        # "晚加入的订阅者"偶发丢失（实测 narrow_passage 有一轮代价地图一直报
        # "Can't update static costmap layer, no map received"，整条 run 作废）。
        # 改成 1 Hz 持续重发，直到本次 episode 结束；幂等，代价约 0.6 MB/s。
        if not _ext_map:
            for _ in range(3):
                map_pub.publish(grid)
                spin(0.3)
        _map_timer = None if _ext_map else node.create_timer(1.0, lambda: map_pub.publish(grid))

        # 行人 → 动态障碍：每 5 Hz 发一个 patch，把行人当前位置标 100、把上一拍位置还原。
        # patch 内的其它格子一律取**静态地图真值**，所以不会误擦墙。
        peds = inst.get("pedestrians") or []
        ped_state = {"prev": [], "prev_rects": [], "n": 0}

        # 预测时域：契约公式是匀速直线，所以"未来"是可算的。把行人未来
        # `PED_PREDICT_S` 秒的扫掠带一起标成占用，等效于动态障碍的轨迹预测
        # （TEB 原生做法是假设匀速直线；这里让三个基线共享同一份预测信息，公平）。
        #
        # **为什么是"横穿走廊"而不是"按当前位置画圆"**：2026-11-16 实测（seed 203，
        # 探针 /tmp/probe2.py）——只标当前位置/未来位置的圆盘时，机器人会在走廊边缘
        # 停下来，而那个停车点距行人的横穿线只有 0.06 m：行人照直走过去，直接穿过
        # 机器人本体（穿透 -0.298 m）。要真正避免，占用的**法向**宽度必须覆盖
        # "底盘外接半径 + 行人半径"，否则机器人会以为自己站得够边上。
        # 半宽 = 0.343（八边形外接半径）+ 0.30（行人半径）+ 0.10（余量）= 0.74 m。
        # 纵向只覆盖行人的"当前 → 未来 PED_PREDICT_S 秒"，所以行人离开马路后
        # 走廊自然让开，机器人不会永久等待。
        # 预测时域 = 走廊在横穿线前"提前封路"的时间，同时也是"行人过去后多久解封"。
        # 两头都要卡住：
        #   - 2.0 s（1.86 m）封 ±2 s：机器人来得及挤进去 → 实测穿透（-0.299 m）；
        #   - 2.5 s（2.3 m）封 ±2.5 s：还是能挤进去 → 实测 3/9 仍穿透；
        #   - 4.0 s（3.7 m）封 ≈4.8 s：安全（0 穿透），但要求 controller 耐心 ≥5 s。
        # 所以本文件取 4.0 s，并把 controller_server 的 failure_tolerance 提到 10 s
        # （见 nav2_baseline.launch.py 的 FAILURE_TOLERANCE），一次等待约 4.8 s 能熬过去。
        # 2026-11-17 定稿：局部点云层不再标记行人（见 nav2_common.yaml），行人的全部信息
        # 都由这条走廊提供，所以走廊必须"自己就能把人挡住"：
        #   * 向前预测 7 s（≈6.3 m）+ 身后 2 m，几乎覆盖整条马路宽度 → 机器人无法从旁边
        #     挤过去，只能停车等（实测 4 s 时 DWA/MPPI 都从旁边绕过，穿透 −0.30）；
        #   * 法向半宽 0.74 m 保证底盘不会贴在横穿线上（见 §2.3）。
        PED_PREDICT_S = 7.0        # 兼容保留
        PED_BACK_M = 2.0           # 兼容保留
        PED_GATE_ETA_S = 6.0       # 行人还有多久会进入马路窗口就提前封路
        PED_GATE_MERGE_M = 2.0     # 相邻横穿线间距小于该值就合并成一个道口
        PED_ROAD_HALF_Y = 4.2      # 马路窗口半宽（行人|y|≤4.2 视为"在路口范围内"）
        PED_HALF_W = 0.343 + 0.30 + 0.10

        def ped_path_xy(ped, tp):
            """契约公式（path frame）：x=x_cross，y=dir·speed·(tp−t_meet)。"""
            lx = float(ped["x_cross"])
            ly = float(ped["dir"]) * float(ped["speed"]) * (tp - float(ped["t_meet"]))
            return lx, ly

        def ped_map_xy(ped, tp):
            lx, ly = ped_path_xy(ped, tp)
            return path_to_world(lx, ly, 0.0, world_pose)[:2]

        _wp = float(world_pose.get("yaw", 0.0))
        _wc, _ws = math.cos(_wp), math.sin(_wp)
        _wx0 = float(world_pose.get("x", 0.0))
        _wy0 = float(world_pose.get("y", 0.0))

        def map_to_path_xy(wx2, wy):
            """map frame → path frame（求逆变换，用来判断某格是否落在走廊里）。"""
            dx = wx2 - _wx0
            dy = wy - _wy0
            return _wc * dx + _ws * dy, -_ws * dx + _wc * dy

        def corridors(tp):
            """道口栏杆（path frame 轴对齐矩形）。

            与旧版的三个差别（对应 docs/9-3行人动态障碍.md §10.3）：
            1. **合并近距离横穿线**：相邻两条横穿线间距 < PED_GATE_MERGE_M 时视为同一个道口，
               用 [min(x_cross) ± HALF_W, max(x_cross) ± HALF_W] 整条带宽——否则两条栏杆
               各自开合会出现"第一条抬了、第二条还没落"的缝（mixed seed 501 两线只隔 1.0 m、
               502 隔 1.6 m，正是撞人的那两个种子）。
            2. **组内任一行人没清空就不抬杆**：合并后的栏杆要等组里所有人都离开才放行。
            3. **"即将进入"才算提前量，"离开"要留身后余量**：旧版用未来位置做重叠判断，
               提前量越大反而越早抬杆（预测位置先离开窗口），语义是反的。
               现在：`已进入或身后余量内` OR `PED_GATE_ETA_S 内会进入` → 封路。
            """
            act = []
            for p in peds:
                lx, ly = ped_path_xy(p, tp)
                ly_future = ped_path_xy(p, tp + PED_GATE_ETA_S)[1]
                inside = abs(ly) <= PED_ROAD_HALF_Y + PED_BACK_M
                approaching = (max(ly, ly_future) >= -PED_ROAD_HALF_Y) and \
                              (min(ly, ly_future) <= PED_ROAD_HALF_Y)
                act.append((lx, bool(inside or approaching)))
            act.sort(key=lambda t: t[0])

            groups = []
            for lx, active in act:
                if groups and (lx - groups[-1]["xmax"]) < PED_GATE_MERGE_M:
                    groups[-1]["xmax"] = lx
                    groups[-1]["any"] = groups[-1]["any"] or active
                else:
                    groups.append({"xmin": lx, "xmax": lx, "any": active})

            robot_px = rec.state[-1][0] if rec.state else None
            out = []
            for g in groups:
                if not g["any"]:
                    continue
                x0 = g["xmin"] - PED_HALF_W
                x1 = g["xmax"] + PED_HALF_W
                # **栏杆不落在机器人身上**：机器人已经进到带子里时不再落杆。
                # 否则它被 LETHAL 包住，DWA/TEB 的每条轨迹都判撞，只能等
                # `patience` 中止（实测 mixed seed 501/TEB 停在 x=4.94 达 21 s）。
                # 代价是"已经进场就别再关门"，宁可让它快速通过也不要卡死。
                if robot_px is not None and (x0 - 0.15) <= robot_px <= (x1 + 0.15):
                    ped_state["skip_inside"] = ped_state.get("skip_inside", 0) + 1
                    continue
                out.append((x0, -PED_ROAD_HALF_Y - PED_HALF_W,
                            x1, PED_ROAD_HALF_Y + PED_HALF_W))
            return out

        def corridor_map_bbox(rect):
            x0, y0, x1, y1 = rect
            corners = [path_to_world(x0, y0, 0.0, world_pose)[:2],
                       path_to_world(x1, y0, 0.0, world_pose)[:2],
                       path_to_world(x0, y1, 0.0, world_pose)[:2],
                       path_to_world(x1, y1, 0.0, world_pose)[:2]]
            xs = [c[0] for c in corners]; ys = [c[1] for c in corners]
            return min(xs), min(ys), max(xs), max(ys)

        def publish_ped_patch():
            if not peds:
                return
            anchor = diag.get("episode_anchor_sim")
            tp = 0.0 if anchor is None else max(0.0, rec.sim_now - anchor)
            res = occ["resolution"]; n = occ["width"]
            ox = occ["origin_x"]; oy = occ["origin_y"]
            rects = corridors(tp)
            cur = [ped_map_xy(p, tp) for p in peds]
            rs = [float(p.get("r", 0.3)) + 0.10 for p in peds]
            # 本拍需要重绘的区域 = 上一拍走廊/圆盘 ∪ 本拍走廊/圆盘
            boxes = [corridor_map_bbox(r) for r in rects]
            for r in ped_state.get("prev_rects", []):
                boxes.append(r)
            if not boxes:
                # **必须发一个空的补丁**：否则代价地图层会一直持有上一帧的走廊栅格，
                # 每拍把已经该放行的走廊重新写回去（实测 mixed seed 501：两个行人都已经
                # 走远、栏杆早该抬起，代价地图里却一直保持 lethal，机器人最后硬撞过去）。
                empty = OccupancyGrid()
                empty.header.frame_id = "map"
                empty.header.stamp = now_msg()
                empty.info.resolution = float(occ["resolution"])
                empty.info.width = 1
                empty.info.height = 1
                empty.info.origin.position.x = float(occ["origin_x"])
                empty.info.origin.position.y = float(occ["origin_y"])
                empty.info.origin.orientation.w = 1.0
                empty.data = [0]
                upd_pub.publish(empty)
                ped_state["prev"] = []
                ped_state["prev_rects"] = []
                ped_state["n"] += 1
                return
            x0 = min(b[0] for b in boxes); y0 = min(b[1] for b in boxes)
            x1 = max(b[2] for b in boxes); y1 = max(b[3] for b in boxes)
            ix0 = max(0, int((x0 - ox) / res) - 1); iy0 = max(0, int((y0 - oy) / res) - 1)
            ix1 = min(n - 1, int((x1 - ox) / res) + 1); iy1 = min(n - 1, int((y1 - oy) / res) + 1)
            w = ix1 - ix0 + 1; h = iy1 - iy0 + 1
            if w <= 0 or h <= 0:
                return
            patch = OccupancyGrid()
            patch.header.frame_id = "map"
            patch.header.stamp = now_msg()
            patch.info.resolution = float(res)
            patch.info.width = int(w); patch.info.height = int(h)
            patch.info.origin.position.x = float(ox + ix0 * res)
            patch.info.origin.position.y = float(oy + iy0 * res)
            patch.info.origin.orientation.w = 1.0
            # **只发栏杆带本身**（纯 gate 语义）。
            # 旧版沿用 /map_updates 的"重绘补丁"语义：把静态地图占用格（100）与每个行人的
            # 0.4 m 圆盘一起标进补丁，补丁 bbox 还随行人走到 |y|>20 m 而膨胀（实测出现
            # 52x800 的补丁、原点 y 从 -20 漂到 -5）。消费端 ped_corridor_layer 会把这些
            # 100 一律写成 LETHAL，并在下一拍把"上一拍写过的格子"恢复成 FREE——
            # 于是静态墙在 FREE/LETHAL 之间来回抖，代价地图自相矛盾，机器人卡在栏杆带里
            # （mixed seed 502：停在 x=7.4 二十多秒不动）。
            # 现在：仅 `rects` 内的格子为 100，其余 0；行人的"身体"由雷达口径负责，
            # 走廊口径只表达"道口栏杆"这一抽象（见 docs/9-3行人动态障碍.md §8.2、§11.6）。
            data = []
            for jy in range(h):
                wy = oy + (iy0 + jy + 0.5) * res
                for jx in range(w):
                    wx2 = ox + (ix0 + jx + 0.5) * res
                    px2, py2 = map_to_path_xy(wx2, wy)
                    v = 0
                    for (x0, y0, x1, y1) in rects:
                        if x0 <= px2 <= x1 and y0 <= py2 <= y1:
                            v = 100
                            break
                    data.append(v)
            patch.data = data
            upd_pub.publish(patch)
            ped_state["prev"] = [(px, py, r) for (px, py), r in zip(cur, rs)]
            ped_state["prev_rects"] = [corridor_map_bbox(r) for r in rects]
            ped_state["n"] += 1

        # 注意：不要用 create_timer 驱动。实测在跟随阶段的 spin 节拍下定时器会被饿死，
        # 导致 /map_updates 一直停在"t=0 的静态位置"，等于行人没动过。
        # 改成从 /odom 回调里按最小间隔触发（odom 60 Hz，稳定且与仿真时间同步）。
        ped_hooks["fn"] = publish_ped_patch if peds else None
        diag["pedestrian_dynamic_obstacle"] = bool(peds)
        say("[nav2] 已发布评分静态地图 /map：%dx%d res=%.2f 占用格=%d"
            % (occ["width"], occ["height"], occ["resolution"], n_static))
        diag["static_cells"] = n_static
        # static_layer 生效后，全局代价地图的致命格应与静态图占用格同量级
        # （实测 983/1102、1015/1529；差值是栅格化、inflation 覆盖与 voxel 清图造成，故取 0.5 倍下限），用它当门限就同时验证了
        # "静态图真的进了代价地图"。
        min_lethal_cells = max(min_lethal_cells, int(0.5 * n_static), 1)

        # 2c) 等感知把障碍真正标进代价地图，并且**标完稳定下来**再规划。
        #     2026-11-14 实测两轮：只等 4 s 或只要求"致命格≥800"时，全局代价地图还在
        #     增长（2965 → 5791 格），Smac 会规划出一条穿过"当时还没标上"的障碍的
        #     10.35 m 近似直线（目标直线距离 10.08 m）；机器人开到该处后 DWB 正确地
        #     拒绝前进 → 卡死在 x≈4.5。判据必须看"稳定"，不是看"够多"。
        say("[nav2] 等全局代价地图标记稳定（≥%d 格且 %.1f s 内变化 <%.0f%%，最长 %.0f s）"
            % (min_lethal_cells, stable_s, 100.0 * stable_tol, warmup_s))
        ok = spin_until_stable(warmup_s, stable_s, min_lethal_cells, stable_tol,
                               lambda: cm.ready and cm.lethal_count())
        if not ok:
            # 偶发：/map 没被 static_layer 收下（厚墙场景实测只标到 602/3338 格）。
            # 重发一次并再等一轮，避免整条 run 因基础设施问题作废。
            say("[nav2] 静态层疑似未生效（致命格 %d < %d）%s"
                % (cm.lethal_count(), min_lethal_cells,
                   "；/map 由外部 map_server 发布，等它" if _ext_map else "，重发 /map 再等一轮"))
            if not _ext_map:
                for _ in range(3):
                    map_pub.publish(grid)
                    spin(0.4)
            ok = spin_until_stable(warmup_s, stable_s, min_lethal_cells, stable_tol,
                                   lambda: cm.ready and cm.lethal_count())
        say("[nav2] 代价地图：ready=%s 致命格=%d" % (cm.ready, cm.lethal_count()))
        if not ok:
            raise RuntimeError("等代价地图标记稳定超时：致命格 %d（阈值 %d）"
                               % (cm.lethal_count(), min_lethal_cells))
        spin(1.0)                       # 再等一个 2 Hz 更新周期，避免刚好卡在边界

        # 2d) 硬校验：仿真场景必须就是清单里的那个实例，否则成绩没有意义
        ident = check_scene_identity(inst, scene_file)
        diag["scene_identity"] = ident
        if ident.get("checked") and not ident.get("ok"):
            raise RuntimeError(
                "仿真场景与实例不匹配：场景 %s 是 (%s, seed=%s)，而本次要评的是 (%s, seed=%s)"
                % (ident["scene"], ident["scene_scenario"], ident["scene_seed"],
                   ident["expect_scenario"], ident["expect_seed"]))
        if not ident.get("checked"):
            say("[nav2] 警告：未做场景身份校验（%s）；请在 runner 里用 --scene 传场景文件"
                % ident.get("reason"))

        # 2e) 几何一致性只做记录、不做拦截（窄走廊的 inflation 会把判据打爆，见
        #     check_scene_identity 的注释），供事后复核用
        mm = scene_mismatch(inst, world_pose, cm.lethal_points())
        diag["scene_geometry"] = mm
        say("[nav2] 几何一致性（仅记录）：致命格 %d，其中离清单几何 >0.6 m 的占 %.1f%%"
            % (mm["lethal_cells"], 100.0 * mm["off_contract_frac"]))
        if not rec.state:
            raise RuntimeError("预热后没有 /odom 样本")
        # 只取起点；轨迹的清空挪到"开始跟踪"那一刻（与行人相位锚点同一时刻）。
        # 否则规划重试的时间会被算进轨迹，导致 judge 的 t=0 与行人时钟错位
        # （2026-11-14 实测：错位 18.5 s，负到达时间 + 假碰撞）。
        start_path = tuple(rec.state[-1])
        start_map = path_to_world(start_path[0], start_path[1], start_path[2], world_pose)

        # 3) 终点换成 world(=map) 系再发 action
        goal_path = (float(inst["goal"]["x"]), float(inst["goal"].get("y", 0.0)),
                     float(inst["goal"].get("yaw", 0.0)))
        goal_map = path_to_world(*goal_path, world_pose)

        def plan_from(start) -> Optional[object]:
            """从 start（map 系 x,y,yaw）规划到目标；穿过致命格的路径一律不采用。"""
            for attempt in range(1, max_path_attempts + 1):
                diag["path_attempts"] += 1
                goal = ComputePathToPose.Goal()
                goal.goal.header.frame_id = "map"
                goal.goal.header.stamp = now_msg()
                goal.goal.pose.position.x = goal_map[0]
                goal.goal.pose.position.y = goal_map[1]
                goal.goal.pose.orientation.z = math.sin(goal_map[2] / 2.0)
                goal.goal.pose.orientation.w = math.cos(goal_map[2] / 2.0)
                goal.start.header.frame_id = "map"
                goal.start.header.stamp = now_msg()
                goal.start.pose.position.x = start[0]
                goal.start.pose.position.y = start[1]
                goal.start.pose.orientation.z = math.sin(start[2] / 2.0)
                goal.start.pose.orientation.w = math.cos(start[2] / 2.0)
                goal.planner_id = planner_id
                goal.use_start = True
                send = planner.send_goal_async(goal)
                spin_until(lambda: send.done(), path_deadline_s)
                h = send.result() if send.done() else None
                if h is None or not h.accepted:
                    say("[nav2] 规划请求被拒（attempt %d）" % attempt)
                    spin(1.0)
                    continue
                rf = h.get_result_async()
                spin_until(lambda: rf.done(), path_deadline_s)
                if not rf.done():
                    say("[nav2] 规划超时（attempt %d）" % attempt)
                    spin(1.0)
                    continue
                result = rf.result().result
                if not result.path.poses:
                    say("[nav2] 规划返回空路径（attempt %d），1 s 后重试" % attempt)
                    spin(1.0)
                    continue
                hits = cm.lethal_hits([(p.pose.position.x, p.pose.position.y)
                                       for p in result.path.poses])
                if hits > 0:
                    # 路径穿过致命格 → 这条路径不能用来跑。宁可重规划，也不允许
                    # "穿过障碍再撞上去"的假数据。
                    say("[nav2] 路径穿过 %d 个致命格（attempt %d），1 s 后重规划" % (hits, attempt))
                    spin(1.0)
                    continue
                return result.path
            return None

        path_msg = plan_from(start_map)
        if path_msg is None:
            raise RuntimeError("全局规划连续 %d 次失败" % max_path_attempts)
        diag["path_len_m"] = round(_path_length(path_msg), 3)
        say("[nav2] 规划成功：%d 个路点，长度 %.2f m" % (len(path_msg.poses), diag["path_len_m"]))

        # 4) 跟踪 + 按需重规划
        #    为什么必须重规划：全局代价地图是**随车前进逐步补全**的（起步时只有 2965 个
        #    致命格，跑完时 5791 个）。一次性路径会随着新障碍被标上而过期，DWB 随后
        #    正确地拒绝前进 → 机器人停住直到 progress_checker 报失败。真实 Nav2 里这一步
        #    由 BT 的 recovery/重规划承担；这里绕过了 BT，就必须自己补。
        def start_follow(path):
            fp = FollowPath.Goal()
            fp.path = path
            fp.controller_id = controller_id
            fp.goal_checker_id = goal_checker_id
            s = follower.send_goal_async(fp)
            spin_until(lambda: s.done(), path_deadline_s)
            hh = s.result() if s.done() else None
            if hh is None or not hh.accepted:
                raise RuntimeError("controller_server 拒绝了 follow_path 请求")
            return hh, hh.get_result_async()

        # 3c) 标记本 run 的 t=0（行人相位锚点），紧贴"开始跟踪"这一刻发。
        #     为什么放这么晚：judge 用的相对时间起点必须和仿真侧的行人时钟对齐。
        #     2026-11-14 实测：锚点提前 3.5 s 发布时，判分会把"已经过马路的行人"算成
        #     "正要撞上机器人"，三个基线在同样种子上一起出现 clearance≈-0.29 m 的假碰撞。
        rec.t.clear(); rec.state.clear(); rec.control.clear()   # 预热与规划段不计入轨迹
        _st = Twist()
        _st.linear.x = 1.0
        start_pub.publish(_st)
        spin(0.2)
        start_pub.publish(Twist())
        spin(0.1)
        diag["episode_anchor_sim"] = round(rec.sim_now, 4)

        handle, res_future = start_follow(path_msg)

        t0 = rec.sim_now                  # 用自己订阅的 /clock（不赌 use_sim_time 生效）
        wall0 = time.monotonic()
        next_check = 0.0
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.02)
            if res_future.done():
                diag["follow_result"] = int(res_future.result().status)
                break
            if rec.sim_now - t0 > timeout_s:
                state["aborted"] = "timeout"
                handle.cancel_goal_async()
                break
            if time.monotonic() - wall0 > timeout_s * 4.0 + 60.0:
                state["aborted"] = "wall_clock"
                handle.cancel_goal_async()
                break
            if time.monotonic() - next_check < replan_period_s or not rec.state:
                continue
            next_check = time.monotonic()
            cur = path_to_world(rec.state[-1][0], rec.state[-1][1], rec.state[-1][2], world_pose)
            poses = [(p.pose.position.x, p.pose.position.y) for p in path_msg.poses]
            k = min(range(len(poses)),
                    key=lambda i: (poses[i][0] - cur[0]) ** 2 + (poses[i][1] - cur[1]) ** 2)
            if cm.lethal_hits(poses[k:]) == 0:
                continue
            diag["replans"] += 1
            if diag["replans"] > max_replans:
                state["aborted"] = "replan_limit"
                handle.cancel_goal_async()
                break
            handle.cancel_goal_async()
            spin(0.3)
            newp = plan_from(cur)
            if newp is None:
                diag["replan_failed"] += 1
                continue
            path_msg = newp
            diag["path_len_m"] = round(_path_length(path_msg), 3)
            say("[nav2] 第 %d 次重规划：%d 个路点，长度 %.2f m"
                % (diag["replans"], len(path_msg.poses), diag["path_len_m"]))
            # 重规划只换路径，**不重置** t=0 / 行人相位 / 轨迹记录
            handle, res_future = start_follow(path_msg)
        # 让取消/收尾的样本也进来
        spin(0.5)
    finally:
        node.destroy_node()
        if own_context and rclpy.ok():
            rclpy.shutdown()

    diag["aborted"] = state["aborted"]
    diag["safety_status"] = state["safety_status"]
    if mpc_diag["solve_ms"] or mpc_diag["max_streak"]:
        diag["mpc_solve_ms"] = mpc_diag["solve_ms"]
        diag["mpc_solver_fail_streak_max"] = mpc_diag["max_streak"]
        diag["mpc_fallback_frames"] = mpc_diag["fallback_frames"]
    return {"trajectory": rec.trajectory(), "estop_times": list(rec.estops), "diag": diag}


def _path_length(path_msg) -> float:
    total = 0.0
    prev = None
    for ps in path_msg.poses:
        p = ps.pose.position
        if prev is not None:
            total += math.hypot(p.x - prev[0], p.y - prev[1])
        prev = (p.x, p.y)
    return total


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="M2 在线自检：对一个契约实例跑一次 follow_path 并打印指标")
    ap.add_argument("--manifest", default=str(default_manifest_path()))
    ap.add_argument("--scenario", default="static_avoid")
    ap.add_argument("--run", type=int, default=1)
    ap.add_argument("--timeout", type=float, default=60.0)
    ap.add_argument("--warmup", type=float, default=4.0)
    ap.add_argument("--dump", default="", help="把轨迹另存为 npz 便于排查")
    args = ap.parse_args(argv)

    import yaml
    from .judge import judge
    document = yaml.safe_load(open(args.manifest, encoding="utf-8"))
    inst = find_instance(document, args.scenario, args.run)
    world_pose = document.get("world_poses", {}).get(str(inst["seed"]),
                                                     {"x": 0.0, "y": 0.0, "yaw": 0.0})
    out = run_episode(inst, world_pose, args.timeout, warmup_s=args.warmup,
                      log=lambda m: print(m, flush=True))
    traj = out["trajectory"]
    print("[nav2] 样本：odom=%d cmd=%d 急停=%d diag=%s"
          % (len(traj["t"]), len(traj["control"]), len(out["estop_times"]), out["diag"]))
    if not traj["t"]:
        print("[nav2] 没有样本，无法判分")
        return 2
    detail = judge(inst, traj, out["estop_times"])
    print("[nav2] judge:", {k: v for k, v in detail.items() if k != "judge_detail"})
    print("[nav2] judge_detail:", detail["judge_detail"])
    if traj["t"]:
        import numpy as np
        t = np.asarray(traj["t"]); s = np.asarray(traj["state"]); c = np.asarray(traj["control"])
        if args.dump:
            np.savez(args.dump, t=t, state=s, control=c)
        n = len(t)
        idx = [int(k * (n - 1) / 9) for k in range(10)]
        print("[nav2] 轨迹采样（每 10% 一个点）: t / x / y / v:")
        for k in idx:
            print("   %6.2f s  x=%6.2f y=%6.2f v=(%5.2f,%5.2f) w=%5.2f"
                  % (t[k], s[k][0], s[k][1], c[k][0], c[k][1], c[k][2]))
        print("[nav2] 终点距离：%.2f m（目标 (%.2f, %.2f)）"
              % (math.hypot(s[-1][0] - inst["goal"]["x"], s[-1][1] - inst["goal"].get("y", 0.0)),
                 inst["goal"]["x"], inst["goal"].get("y", 0.0)))
    return 0 if detail["success"] else 1


if __name__ == "__main__":
    sys.exit(main())
