#!/usr/bin/env python3
"""E02 仿真侧桥接检查台（Isaac Sim 5.1 + ROS 2 Bridge，headless）。

做四件事：
  1) 用官方 Action Graph 节点（ROS2Context / OnPlaybackTick / IsaacReadSimulationTime /
     ROS2PublishClock / ROS2PublishTransformTree）发布 /clock 与 /tf，作为“仿真 → 外部 ROS 2”方向；
  2) 用 ROS2SubscribeTwist 订阅 /w1/ext_to_sim，逐帧读取节点输出，作为“外部 ROS 2 → 仿真”方向；
  3) 按固定剧本 播放 → 暂停 → 恢复 → 停止并重置 → 再播放，记录仿真时间行为；
  4) 全过程写 CSV 事件日志，供 e02_analyze.py 汇总。

必须由 e02_run_harness.sh 启动（该脚本负责 source 系统 ROS 2 Humble，否则 Bridge 起不来）。
本脚本只做验证，不保存 USD、不修改任何用户配置。
"""

import argparse
import csv
import os
import sys
import time

# Isaac Sim 会把不认识的参数当自己的参数，这里先吃掉本脚本的参数
parser = argparse.ArgumentParser()
parser.add_argument("--out", required=True, help="输出目录")
parser.add_argument("--reset-on-stop", default="false",
                    help="IsaacReadSimulationTime 的 resetOnStop（true = 停止后时钟回跳）")
parser.add_argument("--play-seconds", type=float, default=10.0, help="第一段播放的仿真秒数")
parser.add_argument("--pause-seconds", type=float, default=5.0, help="暂停持续的墙上秒数")
parser.add_argument("--resume-seconds", type=float, default=6.0, help="恢复后播放的仿真秒数")
parser.add_argument("--after-reset-seconds", type=float, default=5.0, help="重置后播放的仿真秒数")
parser.add_argument("--target-rtf", type=float, default=1.0,
                    help="目标实时因子（主循环限速用；0 表示不限速）")
parser.add_argument("--trigger-timeout", type=float, default=180.0,
                    help="等待外部第一条 Twist 触发消息的墙上秒上限")
args, _unknown = parser.parse_known_args()

OUT = os.path.abspath(args.out)
os.makedirs(OUT, exist_ok=True)
RESET_ON_STOP = str(args.reset_on_stop).strip().lower() in ("1", "true", "yes")

EVENTS = os.path.join(OUT, "sim_events.csv")
SUBRX = os.path.join(OUT, "sim_sub_rx.csv")
CLOCK = os.path.join(OUT, "sim_clock_samples.csv")

T0_WALL = time.time()


def wall():
    return time.time() - T0_WALL


def say(msg):
    print("[E02-SIM] %6.2f %s" % (wall(), msg), flush=True)


say("harness start; reset_on_stop=%s out=%s" % (RESET_ON_STOP, OUT))
say("ROS_DOMAIN_ID=%s RMW_IMPLEMENTATION=%s ROS_DISTRO=%s"
    % (os.environ.get("ROS_DOMAIN_ID", "<unset>"),
       os.environ.get("RMW_IMPLEMENTATION", "<unset>"),
       os.environ.get("ROS_DISTRO", "<unset>")))

from isaacsim import SimulationApp  # noqa: E402

sim_app = SimulationApp({"headless": True, "width": 640, "height": 480})
say("SimulationApp ready")

import carb  # noqa: E402
import omni.graph.core as og  # noqa: E402
import omni.kit.app  # noqa: E402
import omni.timeline  # noqa: E402
import omni.usd  # noqa: E402
from pxr import Gf, UsdGeom  # noqa: E402

app = omni.kit.app.get_app()
ext_mgr = app.get_extension_manager()

# ---- 1. 启用 ROS 2 Bridge（Isaac Sim 5.1 扩展名）----
ext_name = "isaacsim.ros2.bridge"
if not ext_mgr.is_extension_enabled(ext_name):
    ext_mgr.set_extension_enabled_immediate(ext_name, True)
    sim_app.update()
bridge_ok = ext_mgr.is_extension_enabled(ext_name)
say("bridge extension enabled=%s" % bridge_ok)
if not bridge_ok:
    say("FATAL: ROS2 bridge 未能启用，检查是否 source 了 /opt/ros/humble")
    sim_app.close()
    sys.exit(2)
settings = carb.settings.get_settings()
say("bridge ros_distro setting=%s" % settings.get("/exts/isaacsim.ros2.bridge/ros_distro"))

# ---- 2. 空场景 + 一个用于 /tf 的测试帧 ----
stage = omni.usd.get_context().get_stage()
test_prim_path = "/World/W1TestFrame"
if not stage.GetPrimAtPath("/World"):
    UsdGeom.Xform.Define(stage, "/World")
if not stage.GetPrimAtPath(test_prim_path):
    xf = UsdGeom.Xform.Define(stage, test_prim_path)
    xf.AddTranslateOp().Set(Gf.Vec3d(1.0, 2.0, 0.5))
say("empty scene ready, test frame=%s" % test_prim_path)

# ---- 3. Action Graph ----
GRAPH = "/W1BridgeCheck"
try:
    og.Controller.edit(
        {"graph_path": GRAPH, "evaluator_name": "execution"},
        {
            og.Controller.Keys.CREATE_NODES: [
                ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                ("ROS2Context", "isaacsim.ros2.bridge.ROS2Context"),
                ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                ("PublishClock", "isaacsim.ros2.bridge.ROS2PublishClock"),
                ("PublishTF", "isaacsim.ros2.bridge.ROS2PublishTransformTree"),
                ("SubTwist", "isaacsim.ros2.bridge.ROS2SubscribeTwist"),
            ],
            og.Controller.Keys.SET_VALUES: [
                ("ReadSimTime.inputs:resetOnStop", RESET_ON_STOP),
                ("PublishClock.inputs:topicName", "/clock"),
                ("PublishClock.inputs:nodeNamespace", ""),
                ("PublishClock.inputs:queueSize", 10),
                ("PublishTF.inputs:topicName", "/tf"),
                ("PublishTF.inputs:nodeNamespace", ""),
                ("PublishTF.inputs:targetPrims", [test_prim_path]),
                ("PublishTF.inputs:parentPrim", ""),
                ("SubTwist.inputs:topicName", "/w1/ext_to_sim"),
                ("SubTwist.inputs:nodeNamespace", ""),
                ("SubTwist.inputs:queueSize", 10),
            ],
            og.Controller.Keys.CONNECT: [
                ("OnPlaybackTick.outputs:tick", "PublishClock.inputs:execIn"),
                ("OnPlaybackTick.outputs:tick", "PublishTF.inputs:execIn"),
                ("OnPlaybackTick.outputs:tick", "SubTwist.inputs:execIn"),
                ("ROS2Context.outputs:context", "PublishClock.inputs:context"),
                ("ROS2Context.outputs:context", "PublishTF.inputs:context"),
                ("ROS2Context.outputs:context", "SubTwist.inputs:context"),
                ("ReadSimTime.outputs:simulationTime", "PublishClock.inputs:timeStamp"),
                ("ReadSimTime.outputs:simulationTime", "PublishTF.inputs:timeStamp"),
            ],
        },
    )
    say("action graph %s created" % GRAPH)
except Exception as exc:  # pragma: no cover - 诊断用
    say("FATAL: action graph 创建失败: %r" % (exc,))
    sim_app.close()
    sys.exit(3)

sub_lin = og.Controller.attribute(GRAPH + "/SubTwist.outputs:linearVelocity")
sub_ang = og.Controller.attribute(GRAPH + "/SubTwist.outputs:angularVelocity")

# ---- 4. 剧本 ----
timeline = omni.timeline.get_timeline_interface()

f_ev = open(EVENTS, "w", newline="")
w_ev = csv.writer(f_ev)
w_ev.writerow(["wall_epoch", "wall_s", "sim_time_s", "timeline_state", "event", "detail"])

f_rx = open(SUBRX, "w", newline="")
w_rx = csv.writer(f_rx)
w_rx.writerow(["wall_epoch", "wall_s", "sim_time_s", "lin_x", "lin_y", "lin_z", "ang_x", "ang_y", "ang_z"])

f_clk = open(CLOCK, "w", newline="")
w_clk = csv.writer(f_clk)
w_clk.writerow(["wall_epoch", "wall_s", "sim_time_s"])

last_rx = None
last_clock_log = 0.0
TARGET_RTF = float(args.target_rtf)


def pump(seconds, note=""):
    """按 --target-rtf 限速运行一段时间，同时记录时钟采样与订阅数据。"""
    global last_rx, last_clock_log
    end = time.time() + seconds
    prev_sim = timeline.get_current_time()
    while time.time() < end:
        t_loop = time.time()
        sim_app.update()
        sim_t = timeline.get_current_time()
        w = wall()
        if w - last_clock_log >= 0.05:
            w_clk.writerow([round(time.time(), 6), round(w, 4), round(sim_t, 6)])
            last_clock_log = w
        try:
            lin = tuple(round(float(v), 6) for v in sub_lin.get())
            ang = tuple(round(float(v), 6) for v in sub_ang.get())
        except Exception:
            lin, ang = None, None
        if lin is not None:
            key = (lin, ang)
            if key != last_rx:
                w_rx.writerow([round(time.time(), 6), round(w, 4), round(sim_t, 6)] + list(lin) + list(ang))
                say("subscriber rx  sim=%.3f lin=%s ang=%s" % (sim_t, lin, ang))
                last_rx = key
        # 限速：让仿真时间大约按 TARGET_RTF 倍墙上时间推进
        if TARGET_RTF > 0 and timeline.is_playing():
            target_dt = max(0.0, (sim_t - prev_sim) / TARGET_RTF)
            spent = time.time() - t_loop
            if target_dt > spent:
                time.sleep(min(0.05, target_dt - spent))
        prev_sim = sim_t


def mark(event, detail=""):
    sim_t = timeline.get_current_time()
    w_ev.writerow([round(time.time(), 6), round(wall(), 4), round(sim_t, 6),
                   "playing" if timeline.is_playing() else "stopped", event, detail])
    say("EVENT %s %s (sim=%.3f)" % (event, detail, sim_t))


mark("harness_ready", "reset_on_stop=%s" % RESET_ON_STOP)

# ---- 5. 先开播放，让 /clock 与 /tf 立刻对外的可见，等外部记录器发来第一条 Twist 作触发 ----
say("=== phase 0: 播放并等待外部触发（最多 %.0f 墙上秒）===" % args.trigger_timeout)
timeline.play()
mark("play_start", "phase0_awating_trigger")
trigger_deadline = time.time() + args.trigger_timeout
triggered = False
while time.time() < trigger_deadline:
    pump(0.5)
    if last_rx is not None and abs(last_rx[1][2]) > 0.0:
        triggered = True
        break
    if not sim_app.is_running():
        break
mark("trigger_state", "triggered=%s" % triggered)
if not triggered:
    say("WARN: 未在 %.0f 秒内收到外部 Twist，按时间推进继续剧本" % args.trigger_timeout)

say("=== phase 1: play %.1f sim-seconds ===" % args.play_seconds)
mark("play_start", "phase1")
pump(args.play_seconds)
mark("play_done", "phase1")

say("=== phase 2: pause %.1f wall-seconds ===" % args.pause_seconds)
timeline.pause()
mark("pause_start", "phase2")
pump(args.pause_seconds)
mark("pause_end", "phase2")

say("=== phase 3: resume %.1f sim-seconds ===" % args.resume_seconds)
timeline.play()
mark("resume_start", "phase3")
pump(args.resume_seconds)
mark("resume_done", "phase3")

say("=== phase 4: stop (reset scene) then play again ===")
timeline.stop()
mark("stop_start", "phase4")
pump(2.0)
mark("stop_end", "phase4")
timeline.play()
mark("replay_start", "phase4")
pump(args.after_reset_seconds)
mark("replay_done", "phase4")

mark("harness_end", "ok")
f_ev.close()
f_rx.close()
f_clk.close()
say("harness finished; outputs in %s" % OUT)
sim_app.close()
say("closed")
