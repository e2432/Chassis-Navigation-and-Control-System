#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""9-3 时限冻结用的工作集批量采集：一个场景只启动一次 Isaac，跑完三个基线。

背景与设计（为什么这么写）：
  * 9-3 的公式是 `T_s = max(T_min,s , 1.5 · max_j t̃(s,j))`，只用**成功 run**；
    工作集是 5 场景 × 前 5 种子 = 25 个场景实例，× 3 基线 = 75 run。
  * 每个种子对应**不同的场景几何**（障碍位置不同），所以换种子必须换场景；
    但换基线**不需要**重启 Isaac——三个控制器同时注册在同一个 controller_server 上，
    由 FollowPath goal 的 `controller_id` 选（见 robot_bringup/launch/nav2_baseline.launch.py）。
    这样 75 run 只需要 25 次 Isaac 启动，墙钟从 3 小时量级降到 1 小时量级。
  * 可恢复：已存在 run.json 的 (场景, 种子, 基线) 直接跳过，中断后重跑同一条命令即可。

用法（必须先准备好环境，见下方 env 段）：
    python3 scripts/run_workset.py --out <repo>/out/9-3_workset
"""

from __future__ import annotations

import argparse
import os
import signal
import subprocess
import sys
import time
from pathlib import Path

SCENARIOS = ("static_avoid", "dynamic_pedestrian", "narrow_passage", "dead_end", "mixed")
# mpc = 自研 MPC（Nav2 控制器 id=mpc，落盘 controller=mpc_cpp）；默认集仍是三个基线
BASELINES = ("dwa", "mppi", "teb", "mpc")


def log(msg: str) -> None:
    print("[%s] %s" % (time.strftime("%H:%M:%S"), msg), flush=True)


def repo_root() -> Path:
    return Path(__file__).resolve().parents[2]


def kill_tree(proc: subprocess.Popen) -> None:
    """杀掉 launch 及其子进程（Isaac / nav2 节点都挂在同一个进程组里）。"""
    if proc.poll() is not None:
        return
    try:
        os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
    except Exception:
        try:
            proc.kill()
        except Exception:
            pass
    try:
        proc.wait(timeout=15)
    except Exception:
        pass


def wait_ready(logfile: Path, timeout_s: float = 180.0) -> bool:
    """等 lifecycle 打出 'Managed nodes are active'。"""
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        try:
            text = logfile.read_text(encoding="utf-8", errors="replace")
        except Exception:
            text = ""
        if "Managed nodes are active" in text:
            return True
        if "Failed to bring up all requested nodes" in text or "process has died" in text:
            return False
        time.sleep(2.0)
    return False


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="9-3 工作集批量采集（一场景一启动，三基线共用）")
    ap.add_argument("--out", required=True, help="输出根目录")
    ap.add_argument("--scenarios", nargs="+", default=list(SCENARIOS))
    ap.add_argument("--runs", nargs="+", type=int, default=[1, 2, 3, 4, 5])
    ap.add_argument("--baselines", nargs="+", default=list(BASELINES))
    ap.add_argument("--nav2-delay", default="30.0")
    ap.add_argument("--scene-dir", default="")
    ap.add_argument("--retries", type=int, default=1,
                    help="基础设施失败（栈未就绪/规划器偶发失败）时整场景重试次数")
    ap.add_argument("--keep-stack", action="store_true", help="不杀栈（调试用）")
    ap.add_argument("--mpc-obstacles", choices=["mock", "feeder"], default="mock",
                    help="baseline 含 mpc 时 /mpc/obstacles 的来源："
                         "mock=launch 的空数组心跳（开发/空场景）；"
                         "feeder=scripts/obstacle_feeder.py 发真实栏杆矩形（正式 A 口径）")
    args = ap.parse_args(argv)

    def _split(items):
        out = []
        for it in items:
            out.extend([x.strip() for x in str(it).split(",") if x.strip()])
        return out

    args.scenarios = _split(args.scenarios)
    args.baselines = _split(args.baselines)

    root = repo_root()
    nav_ws = root / "nav_ws_core"
    out = Path(args.out).resolve()
    out.mkdir(parents=True, exist_ok=True)
    scenes_dir = Path(args.scene_dir) if args.scene_dir else out / "scenes"
    scenes_dir.mkdir(parents=True, exist_ok=True)
    maps_dir = out / "maps"
    maps_dir.mkdir(parents=True, exist_ok=True)
    logdir = out / "logs"
    logdir.mkdir(parents=True, exist_ok=True)

    # 场景生成与实例表查询都要用 robot_benchmark 包
    sys.path.insert(0, str(nav_ws / "src" / "robot_benchmark"))
    import yaml
    from robot_benchmark.scenes import write_nav2_map, write_scene
    from robot_benchmark.scenario_gen import find_instance, default_manifest_path

    doc = yaml.safe_load(open(default_manifest_path(), encoding="utf-8"))
    robot_usd = nav_ws / "assets" / "robot_a_chassis_agl_chs03_f01.usd"
    ctrl_list = ",".join(args.baselines)

    total = 0
    done = 0
    failed = []
    retry_queue = []
    def process_scene(scenario, run):
        """处理一个场景实例；返回失败的基线列表（空=全部成功）。"""
        inst = find_instance(doc, scenario, run)
        seed = inst["seed"]
        need = [b for b in args.baselines
                if not (out / b / "runs" / scenario / str(seed) / "run.json").is_file()]
        if not need:
            log("跳过 %s run=%2d seed=%3d（三个基线都已有 run.json）" % (scenario, run, seed))
            return []

        wp = doc["world_poses"][str(seed)]
        usda = write_scene(inst, wp, scenes_dir / ("%s_%02d.usda" % (scenario, run)), robot_usd)
        # 同时写 Nav2 地图（PGM+YAML），交给 nav2_map_server 发布 /map
        nav_map = write_nav2_map(inst, wp, maps_dir / ("%s_%02d" % (scenario, run)))
        log("场景 %s run=%2d seed=%3d 障碍=%d 行人=%d 墙=%d → %s（地图静态格=%d）"
            % (scenario, run, seed, len(inst.get("obstacles") or []),
               len(inst.get("pedestrians") or []), len(inst.get("walls") or []),
               os.path.basename(usda), nav_map["occupied_cells"]))

        stack_log = logdir / ("stack_%s_%02d.log" % (scenario, run))
        use_feeder = ("mpc" in args.baselines) and args.mpc_obstacles == "feeder"
        cmd = ["ros2", "launch", "robot_bringup", "simulation.launch.py",
               "nav2:=%s" % args.baselines[0], "controllers:=%s" % ctrl_list,
               "nav2_delay:=%s" % args.nav2_delay, "map:=%s" % nav_map["yaml"],
               "demo:=false", "rviz:=false", "duration:=0", "scene:=%s" % usda,
               # 算法侧 G5：feeder 在场时必须关掉 launch 的空数组心跳，
               # 否则 /mpc/obstacles 有两个发布者（采集器会直接判"套件不唯一"）
               "mock_obstacles:=%s" % ("false" if use_feeder else "true")]
        with open(stack_log, "wb") as fh:
            proc = subprocess.Popen(cmd, cwd=str(nav_ws), stdout=fh, stderr=subprocess.STDOUT,
                                    start_new_session=True)
        bad = []
        feeder = None
        try:
            if not wait_ready(stack_log):
                log("!! 栈未就绪（%s）" % stack_log)
                return list(need)
            time.sleep(5.0)      # 让 2 Hz 的代价地图再更新几拍

            if use_feeder:
                # G4：正式 /mpc/obstacles 数据源（A 口径：只发栏杆矩形，20 Hz，reliable depth 10）
                feeder_log = logdir / ("feeder_%s_%02d.log" % (scenario, run))
                fcmd = [sys.executable, str(nav_ws / "scripts" / "obstacle_feeder.py"),
                        "--mode", "gate", "--qos", "reliable", "--rate", "20",
                        "--frame", "odom"]
                with open(feeder_log, "wb") as ffh:
                    feeder = subprocess.Popen(fcmd, stdout=ffh, stderr=subprocess.STDOUT,
                                              start_new_session=True)
                log("feeder 已起（%s）" % os.path.basename(feeder_log))
                time.sleep(2.0)
            for b in need:
                if feeder is not None and feeder.poll() is not None:
                    log("!! feeder 提前退出（rc=%s）→ 本轮 mpc 数据不可信" % feeder.returncode)
                    bad.append("mpc")
                    break
                t0 = time.monotonic()
                r = subprocess.run(
                    [sys.executable, "-m", "robot_benchmark.runner", "--backend", "nav2",
                     "--baseline", b, "--scenario", scenario, "--runs", str(run),
                     "--out", str(out), "--scene", str(usda)],
                    cwd=str(nav_ws / "src" / "robot_benchmark"),
                    capture_output=True, text=True)
                ok = (out / b / "runs" / scenario / str(seed) / "run.json").is_file()
                log("  %-4s %s run=%2d → %s（%.1f s）%s"
                    % (b, scenario, run, "OK" if ok else "失败",
                       time.monotonic() - t0,
                       "" if ok else " | " + (r.stderr or r.stdout)[-200:].replace("\n", " ")))
                if not ok:
                    bad.append(b)
        finally:
            if feeder is not None:
                kill_tree(feeder)          # 先收 feeder，避免它在下个场景里抢 /mpc/obstacles
            if not args.keep_stack:
                kill_tree(proc)
                time.sleep(3.0)
        return bad

    for scenario in args.scenarios:
        for run in args.runs:
            total += 1
            bad = process_scene(scenario, run)
            if bad:
                failed.extend((scenario, run, b) for b in bad)
                retry_queue.append((scenario, run))

    # 基础设施失败（栈未就绪 / 规划器偶发失败 / 代价地图门限）允许整场景重试
    for attempt in range(1, args.retries + 1):
        if not retry_queue:
            break
        log("=== 第 %d 次重试：%d 个场景实例 ===" % (attempt, len(retry_queue)))
        queue, retry_queue = retry_queue, []
        failed = []
        for (scenario, run) in queue:
            bad = process_scene(scenario, run)
            if bad:
                failed.extend((scenario, run, b) for b in bad)
                retry_queue.append((scenario, run))

    log("完成：处理 %d 个场景实例，失败 %d" % (total, len(failed)))
    for f in failed:
        log("  失败项：%s" % (f,))
    return 0 if not failed else 1


if __name__ == "__main__":
    sys.exit(main())
