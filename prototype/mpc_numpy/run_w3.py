#!/usr/bin/env python3
"""W3 入口：换真 NavFn 参考（闭环第一刀）+ 验收四条（docs/MPC公式化.md §9/§12）。

用法（conda pro 环境；ROS 环境需已 source，经 run_w3.sh 调用）：
  python run_w3.py --all                  # 三场景 + 验收表 + 诊断图
  python run_w3.py --scenario S1
  python run_w3.py --all --chassis chassis/diff.yaml   # 迁移冒烟预演（vy_max=0）
退出码 0 ⟺ 全部场景过验收②③④且到达（①一条命令由 run_w3.sh 保证）。

换源（§12）：subprocess 调系统 Python 的 mpc_proto/navfn_source.py，一次性
ComputePathToPose 取折线 → dataclasses.replace(sc, waypoints=...) → sim.run
零改动。参考机制（§2 时间律/目标截断）不变；不平滑、不抽稀、不重规划。

编排：每场景独立拉起 map_server+planner_server（规划栈逐场景重启）——
LoadMap 单栈换图存在 StaticLayer 异步竞态，会串图（旧图上规划出非空错路）；
新栈从 latched 地图初始化是 Nav2 标准路径，从根上消除串图。
"""
from __future__ import annotations

import argparse
import dataclasses
import json
import os
import shutil
import signal
import subprocess
import sys
import time
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT))

from mpc_proto.chassis import load_chassis
from mpc_proto.mpc import MPCConfig
from mpc_proto.plotting import plot_run
from mpc_proto.scenarios import ALL, by_name
from mpc_proto.sim import RunResult, run
from run_sim import fmt_row, print_table


def fetch_json(sys_py: str, fetcher: Path, subcmd: str, sub_args: list[str], timeout: float) -> dict:
    """调 navfn_source.py，解析 stdout 末行 JSON；任何异常 → {'status':'error',...}。"""
    try:
        p = subprocess.run(
            [sys_py, str(fetcher), subcmd, *sub_args],
            capture_output=True, text=True, timeout=timeout,
        )
    except subprocess.TimeoutExpired:
        return {"status": "error", "reason": f"fetcher 超时 {timeout}s"}
    lines = p.stdout.strip().splitlines()
    if not lines:
        tail = (p.stderr or "").strip()[-300:]
        return {"status": "error", "reason": f"fetcher 无 JSON 输出（rc={p.returncode}）: {tail}"}
    try:
        return json.loads(lines[-1])
    except json.JSONDecodeError as e:
        return {"status": "error", "reason": f"JSON 解析失败: {e}"}


def fail_row(name: str, reason: str) -> dict:
    return {
        "场景": name, "到达": "否", "t [s]": "—", "解算 ok/fail": "—",
        "min间隙 [m]": "—", "TV/拍": "—", "max slack": "—",
        "②逐步解算": "FAIL", "③间隙>0": "FAIL", "④Δu有界": "FAIL",
        "_reason": reason,
    }


def bringup(map_yaml: Path, log_dir: Path, name: str) -> list[subprocess.Popen]:
    """逐场景拉起 map_server + planner_server；各自独立进程组（teardown 用 killpg）。"""
    log_dir.mkdir(parents=True, exist_ok=True)
    procs = []
    for exe, log in (("nav2_map_server map_server", f"{name}_map.log"),
                     ("nav2_planner planner_server", f"{name}_planner.log")):
        pkg, node = exe.split()
        cmd = ["ros2", "run", pkg, node, "--ros-args",
               "--params-file", str(ROOT / "nav2_w3_params.yaml")]
        if node == "map_server":
            cmd += ["-p", f"yaml_filename:={map_yaml}"]
        f = open(log_dir / log, "w")
        procs.append(subprocess.Popen(cmd, start_new_session=True,
                                      stdout=f, stderr=subprocess.STDOUT))
        f.close()
    return procs


def lifecycle_activate() -> bool:
    for node in ("/map_server", "/planner_server"):
        for transition in ("configure", "activate"):
            for attempt in range(3):
                r = subprocess.run(["ros2", "lifecycle", "set", node, transition],
                                   capture_output=True, text=True, timeout=30)
                if r.returncode == 0 and "successful" in (r.stdout + r.stderr).lower():
                    break
                if attempt == 2:
                    print(f"  ✗ lifecycle {node} {transition} 失败: "
                          f"{(r.stdout + r.stderr).strip()[:200]}")
                    return False
                time.sleep(1.0)
    return True


def teardown(procs: list[subprocess.Popen]) -> None:
    for p in procs:
        try:
            os.killpg(p.pid, signal.SIGTERM)
        except (ProcessLookupError, PermissionError):
            pass
    for p in procs:
        try:
            p.wait(timeout=5)
        except subprocess.TimeoutExpired:
            try:
                os.killpg(p.pid, signal.SIGKILL)
            except (ProcessLookupError, PermissionError):
                pass


def main() -> int:
    ap = argparse.ArgumentParser(description="W3 numpy MPC（真 NavFn 参考）")
    ap.add_argument("--all", action="store_true", help="跑 S1/S2/S3 全部场景")
    ap.add_argument("--scenario", choices=["S1", "S2", "S3"], help="单场景")
    ap.add_argument("--N", type=int, default=40, help="预测时域步数（定死 40）")
    ap.add_argument("--chassis", default=str(ROOT / "chassis/omni.yaml"))
    ap.add_argument("--out", default=str(ROOT / "out"))
    ap.add_argument("--map-dir", default=str(ROOT / "out/maps"))
    ap.add_argument("--sys-python", default="/usr/bin/python3")
    ap.add_argument("--fetcher", default=str(ROOT / "mpc_proto/navfn_source.py"))
    ap.add_argument("--action-timeout", type=float, default=30.0)
    ap.add_argument("--no-plot", action="store_true")
    ap.add_argument("--quiet", action="store_true", help="不打印逐步日志")
    args = ap.parse_args()

    if not args.all and not args.scenario:
        ap.error("需 --all 或 --scenario")
    if shutil.which("ros2") is None:
        print("找不到 ros2（需 ROS 环境）——请经 run_w3.sh 运行", file=sys.stderr)
        return 1

    chassis = load_chassis(args.chassis)
    cfg = MPCConfig(N=args.N)
    out_dir = Path(args.out)
    fetcher = Path(args.fetcher)
    print(
        f"MPC W3（NavFn 真参考）| N={cfg.N} dt={cfg.dt}s 视界={cfg.N * cfg.dt:.1f}s | "
        f"chassis={chassis.type} r={chassis.radius} vy_max={chassis.vy_max} | "
        f"q=({cfg.q_lat},{cfg.q_long},{cfg.q_theta}) ρ={cfg.rho_slack} d_safe={cfg.d_safe}"
    )

    scenes = list(ALL) if args.all else [lambda: by_name(args.scenario)]
    rows = []
    all_pass = True
    for fn in scenes:
        sc0 = fn()
        print(f"\n=== {sc0.name} {sc0.desc}（NavFn 真参考）===")
        map_yaml = (Path(args.map_dir) / sc0.name / f"{sc0.name}.yaml").resolve()

        # 1) 逐场景规划栈（新栈从 latched 地图初始化，杜绝串图）
        procs = bringup(map_yaml, out_dir / "w3_logs", sc0.name)
        try:
            time.sleep(2.0)  # 节点服务就绪
            if not lifecycle_activate():
                rows.append(fail_row(sc0.name, "lifecycle 转移失败"))
                all_pass = False
                continue

            # 2) 一次性取 NavFn 折线（空路径有界重试，兜住 latched 地图加载竞态）
            wps, length = None, 0.0
            r: dict = {}
            for attempt in range(5):
                r = fetch_json(
                    args.sys_python, fetcher, "get-path",
                    ["--start", *[f"{v:.6g}" for v in sc0.start],
                     "--goal", *[f"{v:.6g}" for v in sc0.goal],
                     "--timeout", str(args.action_timeout)],
                    args.action_timeout + 15.0,
                )
                if r.get("status") == "ok":
                    cand = np.asarray(r["waypoints"], dtype=float)
                    if len(cand) >= 2:
                        wps, length = cand, float(r["length"])
                        break
                    r["reason"] = f"路径点数不足: {len(cand)}"
                if attempt < 4:
                    print(f"  ! get-path 第 {attempt + 1} 次未取到路径（{r.get('reason')}），1 s 后重试")
                    time.sleep(1.0)
            if wps is None:
                print(f"  ✗ get-path 失败: {r.get('reason')}")
                rows.append(fail_row(sc0.name, f"get-path: {r.get('reason')}"))
                all_pass = False
                continue
        finally:
            teardown(procs)  # 规划到手即收栈；sim 推演不再依赖 ROS

        start_err = float(np.linalg.norm(wps[0] - sc0.start[:2]))
        goal_err = float(np.linalg.norm(wps[-1] - sc0.goal[:2]))
        if goal_err > 0.15:  # 回退：NavFn 提前收径时补目标点（§12 编排回退）
            wps = np.vstack([wps, sc0.goal[:2]])
            print(f"  ! 端点距目标 {goal_err:.2f} m > 0.15，补目标 waypoint")
            goal_err = float(np.linalg.norm(wps[-1] - sc0.goal[:2]))
        print(f"  NavFn {sc0.name}: M={len(wps)}, L={length:.2f} m, "
              f"端点误差 start={start_err:.3f} goal={goal_err:.3f}")

        # 3) 换源：sim.run / ReferencePath 零改动
        sc = dataclasses.replace(sc0, waypoints=wps)
        res: RunResult = run(sc, chassis, cfg, log_every=0 if args.quiet else 100)
        rows.append(fmt_row(res))
        all_pass &= res.acceptance_pass() and res.arrived
        if not args.no_plot:
            p = plot_run(sc, chassis, res, out_dir / f"{sc.name}.png")
            print(f"  诊断图 → {p}")

    print_table(rows)
    print(
        "\n验收②逐步解算成功 ③全程间隙>0 ④Δu总变差有界（NavFn 参考）：",
        "ALL PASS" if all_pass else "FAIL",
        "（①一条命令见 run_w3.sh；到达判定 0.25m/0.25rad）",
    )
    return 0 if all_pass else 1


if __name__ == "__main__":
    raise SystemExit(main())
