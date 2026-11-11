#!/usr/bin/env python3
"""W3/W4 入口：换真 NavFn 参考（闭环）+ 验收/实例批量（docs/MPC公式化.md §9/§12）。

用法（conda pro 环境；ROS 环境需已 source，经 run_w3.sh / run_w4.sh 调用）：
  python run_w3.py --all                  # S1~S3 + 验收表 + 诊断图（W3 回归）
  python run_w3.py --scenario S1
  python run_w3.py --instance static_avoid:1        # W4 单实例端到端
  python run_w3.py --instances static_avoid --runs 1-5
  python run_w3.py --suite --runs 1-5               # 5 场景 × 前 5（调参工作集）
  python run_w3.py --suite --runs all               # 全 20（判读数据）
  python run_w3.py --suite --runs all --map-only    # 只取 NavFn 路径做可行性扫描
  python run_w3.py --all --chassis chassis/diff.yaml   # 迁移冒烟预演（vy_max=0）

换源（§12）：subprocess 调系统 Python 的 mpc_proto/navfn_source.py，一次性
ComputePathToPose 取折线 → dataclasses.replace(sc, waypoints=...) → sim.run
零改动。参考机制（§2 时间律/目标截断）不变；不平滑、不抽稀、不重规划。

编排：每个实例独立拉起 map_server+planner_server（规划栈逐实例重启）——
LoadMap 单栈换图存在 StaticLayer 异步竞态，会串图；新栈从 latched 地图初始化
是 Nav2 标准路径，从根上消除串图。

W4 实例模式采用 MPCConfig.benchmark() 预设（调参唯一入口），S1~S3 继续用
MPCConfig 默认值——W2/W3 存档基准不动。NavFn/生命周期失败写一条 schema 同构的
failure run（failure_class=other），保证汇总 N 完整、不伪造成 collision/timeout。
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
from dataclasses import dataclass
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT))

from mpc_proto.chassis import load_chassis
from mpc_proto.mapping import write_map
from mpc_proto.mpc import MPCConfig
from mpc_proto.plotting import plot_run
from mpc_proto.runlog import write_failure, write_run
from mpc_proto.scenarios import (
    ALL,
    SCENARIO_KEYS,
    InstanceMeta,
    Scenario,
    by_name,
    load_instance,
    load_instances,
    parse_instance_arg,
    parse_runs_spec,
)
from mpc_proto.sim import RunResult, run
from run_sim import fmt_row, print_table


@dataclass
class RunSpec:
    """一条待跑记录：模拟场景（meta=None）或基准实例。"""

    sc: Scenario
    meta: InstanceMeta | None = None

    @property
    def tag(self) -> str:
        return self.meta.scenario_id if self.meta is not None else self.sc.name


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
    """逐实例拉起 map_server + planner_server；各自独立进程组（teardown 用 killpg）。"""
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


def fetch_navfn_path(
    sc0: Scenario, map_yaml: Path, log_dir: Path, tag: str, args
) -> tuple[np.ndarray | None, float, str | None]:
    """一次拉起规划栈 → 取折线 → 收栈；失败返回 (None, 0, reason)。"""
    procs = bringup(map_yaml, log_dir, tag)
    try:
        time.sleep(2.0)  # 节点服务就绪
        if not lifecycle_activate():
            return None, 0.0, "lifecycle 转移失败"

        wps, length = None, 0.0
        r: dict = {}
        for attempt in range(5):
            r = fetch_json(
                args.sys_python, Path(args.fetcher), "get-path",
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
            return None, 0.0, f"get-path: {r.get('reason')}"
        return wps, length, None
    finally:
        teardown(procs)  # 规划到手即收栈；sim 推演不再依赖 ROS


def instance_keys(text: str) -> tuple[str, ...]:
    """解析 --instances：all 或逗号分隔的 5 个场景键。"""
    if text.strip().lower() == "all":
        return SCENARIO_KEYS
    keys = tuple(k.strip() for k in text.split(",") if k.strip())
    if not keys:
        raise ValueError("--instances 不能为空（用 all 或场景键列表）")
    unknown = [k for k in keys if k not in SCENARIO_KEYS]
    if unknown:
        raise ValueError(f"--instances 未知场景键 {unknown}（可选 {SCENARIO_KEYS} 或 all）")
    return keys


def build_specs(args, runs: list[int] | None) -> list[RunSpec]:
    """按 CLI 模式构建待跑清单。runs 为 None 表示不适用（S1~S3）。"""
    if args.instance:
        key, run_no = parse_instance_arg(args.instance)
        sc, meta = load_instance(key, run_no)
        return [RunSpec(sc, meta)]
    if args.instances is not None or args.suite:
        keys = set(instance_keys(args.instances or "all"))
        return [RunSpec(sc, meta) for sc, meta in load_instances(runs=runs) if meta.scenario in keys]
    if args.all:
        return [RunSpec(fn()) for fn in ALL]
    if args.scenario:
        return [RunSpec(by_name(args.scenario))]
    raise ValueError("需 --all / --scenario / --instance / --instances / --suite 五选一")


def main() -> int:
    ap = argparse.ArgumentParser(description="W3/W4 numpy MPC（真 NavFn 参考 + 基准实例）")
    mode = ap.add_mutually_exclusive_group(required=True)
    mode.add_argument("--all", action="store_true", help="跑 S1/S2/S3 全部模拟场景")
    mode.add_argument("--scenario", choices=["S1", "S2", "S3"], help="单模拟场景")
    mode.add_argument("--instance", metavar="<scenario>:<run>", help="W4 单实例（如 static_avoid:1）")
    mode.add_argument("--instances", metavar="all|key1,key2", help="W4 实例批量（场景键列表或 all）")
    mode.add_argument("--suite", action="store_true", help="W4 5 场景全实例（等价 --instances all）")
    ap.add_argument("--runs", default=None, help="实例 run 过滤：all / 1-5 / 1,3,5（仅 --instances/--suite）")
    ap.add_argument("--map-only", action="store_true", help="只写图并取 NavFn 路径，不推演/不落 run（实例模式）")
    ap.add_argument("--N", type=int, default=40, help="预测时域步数（定死 40）")
    ap.add_argument("--chassis", default=str(ROOT / "chassis/omni.yaml"))
    ap.add_argument("--out", default=str(ROOT / "out"))
    ap.add_argument("--map-dir", default=str(ROOT / "out/maps"))
    ap.add_argument("--sys-python", default="/usr/bin/python3")
    ap.add_argument("--fetcher", default=str(ROOT / "mpc_proto/navfn_source.py"))
    ap.add_argument("--action-timeout", type=float, default=30.0)
    ap.add_argument("--no-vo", action="store_true", help="消融：动态障碍不作速度预测")
    ap.add_argument("--no-esdf", action="store_true", help="消融：静态几何（圆+墙）不进 QP")
    ap.add_argument("--no-slack", action="store_true", help="消融：障碍约束转硬约束")
    ap.add_argument("--no-terminal", action="store_true", help="消融：关闭终端代价")
    ap.add_argument("--no-plot", action="store_true")
    ap.add_argument("--quiet", action="store_true", help="不打印逐步日志")
    args = ap.parse_args()

    instance_mode = bool(args.instance or args.instances is not None or args.suite)
    if args.runs is not None and not (args.instances is not None or args.suite):
        ap.error("--runs 仅用于 --instances / --suite")
    if args.map_only and not instance_mode:
        ap.error("--map-only 仅用于实例模式（--instance / --instances / --suite）")
    if shutil.which("ros2") is None:
        print("找不到 ros2（需 ROS 环境）——请经 run_w3.sh / run_w4.sh 运行", file=sys.stderr)
        return 1

    chassis = load_chassis(args.chassis)
    ablate = dict(
        N=args.N,
        vo=not args.no_vo,
        esdf=not args.no_esdf,
        slack=not args.no_slack,
        terminal=not args.no_terminal,
    )
    try:
        if instance_mode:
            cfg = MPCConfig.benchmark(**ablate)
            runs = parse_runs_spec(args.runs or "all")
            specs = build_specs(args, runs)
        else:
            cfg = MPCConfig(**ablate)
            specs = build_specs(args, None)
    except (KeyError, ValueError, FileNotFoundError) as exc:
        print(f"配置/实例加载失败: {exc}", file=sys.stderr)
        return 2

    out_dir = Path(args.out)
    map_dir = Path(args.map_dir)
    print(
        f"MPC {'W4 实例（NavFn 真参考）' if instance_mode else 'W3（NavFn 真参考）'} | "
        f"N={cfg.N} dt={cfg.dt}s v_ref={cfg.v_ref} d_safe={cfg.d_safe} | "
        f"chassis={chassis.type} r={chassis.radius} | 共 {len(specs)} 条"
    )

    base_params = {**cfg.params_snapshot(), "preset": "benchmark"} if instance_mode else {}
    rows: list[dict] = []
    all_pass = True
    infra_failures = 0
    map_attempted = 0
    map_fetched = 0

    for spec in specs:
        sc0, meta = spec.sc, spec.meta
        tag = spec.tag
        if meta is not None:
            print(f"\n=== {tag}（{sc0.desc}）===")
            map_yaml = write_map(sc0, map_dir, name=tag).resolve()
            log_dir = out_dir / "w4_logs" / meta.scenario
            seed, run_no = meta.seed, meta.run
        else:
            print(f"\n=== {sc0.name} {sc0.desc}（NavFn 真参考）===")
            map_yaml = (map_dir / sc0.name / f"{sc0.name}.yaml").resolve()
            log_dir = out_dir / "w3_logs"
            seed = run_no = None

        map_attempted += 1
        wps, length, reason = fetch_navfn_path(sc0, map_yaml, log_dir, tag, args)
        if wps is None:
            print(f"  ✗ {reason}")
            infra_failures += 1
            all_pass = False
            if meta is not None:
                write_failure(
                    out_dir, meta.scenario, cfg,
                    seed=seed, run=run_no, failure_class="other", reason=reason,
                    params={**base_params, "instance": tag, "failure_phase": "navfn"},
                )
            rows.append(fail_row(tag, reason))
            continue
        map_fetched += 1

        start_err = float(np.linalg.norm(wps[0] - sc0.start[:2]))
        goal_err = float(np.linalg.norm(wps[-1] - sc0.goal[:2]))
        if goal_err > 0.15:  # 回退：NavFn 提前收径时补目标点（§12 编排回退）
            wps = np.vstack([wps, sc0.goal[:2]])
            print(f"  ! 端点距目标 {goal_err:.2f} m > 0.15，补目标 waypoint")
            goal_err = float(np.linalg.norm(wps[-1] - sc0.goal[:2]))
        print(f"  NavFn {tag}: M={len(wps)}, L={length:.2f} m, "
              f"端点误差 start={start_err:.3f} goal={goal_err:.3f}")

        if args.map_only:
            continue

        # 换源：sim.run / ReferencePath 零改动
        sc = dataclasses.replace(sc0, waypoints=wps)
        res: RunResult = run(sc, chassis, cfg, log_every=0 if args.quiet else 100)
        if meta is not None:
            params = {
                **base_params,
                "instance": tag,
                "navfn_waypoints": int(len(wps)),
                "navfn_length_m": round(length, 3),
            }
            run_dir = write_run(
                out_dir, meta.scenario, cfg, res, seed=seed, run=run_no, params=params
            )
            plot_path = out_dir / "figs" / f"{tag}.png"
        else:
            run_dir = write_run(out_dir, sc.name, cfg, res)
            plot_path = out_dir / f"{sc.name}.png"
        print(f"  run 记录 → {run_dir}")
        rows.append(fmt_row(res))
        all_pass &= res.acceptance_pass() and res.arrived
        if not args.no_plot:
            p = plot_run(sc, chassis, res, plot_path, d_safe=cfg.d_safe)
            print(f"  诊断图 → {p}")

    if args.map_only:
        print(f"\nmap-only：{map_fetched}/{map_attempted} 条 NavFn 路径取到")
        return 0 if map_fetched == map_attempted else 1

    if rows:
        print_table(rows)
    if instance_mode:
        print(
            f"\nW4 实例批量：{len(rows)} 条 run 记录，NavFn/生命周期失败 {infra_failures} 条"
            f"（控制失败作为数据交给汇总，不中断批次）"
        )
        if args.instance:
            return 0 if (infra_failures == 0 and all_pass) else 1
        return 0 if infra_failures == 0 else 1

    print(
        "\n验收②逐步解算成功 ③全程间隙>0 ④Δu总变差有界（NavFn 参考）：",
        "ALL PASS" if all_pass else "FAIL",
        "（①一条命令见 run_w3.sh；到达判定 0.25m/0.25rad）",
    )
    return 0 if all_pass else 1


if __name__ == "__main__":
    raise SystemExit(main())
