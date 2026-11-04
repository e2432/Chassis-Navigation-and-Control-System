#!/usr/bin/env python3
"""E06 结果分析：按阶段核对“指令方向 = 实际运动方向”，并检查零指令与超时停车。

判定用仿真时间窗内的**机体坐标系位移**（把世界系位移按阶段起始 yaw 旋转回来），
这样即使机器人在测试中转过向，判定依然是“相对机体前后/左右”。
"""

import argparse
import csv
import json
import math
import os

ap = argparse.ArgumentParser()
ap.add_argument("--out", required=True)
a = ap.parse_args()
OUT = os.path.abspath(a.out)


def load(name):
    p = os.path.join(OUT, name)
    if not os.path.exists(p):
        return []
    with open(p, newline="") as f:
        return list(csv.DictReader(f))


drive = load("drive_log.csv")
phases = load("phases.csv")
if not drive or not phases:
    raise SystemExit("缺少 drive_log.csv 或 phases.csv")

for r in drive:
    for k in r:
        try:
            r[k] = float(r[k])
        except (TypeError, ValueError):
            pass


def window(t0, t1):
    return [r for r in drive if t0 <= r["sim_t"] <= t1]


def interp_pose(t):
    """取最近一帧位姿。"""
    best = min(drive, key=lambda r: abs(r["sim_t"] - t))
    return best["x"], best["y"], best["yaw"]


results = []
for ph in phases:
    name = ph["name"]
    t0, t1 = float(ph["t_start"]), float(ph["t_end"])
    rows = window(t0, t1)
    if len(rows) < 3:
        continue
    x0, y0, yaw0 = rows[0]["x"], rows[0]["y"], rows[0]["yaw"]
    x1, y1, yaw1 = rows[-1]["x"], rows[-1]["y"], rows[-1]["yaw"]
    dx_w, dy_w = x1 - x0, y1 - y0
    c, s = math.cos(yaw0), math.sin(yaw0)
    dx_b, dy_b = dx_w * c + dy_w * s, -dx_w * s + dy_w * c      # 世界 -> 阶段起始机体系
    dyaw = (yaw1 - yaw0 + math.pi) % (2 * math.pi) - math.pi
    mvx = sum(r["meas_vx"] for r in rows) / len(rows)
    mvy = sum(r["meas_vy"] for r in rows) / len(rows)
    mwz = sum(r["meas_wz"] for r in rows) / len(rows)
    results.append(dict(name=name, t0=t0, t1=t1, dur=t1 - t0,
                        cmd=(float(ph["cmd_vx"]), float(ph["cmd_vy"]), float(ph["cmd_wz"])),
                        dx_b=dx_b, dy_b=dy_b, dyaw=dyaw,
                        meas_vx=mvx, meas_vy=mvy, meas_wz=mwz,
                        n=len(rows)))

by = {r["name"]: r for r in results}
TOL = 0.04          # 侧向/轴向串扰容差 (m)
YAW_TOL = 0.10      # rad


def check_dir(r, ax, sign, min_disp=0.15):
    """判定沿指定机体轴是否按期望方向运动（ax='x'/'y'），另一轴与 yaw 串扰受限。"""
    if r is None:
        return False, "阶段缺失"
    main = r["dx_b"] if ax == "x" else r["dy_b"]
    cross = abs(r["dy_b"]) if ax == "x" else abs(r["dx_b"])
    ok = (main * sign > min_disp) and cross < TOL and abs(r["dyaw"]) < YAW_TOL
    return ok, "主轴%+.3fm(需%+.2f) 串扰%.3fm 偏航%+.3frad" % (main, sign * min_disp, cross, r["dyaw"])


def check_rot(r, sign, min_yaw=0.45):
    if r is None:
        return False, "阶段缺失"
    pos = math.hypot(r["dx_b"], r["dy_b"])
    ok = (r["dyaw"] * sign > min_yaw) and pos < 0.12
    return ok, "偏航%+.3frad(需%+.2f) 位置漂移%.3fm" % (r["dyaw"], sign * min_yaw, pos)


checks = {}
checks["前进 +0.1 m/s"] = check_dir(by.get("forward_+0.1"), "x", +1)
checks["后退 -0.1 m/s"] = check_dir(by.get("backward_-0.1"), "x", -1)
checks["左移 +0.1 m/s (vy+)"] = check_dir(by.get("left_+0.1"), "y", +1)
checks["右移 -0.1 m/s (vy-)"] = check_dir(by.get("right_-0.1"), "y", -1)
checks["逆时针 +0.3 rad/s"] = check_rot(by.get("ccw_+0.3"), +1)
checks["顺时针 -0.3 rad/s"] = check_rot(by.get("cw_-0.3"), -1)

comb = by.get("combined")
if comb:
    ok = (comb["dx_b"] > 0.15 and comb["dy_b"] > 0.05 and comb["dyaw"] > 0.30
          and comb["meas_vx"] > 0.05 and comb["meas_vy"] > 0.02 and comb["meas_wz"] > 0.10)
    checks["组合运动 (vx+,vy+,ω+)"] = (ok, "机体系位移(%.3f, %.3f)m 偏航%+.3frad" % (comb["dx_b"], comb["dy_b"], comb["dyaw"]))
else:
    checks["组合运动 (vx+,vy+,ω+)"] = (False, "阶段缺失")

zero = by.get("zero_cmd")
if zero:
    mv = math.hypot(zero["meas_vx"], zero["meas_vy"])
    ok = mv < 0.005 and abs(zero["dx_b"]) < 0.01 and abs(zero["dy_b"]) < 0.01 and abs(zero["meas_wz"]) < 0.02
    checks["零指令停车"] = (ok, "残余速度 %.4f m/s，位移 (%.4f, %.4f)m" % (mv, zero["dx_b"], zero["dy_b"]))
else:
    checks["零指令停车"] = (False, "阶段缺失")

# 超时停车：指令停发后，速度应在 0.5 s（+1 帧容差）内归零并保持
drop = by.get("cmd_drop")
wait = by.get("cmd_drop_wait")
if drop:
    t_stop = drop["t1"]
    after = [r for r in drive if t_stop + 0.65 <= r["sim_t"] <= t_stop + 2.0]
    v_after = max((math.hypot(r["meas_vx"], r["meas_vy"]) for r in after), default=9.9)
    timeout_flag = max((r["timeout"] for r in after), default=0)
    early = [r for r in drive if t_stop + 0.15 <= r["sim_t"] <= t_stop + 0.6]
    v_early = max((math.hypot(r["meas_vx"], r["meas_vy"]) for r in early), default=9.9)
    ok = v_after < 0.02 and timeout_flag >= 1
    checks["指令超时停车"] = (ok, "停发后 0.65~2.0s 最大速度 %.4f m/s，timeout 标志=%d，过渡期(0.15~0.6s)最大 %.3f m/s"
                              % (v_after, int(timeout_flag), v_early))
else:
    checks["指令超时停车"] = (False, "阶段缺失")

npass = sum(1 for ok, _ in checks.values() if ok)
summary = {"mode": "kinematic" if "kinematic" in open(os.path.join(OUT, "sim_stdout.log"), "rb").read().decode("utf-8", "ignore")[:4000] else "physics",
           "checks": {k: {"pass": v[0], "detail": v[1]} for k, v in checks.items()},
           "n_pass": npass, "n_total": len(checks),
           "phases": results}
with open(os.path.join(OUT, "summary.json"), "w") as f:
    json.dump(summary, f, indent=2, ensure_ascii=False)

lines = ["E06 全向速度输入检查（%s）" % summary["mode"], "=" * 60]
for k, (ok, detail) in checks.items():
    lines.append("[%s] %-24s %s" % ("PASS" if ok else "FAIL", k, detail))
lines.append("")
lines.append("PASS=%d / %d" % (npass, len(checks)))
lines.append("")
lines.append("%-16s %8s %8s %8s %9s %9s %9s" % ("阶段", "t0", "t1", "机体系Δx", "机体系Δy", "Δyaw", "实测vx"))
for r in results:
    lines.append("%-16s %8.2f %8.2f %8.3f %9.3f %9.3f %9.3f"
                 % (r["name"], r["t0"], r["t1"], r["dx_b"], r["dy_b"], r["dyaw"], r["meas_vx"]))
text = "\n".join(lines)
open(os.path.join(OUT, "summary.txt"), "w").write(text + "\n")
print(text)
