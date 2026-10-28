#!/usr/bin/env python3
"""EDT sanity（**非验收**，W5~6 排雷）：scipy 栅格 EDT + 有限差分梯度 vs 解析梯度。

目的：W5~6 静态障碍改走 costmap 栅格 EDT 时，梯度行为可预期——
自由空间内 EDT 梯度方向 ≈ 解析径向；栅格化只会让梯度分片常数/台阶化，
QP 线性化仍可用（有限差分补梯度），但要在近障处注意分辨率误差。

用法：python edt_sanity.py
"""
from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
from scipy import ndimage

ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT))

from mpc_proto.distance import Obstacle, clearance


def main() -> int:
    # S1 静障布局：圆盘 (4.0, 0.0) r=0.6 + 底盘 0.3 → EDT 场应对 r_total=0.9 的圆
    obs = Obstacle(1, np.array([4.0, 0.0]), np.zeros(2), 0.6)
    robot_r = 0.3
    r_total = obs.radius + robot_r

    res = 0.02  # m/格
    xs = np.arange(0.0, 8.0 + res, res)
    ys = np.arange(-3.0, 3.0 + res, res)
    X, Y = np.meshgrid(xs, ys, indexing="xy")
    # 占据栅格：障碍内部 = 0，外部 = 1（scipy EDT 对非零体素求距）
    occ = ((X - obs.p0[0]) ** 2 + (Y - obs.p0[1]) ** 2 > obs.radius ** 2).astype(np.uint8)
    # EDT：到最近零体素（障碍内部）的距离，单位格 → 米
    edt = ndimage.distance_transform_edt(occ) * res

    # 解析：外缘间隙 d = ‖p−c‖ − r_total；栅格 EDT 是「到障碍表面距离」，
    # 外缘间隙 = EDT − r_robot（不是 − r_total：EDT 已含障碍半径）
    dist_center = np.hypot(X - obs.p0[0], Y - obs.p0[1])
    d_analytic = dist_center - r_total
    d_edt = edt - robot_r

    # 有限差分梯度（EDT 场）vs 解析梯度（取自由区、离障碍 0.15~2 m 的环带）
    gy, gx = np.gradient(edt, res, res)  # ∇EDT（对 x=轴1？np.gradient 按 axis 序）
    # np.gradient(edt, res, res)：第一维是 y，第二维是 x → grad_y, grad_x
    band = (d_analytic > 0.15) & (d_analytic < 2.0)
    ax = (X[band] - obs.p0[0]) / dist_center[band]
    ay = (Y[band] - obs.p0[1]) / dist_center[band]
    fx, fy = gx[band], gy[band]
    fn = np.hypot(fx, fy) + 1e-12
    cosang = (fx * ax + fy * ay) / fn  # 单位化后夹角余弦

    # 近障内切处（0~0.15 m）EDT 阶梯化更明显，单独报
    near = (d_analytic > 0.0) & (d_analytic <= 0.15)
    if near.any():
        nx_ = (X[near] - obs.p0[0]) / dist_center[near]
        ny_ = (Y[near] - obs.p0[1]) / dist_center[near]
        gfx, gfy = gx[near], gy[near]
        gfn = np.hypot(gfx, gfy) + 1e-12
        cos_near = (gfx * nx_ + gfy * ny_) / gfn
    else:
        cos_near = np.array([np.nan])

    err = np.abs(d_edt - d_analytic)
    free = occ == 1  # 障碍内部 EDT≡0 无意义，只看自由区
    print("EDT sanity（非验收）—— 圆盘障碍 S1 布局，栅格 %.2f m" % res)
    print(f"  EDT vs 解析间隙：max|Δd| = {err[free].max():.4f} m（自由区），"
          f"mean = {err[band].mean():.5f} m")
    print(f"  有限差分梯度 vs 解析径向：环带(0.15~2m) cos∠ 中位 = {np.median(cosang):.4f}，"
          f"P5 = {np.percentile(cosang, 5):.4f}")
    print(f"  近障带(0~0.15m) cos∠ 中位 = {np.nanmedian(cos_near):.4f}（EDT 阶梯化预期更差）")

    # 判定（sanity 口径，非 W2 验收）：环带梯度方向一致（cos P5 > 0.9）、间隙误差 < 2 格
    ok = np.percentile(cosang, 5) > 0.9 and err[free].max() < 2.5 * res
    print("  sanity:", "PASS" if ok else "FAIL（W5~6 需复查分辨率/梯度口径）")
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
