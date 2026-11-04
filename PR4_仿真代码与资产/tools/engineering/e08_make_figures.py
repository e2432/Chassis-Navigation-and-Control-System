#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""E08 出图：把记录下来的点云与 IMU 画成"RViz 等价"的检查图。

产出（results/w2/e08/）：
  cloud_topview.png   —— 雷达坐标系俯视图：数帧点云叠加（颜色=时间），能直观看环境结构
  cloud_frames.png    —— 4 个关键时刻（静止/转 90°/前进/侧移）的单帧点云对比
  imu_series.png      —— IMU 三轴加速度 + 角速度随时间，标出转向段

用法: python3 tools/engineering/e08_make_figures.py --dir results/w2/e08
"""
import argparse
import csv
import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

plt.rcParams["font.sans-serif"] = ["Noto Sans CJK JP", "Noto Sans CJK SC", "Noto Serif CJK SC",
                                   "AR PL UMing TW MBE", "DejaVu Sans"]
plt.rcParams["axes.unicode_minus"] = False


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dir", default="results/w2/e08")
    a = ap.parse_args()

    d = np.load(os.path.join(a.dir, "cloud_sample.npz"), allow_pickle=True)
    stamps = d["stamps"]
    clouds = list(d["clouds"])

    # ---- 1. 俯视图：多帧叠加，颜色=仿真时间 ----
    fig, ax = plt.subplots(figsize=(9, 9))
    cmap = plt.get_cmap("viridis")
    idx = np.linspace(0, len(clouds) - 1, 40).astype(int)
    for n, i in enumerate(idx):
        P = np.asarray(clouds[i])
        if len(P) == 0:
            continue
        ax.scatter(P[:, 0], P[:, 1], s=0.6, color=cmap(n / max(1, len(idx) - 1)),
                   alpha=0.5, linewidths=0)
    ax.plot(0, 0, "r*", ms=18, label="lidar (lidar_front_link)")
    ax.add_patch(plt.Circle((0, 0), 0.33, fill=False, color="r", ls="--", lw=1))
    ax.text(0, -0.45, "车体包络 ~0.66 m", color="r", ha="center", fontsize=8)
    ax.set_aspect("equal")
    ax.set_xlabel("x [m]（雷达前方）")
    ax.set_ylabel("y [m]（左）")
    ax.set_title("E08 /lidar/points 俯视图（40 帧叠加，颜色=时间先后）")
    ax.grid(alpha=0.3)
    sm = plt.cm.ScalarMappable(cmap=cmap, norm=plt.Normalize(stamps[idx[0]], stamps[idx[-1]]))
    cb = fig.colorbar(sm, ax=ax, shrink=0.7)
    cb.set_label("仿真时间 [s]")
    ax.legend(loc="upper right")
    ax.set_xlim(-6, 6)
    ax.set_ylim(-6, 6)
    fig.tight_layout()
    fig.savefig(os.path.join(a.dir, "cloud_topview.png"), dpi=130)
    plt.close(fig)

    # ---- 2. 关键时刻单帧对比 ----
    picks = [0, len(clouds) // 3, len(clouds) // 2, int(len(clouds) * 0.75)]
    fig, axes = plt.subplots(2, 2, figsize=(11, 10))
    for ax2, i in zip(axes.ravel(), picks):
        P = np.asarray(clouds[i])
        if len(P):
            ax2.scatter(P[:, 0], P[:, 1], s=1.2, c=P[:, 2], cmap="plasma", linewidths=0)
        ax2.plot(0, 0, "r*", ms=14)
        ax2.set_aspect("equal")
        ax2.set_xlim(-6, 6)
        ax2.set_ylim(-6, 6)
        ax2.grid(alpha=0.3)
        ax2.set_title("t=%.1f s, %d 点" % (stamps[i], len(P)), fontsize=10)
        ax2.set_xlabel("x [m]")
        ax2.set_ylabel("y [m]")
    fig.suptitle("E08 单帧点云（颜色=z 高度，红*=雷达位置）")
    fig.tight_layout()
    fig.savefig(os.path.join(a.dir, "cloud_frames.png"), dpi=130)
    plt.close(fig)

    # ---- 3. IMU 时序 ----
    rows = list(csv.DictReader(open(os.path.join(a.dir, "imu_frames.csv"))))
    t = np.array([float(r["stamp_sec"]) for r in rows])
    t -= t[0]
    ax3, ax4 = plt.subplots if False else (None, None)
    fig, (ax3, ax4) = plt.subplots(2, 1, figsize=(11, 7), sharex=True)
    ax3.plot(t, [float(r["ax"]) for r in rows], lw=0.8, label="ax")
    ax3.plot(t, [float(r["ay"]) for r in rows], lw=0.8, label="ay")
    ax3.plot(t, [float(r["az"]) for r in rows], lw=0.8, label="az")
    ax3.axhline(9.81, color="k", ls=":", lw=0.8)
    ax3.text(t[-1] * 0.02, 9.83, "g = 9.81", fontsize=8)
    ax3.set_ylabel("线加速度 [m/s²]")
    ax3.set_title("E08 /imu（含重力；静止段 az≈9.81，ax≈ay≈0）")
    ax3.legend(loc="lower right", fontsize=8)
    ax3.grid(alpha=0.3)
    ax4.plot(t, [float(r["wx"]) for r in rows], lw=0.8, label="wx")
    ax4.plot(t, [float(r["wy"]) for r in rows], lw=0.8, label="wy")
    ax4.plot(t, [float(r["wz"]) for r in rows], lw=0.8, label="wz")
    ax4.axhline(0, color="k", lw=0.5)
    ax4.set_ylabel("角速度 [rad/s]")
    ax4.set_xlabel("时间 [s]（相对首帧）")
    ax4.legend(loc="lower right", fontsize=8)
    ax4.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(os.path.join(a.dir, "imu_series.png"), dpi=130)
    plt.close(fig)
    print("已写出 cloud_topview.png / cloud_frames.png / imu_series.png 到 %s" % a.dir)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
