"""模拟诊断图（matplotlib PNG）。

配色用已校验默认色板的分类槽位（固定顺序：机器人=蓝1、静态障碍=青2、动态障碍=黄3），
参考线用弱化灰（chrome）；状态色仅用于阈值线。单轴原则：线速度与角速度分面，不双轴。
"""
from __future__ import annotations

from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

# 色板（references/palette.md，light mode）
SURFACE = "#fcfcfb"
INK = "#0b0b0b"
INK_2 = "#52514e"
MUTED = "#898781"
GRID = "#e1e0d9"
BASELINE = "#c3c2b7"
S1 = "#2a78d6"  # 蓝：机器人轨迹 / 横向序列
S2 = "#1baf7a"  # 青：静态障碍（sub-3:1 → 直接标注）
S3 = "#eda100"  # 黄：动态障碍（sub-3:1 → 直接标注）
CRITICAL = "#d03b3b"  # 状态色：零间隙硬边界（带标签，非仅颜色）

plt.rcParams.update(
    {
        "figure.facecolor": SURFACE,
        "axes.facecolor": SURFACE,
        "axes.edgecolor": BASELINE,
        "axes.labelcolor": INK_2,
        "axes.titlecolor": INK,
        "xtick.color": MUTED,
        "ytick.color": MUTED,
        "grid.color": GRID,
        "grid.linewidth": 0.8,
        "text.color": INK,
        "font.size": 10,
        "axes.titlesize": 11,
        "legend.frameon": False,
        "font.sans-serif": ["Noto Sans CJK SC", "Noto Sans CJK JP", "DejaVu Sans"],
        "axes.unicode_minus": False,
    }
)


def _style(ax):
    ax.grid(True, which="major", linestyle="-", alpha=0.9)
    ax.set_axisbelow(True)
    for side in ("top", "right"):
        ax.spines[side].set_visible(False)


def plot_run(scenario, chassis, res, out_path: Path):
    """单场景诊断图：轨迹俯视 / 间隙时序 / 控制量时序（速度、角速度分面）。"""
    fig = plt.figure(figsize=(12, 9))
    gs = fig.add_gridspec(3, 2, height_ratios=[1.2, 1.0, 1.0], hspace=0.35, wspace=0.25)
    ax_traj = fig.add_subplot(gs[:, 0])
    ax_clear = fig.add_subplot(gs[0, 1])
    ax_vel = fig.add_subplot(gs[1, 1])
    ax_omega = fig.add_subplot(gs[2, 1], sharex=ax_vel)

    # ---- 轨迹俯视 ----
    wp = np.asarray(scenario.waypoints, dtype=float)
    ax_traj.plot(
        wp[:, 0], wp[:, 1],
        color=MUTED, lw=1.6, linestyle=(0, (5, 4)), label="参考折线", zorder=2,
    )
    if res.zs.shape[0]:
        ax_traj.plot(res.zs[:, 0], res.zs[:, 1], color=S1, lw=1.8, label="机器人轨迹", zorder=4)
    # 动态障碍拖尾只画到推演结束（避免按 t_max 画满把坐标轴拉爆）
    t_end = float(res.t_ctrl[-1]) if res.t_ctrl.size else scenario.t_max
    pts = [wp, res.zs[:, :2] if res.zs.shape[0] else wp]
    for obs in scenario.obstacles:
        c0 = obs.p0
        col = S2 if not obs.dynamic else S3
        if obs.dynamic:
            ts = np.arange(0.0, t_end + 1e-9, 1.0)
            trail = np.array([obs.center_at(t) for t in ts])
            ax_traj.plot(trail[:, 0], trail[:, 1], color=col, lw=1.2, alpha=0.85, zorder=3)
            ax_traj.scatter(trail[::2, 0], trail[::2, 1], s=18, color=col, zorder=3)
            pts.append(trail)
        circ = plt.Circle(c0, obs.radius, facecolor=col, edgecolor=col, alpha=0.35, lw=1.4, zorder=1)
        ax_traj.add_patch(circ)
        pts.append(np.array([[c0[0] - obs.radius, c0[1] - obs.radius], [c0[0] + obs.radius, c0[1] + obs.radius]]))
        ax_traj.annotate(
            ("静障" if not obs.dynamic else "动障") + f"#{obs.obs_id}",
            (c0[0], c0[1] + obs.radius + 0.12),
            color=INK_2, fontsize=9, ha="center",
        )
    ax_traj.scatter(*scenario.start[:2], marker="o", s=55, facecolor="none", edgecolor=INK, zorder=5)
    ax_traj.annotate("起点", scenario.start[:2] + np.array([0.15, -0.4]), color=INK, fontsize=9)
    ax_traj.scatter(*scenario.goal[:2], marker="*", s=110, color=INK, zorder=5)
    ax_traj.annotate("目标", scenario.goal[:2] + np.array([0.15, -0.4]), color=INK, fontsize=9)
    all_pts = np.vstack(pts)
    pad = 0.6
    ax_traj.set_xlim(all_pts[:, 0].min() - pad, all_pts[:, 0].max() + pad)
    ax_traj.set_ylim(all_pts[:, 1].min() - pad, all_pts[:, 1].max() + pad)
    ax_traj.set_aspect("equal", adjustable="box")
    ax_traj.set_xlabel("x [m]")
    ax_traj.set_ylabel("y [m]")
    ax_traj.set_title(f"{scenario.name} 轨迹俯视（{scenario.desc}）")
    _style(ax_traj)
    ax_traj.legend(loc="upper right")

    # ---- 间隙时序 ----
    if res.clearances.size:
        t_c = np.arange(res.clearances.size) * 0.01  # dt_sim = 0.01
        ax_clear.plot(t_c, res.clearances, color=S1, lw=1.8)
        ax_clear.axhline(0.0, color=CRITICAL, lw=1.4, linestyle=(0, (4, 3)), label="0（接触边界）")
        ax_clear.axhline(0.20, color=MUTED, lw=1.2, linestyle=(0, (2, 3)), label="d_safe=0.20")
    ax_clear.set_ylabel("外缘间隙 [m]")
    ax_clear.set_xlabel("t [s]")
    ax_clear.set_title("全程间隙（真值几何）")
    _style(ax_clear)
    ax_clear.legend(loc="lower right")

    # ---- 控制量：线速度 / 角速度分面（不双轴）----
    if res.us.shape[0]:
        t_u = res.t_ctrl
        ax_vel.plot(t_u, res.us[:, 0], color=S1, lw=1.8, label="v_x")
        ax_vel.plot(t_u, res.us[:, 1], color=S2, lw=1.8, label="v_y")
        ax_omega.plot(t_u, res.us[:, 2], color=S3, lw=1.8)
    ax_vel.axhline(0, color=BASELINE, lw=1.0)
    ax_omega.axhline(0, color=BASELINE, lw=1.0)
    ax_vel.set_ylabel("线速度 [m/s]")
    ax_vel.set_title("控制量：线速度")
    _style(ax_vel)
    ax_vel.legend(loc="lower right")
    ax_omega.set_ylabel("ω [rad/s]")
    ax_omega.set_xlabel("t [s]")
    ax_omega.set_title("控制量：角速度")
    _style(ax_omega)

    out_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_path, dpi=130, bbox_inches="tight", facecolor=SURFACE)
    plt.close(fig)
    return out_path
