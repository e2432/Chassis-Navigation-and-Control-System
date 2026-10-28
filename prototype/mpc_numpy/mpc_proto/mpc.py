"""MPC QP 装配与 OSQP 求解（docs/MPC公式化.md §4~§8）。

七项代价栈 → 一个 QP（condensed）；障碍一律软约束（L1 slack + 大权重）；
limits 全部 box/线性进 QP；OSQP 失败 → 零速回退 + 日志。
"""
from __future__ import annotations

import logging
from dataclasses import dataclass, field

import numpy as np
import osqp
from scipy import sparse

from .chassis import Chassis
from .distance import Obstacle, clearance
from .reference import ReferencePath

log = logging.getLogger("mpc_proto")


@dataclass
class MPCConfig:
    N: int = 40
    dt: float = 0.05
    # 七项代价栈权重（docs/MPC公式化.md §4；q_long ≤ 0.1·q_lat）
    q_lat: float = 4.0
    q_long: float = 0.4
    q_theta: float = 1.5
    r_v: float = 0.05
    r_omega: float = 0.02
    r_dv: float = 0.20
    r_domega: float = 0.10
    terminal_factor: float = 2.0  # ≥ 2× 阶段对应项
    rho_slack: float = 80.0  # L1 线性罚，静/动共档
    d_safe: float = 0.20  # 软目标间隙（外缘）
    n_sqp: int = 2  # 每拍 SQP 迭代
    eps_abs: float = 1e-4
    eps_rel: float = 1e-4
    max_iter: int = 4000

    def Q_stage(self) -> np.ndarray:
        return np.diag([self.q_lat, self.q_long, self.q_theta])

    def Q_terminal(self) -> np.ndarray:
        return self.terminal_factor * self.Q_stage()

    def R(self) -> np.ndarray:
        return np.diag([self.r_v, self.r_v, self.r_omega])

    def Rd(self) -> np.ndarray:
        return np.diag([self.r_dv, self.r_dv, self.r_domega])


@dataclass
class SolveResult:
    u: np.ndarray  # (3,) 本拍控制
    success: bool  # OSQP solved（回退 = False）
    status: str
    fallback: bool
    obj: float = 0.0
    max_slack: float = 0.0
    qp_iters: int = 0
    U: np.ndarray | None = None  # 全时域解（SQP 名义更新用）


def rollout(z0: np.ndarray, u_seq: np.ndarray, dt: float) -> np.ndarray:
    """非线性完美运动学 rollout；u_seq (N,3) → z (N+1,3)。"""
    N = u_seq.shape[0]
    z = np.zeros((N + 1, 3))
    z[0] = z0
    for k in range(N):
        x, y, th = z[k]
        vx, vy, w = u_seq[k]
        z[k + 1, 0] = x + (vx * np.cos(th) - vy * np.sin(th)) * dt
        z[k + 1, 1] = y + (vx * np.sin(th) + vy * np.cos(th)) * dt
        z[k + 1, 2] = th + w * dt
    return z


def _linearize(zbar_k: np.ndarray, ubar_k: np.ndarray, zbar_next: np.ndarray, dt: float):
    """LTV 一阶展开：z_{k+1} ≈ A z_k + B u_k + c（名义序列满足非线性递推）。"""
    th = zbar_k[2]
    vx, vy = ubar_k[0], ubar_k[1]
    st, ct = np.sin(th), np.cos(th)
    A = np.array(
        [
            [1.0, 0.0, (-vx * st - vy * ct) * dt],
            [0.0, 1.0, (vx * ct - vy * st) * dt],
            [0.0, 0.0, 1.0],
        ]
    )
    B = np.array(
        [
            [ct * dt, -st * dt, 0.0],
            [st * dt, ct * dt, 0.0],
            [0.0, 0.0, dt],
        ]
    )
    c = zbar_next - A @ zbar_k - B @ ubar_k
    return A, B, c


class MPCSolver:
    """每拍：SQP 线性化 → condensed QP → OSQP；失败回退零速。"""

    def __init__(self, chassis: Chassis, cfg: MPCConfig):
        self.chassis = chassis
        self.cfg = cfg
        self._u_prev = np.zeros(3)
        self.fallback_streak = 0
        self.alarm = False

    def solve(
        self,
        z0: np.ndarray,
        obstacles: list[Obstacle],
        reference: ReferencePath,
        t_now: float = 0.0,
    ) -> SolveResult:
        cfg, ch = self.cfg, self.chassis
        N, dt = cfg.N, cfg.dt
        z0 = np.asarray(z0, dtype=float).copy()
        z_ref, u_ref = reference.sequence(t_now)

        # 名义序列初始化：前馈 rollout（后续拍可用 warm start，W2 固定前馈起步）
        ubar = u_ref.copy()
        zbar = rollout(z0, ubar, dt)

        result = None
        last_ok = None
        for _ in range(max(1, cfg.n_sqp)):
            result = self._solve_qp(z0, zbar, ubar, z_ref, obstacles)
            if not result.success:
                break
            last_ok = result
            ubar = result.U.reshape(N, 3)  # type: ignore[union-attr]
            zbar = rollout(z0, ubar, dt)

        # SQP 后续迭代失败时沿用上一次成功解，而不是丢弃
        if last_ok is not None and (result is None or not result.success):
            result = last_ok

        assert result is not None
        if result.success:
            self._u_prev = result.u.copy()
            self.fallback_streak = 0
            return result

        # 回退（决策「求解器回退」）：零速 + 日志，下拍重试；连续 5 拍报警
        self.fallback_streak += 1
        self._u_prev = np.zeros(3)
        if self.fallback_streak >= 5:
            self.alarm = True
            log.error("OSQP 连续 %d 拍失败，报警（不切第二控制器）", self.fallback_streak)
        else:
            log.warning("OSQP 失败(%s) → 本拍零速回退，streak=%d", result.status, self.fallback_streak)
        return SolveResult(
            u=np.zeros(3),
            success=False,
            status=result.status,
            fallback=True,
        )

    # ---- QP 装配（condensed，docs/MPC公式化.md §7）----

    def _solve_qp(
        self,
        z0: np.ndarray,
        zbar: np.ndarray,
        ubar: np.ndarray,
        z_ref: np.ndarray,
        obstacles: list[Obstacle],
    ) -> SolveResult:
        cfg, ch = self.cfg, self.chassis
        N, dt = cfg.N, cfg.dt
        n_obs = len(obstacles)
        n_u = 3 * N
        n_s = N * n_obs  # 障碍软约束 slack：k=1..N × j
        nx = n_u + n_s

        # 1) LTV 递推 → z_k = Xi_k U + xi_k（k=0..N）
        Xi = np.zeros((N + 1, 3, n_u))
        xi = np.zeros((N + 1, 3))
        xi[0] = z0
        As, Bs = [], []
        for k in range(N):
            A, B, c = _linearize(zbar[k], ubar[k], zbar[k + 1], dt)
            As.append(A)
            Bs.append(B)
            Xi[k + 1] = A @ Xi[k]
            xi[k + 1] = A @ xi[k] + c
            Xi[k + 1, :, 3 * k : 3 * k + 3] += B

        # 2) 代价：H (U 部分)、f
        H = np.zeros((n_u, n_u))
        f = np.zeros(nx)

        Qs = cfg.Q_stage()
        Qt = cfg.Q_terminal()
        for k in range(1, N + 1):
            th_ref = z_ref[k, 2]
            st, ct = np.sin(th_ref), np.cos(th_ref)
            M = np.array([[-st, ct, 0.0], [ct, st, 0.0], [0.0, 0.0, 1.0]])
            Ek = M @ Xi[k]  # (3, n_u)
            e0k = M @ (xi[k] - z_ref[k])
            Q = Qt if k == N else Qs
            H += Ek.T @ Q @ Ek
            f[:n_u] += 2.0 * (Ek.T @ Q @ e0k)

        # 控制能量 Σ u_kᵀ R u_k
        R = cfg.R()
        for k in range(N):
            H[3 * k : 3 * k + 3, 3 * k : 3 * k + 3] += R

        # 控制增量 Σ Δu_kᵀ R_Δ Δu_k，u_{-1} = 上一拍实发 u_prev
        # k=0:  (u_0−p)ᵀRd(u_0−p)   → H[0,0] += Rd,      f[0] += −2·Rd·p
        # k≥1:  (u_k−u_{k−1})ᵀRd(·) → H[k,k] += Rd, H[k−1,k−1] += Rd, 交叉块 −Rd
        Rd = cfg.Rd()
        u_prev = self._u_prev
        for k in range(N):
            idx = slice(3 * k, 3 * k + 3)
            H[idx, idx] += Rd
            if k == 0:
                f[0:3] += -2.0 * Rd @ u_prev
            else:
                prev = slice(3 * (k - 1), 3 * (k - 1) + 3)
                H[prev, prev] += Rd
                H[idx, prev] += -Rd
                H[prev, idx] += -Rd

        # slack L1 线性罚
        if n_s:
            f[n_u:] = cfg.rho_slack

        # 3) 约束
        rows, lb, ub = [], [], []

        # box：u_min ≤ u_k ≤ u_max
        for k in range(N):
            for i in range(3):
                row = np.zeros(nx)
                row[3 * k + i] = 1.0
                rows.append(row)
                lb.append(ch.u_min()[i])
                ub.append(ch.u_max()[i])

        # 增量界：|u_k − u_{k−1}| ≤ du_max
        du = ch.du_max(dt)
        for k in range(N):
            for i in range(3):
                row = np.zeros(nx)
                row[3 * k + i] = 1.0
                if k > 0:
                    row[3 * (k - 1) + i] = -1.0
                    lo, hi = -du[i], du[i]
                else:
                    lo, hi = u_prev[i] - du[i], u_prev[i] + du[i]
                rows.append(row)
                lb.append(lo)
                ub.append(hi)

        # 障碍软约束：∇dᵀ p_k + s ≥ d_safe − d̂ + ∇dᵀ p̂（k=1..N）
        for k in range(1, N + 1):
            p_hat = zbar[k, :2]
            tau = k * dt
            for j, obs in enumerate(obstacles):
                cl = clearance(p_hat, obs, tau, ch.radius)
                g = cl.grad
                rhs = cfg.d_safe - cl.d + float(g @ (p_hat - xi[k, :2]))
                row = np.zeros(nx)
                row[:n_u] = g @ Xi[k, :2, :]
                row[n_u + (k - 1) * n_obs + j] = 1.0
                rows.append(row)
                lb.append(rhs)
                ub.append(np.inf)

        # slack ≥ 0
        for idx_s in range(n_s):
            row = np.zeros(nx)
            row[n_u + idx_s] = 1.0
            rows.append(row)
            lb.append(0.0)
            ub.append(np.inf)

        A_c = np.vstack(rows) if rows else np.zeros((0, nx))
        l_c = np.array(lb)
        u_c = np.array(ub)

        # 4) OSQP（min ½ xᵀ P x + qᵀ x；x = [U; S]，slack 块 P=0、线性罚在 q）
        P = np.zeros((nx, nx))
        P[:n_u, :n_u] = 2.0 * H
        if n_s:
            P[n_u:, n_u:] = 1e-8 * np.eye(n_s)  # slack 块微正则，利于数值
        P = 0.5 * (P + P.T)
        q = f
        prob = osqp.OSQP()
        try:
            prob.setup(
                P=sparse.csc_matrix(P),
                q=q,
                A=sparse.csc_matrix(A_c),
                l=l_c,
                u=u_c,
                verbose=False,
                eps_abs=cfg.eps_abs,
                eps_rel=cfg.eps_rel,
                max_iter=cfg.max_iter,
                polish=True,
            )
            res = prob.solve()
        except Exception as exc:  # noqa: BLE001 —— 求解器异常一律走回退
            log.error("OSQP 异常: %s", exc)
            return SolveResult(u=np.zeros(3), success=False, status=f"exception:{exc}", fallback=False)

        status = res.info.status
        solved = status in ("solved", "solved inaccurate") and res.x is not None
        if not solved:
            return SolveResult(u=np.zeros(3), success=False, status=status, fallback=False)

        U = res.x[:n_u]
        s_vals = res.x[n_u:]
        max_slack = float(np.max(s_vals)) if n_s else 0.0
        return SolveResult(
            u=U[:3].copy(),
            success=True,
            status=status,
            fallback=False,
            obj=float(res.info.obj_val),
            max_slack=max_slack,
            qp_iters=int(res.info.iter),
            U=U,
        )
