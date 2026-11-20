#include "mpc_controller/mpc_core.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>
#include <vector>

extern "C" {
#include "osqp.h"
}

namespace mpc_controller
{
namespace
{

constexpr double kInf = std::numeric_limits<double>::infinity();
using Matrix3xX = Eigen::Matrix<double, 3, Eigen::Dynamic>;

struct Clearance
{
  double d{0.0};
  Eigen::Vector2d grad{Eigen::Vector2d::Zero()};
};

Clearance clearance(
  const Eigen::Vector2d & p, const Obstacle & obstacle, double tau, double robot_radius)
{
  if (obstacle.type == ObstacleType::Wall) {
    const double cx = 0.5 * (obstacle.x0 + obstacle.x1);
    const double cy = 0.5 * (obstacle.y0 + obstacle.y1);
    const double hx = 0.5 * (obstacle.x1 - obstacle.x0);
    const double hy = 0.5 * (obstacle.y1 - obstacle.y0);
    const double dx = p.x() - cx;
    const double dy = p.y() - cy;
    const double qx = std::abs(dx) - hx;
    const double qy = std::abs(dy) - hy;
    const double sx = dx >= 0.0 ? 1.0 : -1.0;
    const double sy = dy >= 0.0 ? 1.0 : -1.0;

    if (qx > 0.0 || qy > 0.0) {
      const double ox = std::max(qx, 0.0);
      const double oy = std::max(qy, 0.0);
      const double d = std::hypot(ox, oy);
      if (d < 1e-12) {
        return {0.0 - robot_radius, Eigen::Vector2d(sx, 0.0)};
      }
      return {d - robot_radius, Eigen::Vector2d((ox / d) * sx, (oy / d) * sy)};
    }
    if (qx >= qy) {
      return {qx - robot_radius, Eigen::Vector2d(sx, 0.0)};
    }
    return {qy - robot_radius, Eigen::Vector2d(0.0, sy)};
  }

  const Eigen::Vector2d center = obstacle.p0 + obstacle.v * tau;
  const Eigen::Vector2d delta = p - center;
  const double distance = delta.norm();
  const double radius_sum = obstacle.radius + robot_radius;
  if (distance < 1e-9) {
    return {-radius_sum, Eigen::Vector2d(0.0, 1.0)};
  }
  return {distance - radius_sum, delta / distance};
}

void linearize(
  const Eigen::Vector3d & zbar_k,
  const Eigen::Vector3d & ubar_k,
  const Eigen::Vector3d & zbar_next,
  double dt,
  Eigen::Matrix3d & A,
  Eigen::Matrix3d & B,
  Eigen::Vector3d & c)
{
  const double theta = zbar_k.z();
  const double vx = ubar_k.x();
  const double vy = ubar_k.y();
  const double st = std::sin(theta);
  const double ct = std::cos(theta);

  A << 1.0, 0.0, (-vx * st - vy * ct) * dt,
    0.0, 1.0, (vx * ct - vy * st) * dt,
    0.0, 0.0, 1.0;
  B << ct * dt, -st * dt, 0.0,
    st * dt, ct * dt, 0.0,
    0.0, 0.0, dt;
  c = zbar_next - A * zbar_k - B * ubar_k;
}

Eigen::MatrixXd rollout(
  const Eigen::Vector3d & z0, const Eigen::MatrixXd & u_seq, double dt)
{
  const int horizon = static_cast<int>(u_seq.rows());
  Eigen::MatrixXd z(horizon + 1, 3);
  z.row(0) = z0.transpose();
  for (int k = 0; k < horizon; ++k) {
    const double x = z(k, 0);
    const double y = z(k, 1);
    const double theta = z(k, 2);
    const double vx = u_seq(k, 0);
    const double vy = u_seq(k, 1);
    const double omega = u_seq(k, 2);
    z(k + 1, 0) = x + (vx * std::cos(theta) - vy * std::sin(theta)) * dt;
    z(k + 1, 1) = y + (vx * std::sin(theta) + vy * std::cos(theta)) * dt;
    z(k + 1, 2) = theta + omega * dt;
  }
  return z;
}

struct CscData
{
  std::vector<c_int> p;
  std::vector<c_int> i;
  std::vector<c_float> x;
};

CscData denseToCsc(const Eigen::MatrixXd & matrix, bool upper_only)
{
  CscData csc;
  const int rows = static_cast<int>(matrix.rows());
  const int cols = static_cast<int>(matrix.cols());
  csc.p.resize(cols + 1);
  for (int col = 0; col < cols; ++col) {
    csc.p[col] = static_cast<c_int>(csc.x.size());
    for (int row = 0; row < rows; ++row) {
      if (upper_only && row > col) {
        continue;
      }
      const double value = matrix(row, col);
      if (std::abs(value) > 1e-15) {
        csc.i.push_back(static_cast<c_int>(row));
        csc.x.push_back(static_cast<c_float>(value));
      }
    }
  }
  csc.p[cols] = static_cast<c_int>(csc.x.size());
  return csc;
}

}  // namespace

Obstacle Obstacle::circle(
  int id, const Eigen::Vector2d & p0, const Eigen::Vector2d & v, double radius)
{
  Obstacle obstacle;
  obstacle.type = ObstacleType::Circle;
  obstacle.id = id;
  obstacle.p0 = p0;
  obstacle.v = v;
  obstacle.radius = radius;
  return obstacle;
}

Obstacle Obstacle::wall(double x0, double y0, double x1, double y1)
{
  if (!(x0 < x1 && y0 < y1)) {
    throw std::invalid_argument("wall must satisfy x0 < x1 and y0 < y1");
  }
  Obstacle obstacle;
  obstacle.type = ObstacleType::Wall;
  obstacle.x0 = x0;
  obstacle.y0 = y0;
  obstacle.x1 = x1;
  obstacle.y1 = y1;
  return obstacle;
}

MpcCore::MpcCore(const MpcConfig & cfg, const ChassisParams & chassis)
: cfg_(cfg), chassis_(chassis)
{
  if (cfg_.N <= 0 || cfg_.dt <= 0.0) {
    throw std::invalid_argument("MpcCore: N and dt must be positive");
  }
  if (cfg_.q_long > 0.1 * cfg_.q_lat + 1e-12) {
    throw std::invalid_argument("MpcCore: q_long must be <= 0.1 * q_lat");
  }
}

QpProblem MpcCore::buildQp(
  const Eigen::Vector3d & z0,
  const Eigen::MatrixXd & zbar,
  const Eigen::MatrixXd & ubar,
  const Eigen::MatrixXd & z_ref,
  const std::vector<Obstacle> & obstacles,
  const Eigen::Vector3d & u_prev) const
{
  const int N = cfg_.N;
  const double dt = cfg_.dt;
  if (zbar.rows() != N + 1 || zbar.cols() != 3 ||
    z_ref.rows() != N + 1 || z_ref.cols() != 3 ||
    ubar.rows() != N || ubar.cols() != 3)
  {
    throw std::invalid_argument("MpcCore::buildQp: nominal/reference dimensions do not match N");
  }

  const int n_obs = static_cast<int>(obstacles.size());
  const int n_u = 3 * N;
  const int n_s = cfg_.slack ? N * n_obs : 0;
  const int nx = n_u + n_s;

  // 1) LTV recursion: z_k = Xi_k U + xi_k.
  std::vector<Matrix3xX> xi_u(N + 1, Matrix3xX::Zero(3, n_u));
  std::vector<Eigen::Vector3d> xi_b(N + 1, Eigen::Vector3d::Zero());
  xi_b[0] = z0;
  for (int k = 0; k < N; ++k) {
    Eigen::Matrix3d A;
    Eigen::Matrix3d B;
    Eigen::Vector3d c;
    linearize(zbar.row(k).transpose(), ubar.row(k).transpose(), zbar.row(k + 1).transpose(),
      dt, A, B, c);
    xi_u[k + 1] = A * xi_u[k];
    xi_b[k + 1] = A * xi_b[k] + c;
    xi_u[k + 1].block<3, 3>(0, 3 * k) += B;
  }

  // 2) Stage/terminal cost.
  Eigen::MatrixXd H = Eigen::MatrixXd::Zero(n_u, n_u);
  Eigen::VectorXd f = Eigen::VectorXd::Zero(nx);
  const Eigen::Vector3d q_stage(cfg_.q_lat, cfg_.q_long, cfg_.q_theta);
  Eigen::Vector3d q_terminal = Eigen::Vector3d::Zero();
  if (cfg_.terminal) {
    q_terminal = cfg_.terminal_factor * q_stage;
  }

  for (int k = 1; k <= N; ++k) {
    const double theta = z_ref(k, 2);
    const double st = std::sin(theta);
    const double ct = std::cos(theta);
    Eigen::Matrix3d M;
    M << -st, ct, 0.0,
      ct, st, 0.0,
      0.0, 0.0, 1.0;
    const Matrix3xX Ek = M * xi_u[k];
    const Eigen::Vector3d e0 = M * (xi_b[k] - z_ref.row(k).transpose());
    const Eigen::Vector3d & qdiag = (k == N) ? q_terminal : q_stage;
    H += Ek.transpose() * qdiag.asDiagonal() * Ek;
    f.head(n_u) += 2.0 * (Ek.transpose() * (qdiag.asDiagonal() * e0));
  }

  const Eigen::Vector3d r(cfg_.r_v, cfg_.r_v, cfg_.r_omega);
  const Eigen::Vector3d rd(cfg_.r_dv, cfg_.r_dv, cfg_.r_domega);
  for (int k = 0; k < N; ++k) {
    H.block<3, 3>(3 * k, 3 * k) += r.asDiagonal();
  }
  for (int k = 0; k < N; ++k) {
    H.block<3, 3>(3 * k, 3 * k) += rd.asDiagonal();
    if (k == 0) {
      f.segment<3>(0) += -2.0 * (rd.array() * u_prev.array()).matrix();
    } else {
      H.block<3, 3>(3 * (k - 1), 3 * (k - 1)) += rd.asDiagonal();
      H.block<3, 3>(3 * k, 3 * (k - 1)) -= rd.asDiagonal();
      H.block<3, 3>(3 * (k - 1), 3 * k) -= rd.asDiagonal();
    }
  }
  if (n_s > 0) {
    f.tail(n_s).setConstant(cfg_.rho_slack);
  }

  // 3) Constraints: box, rate, soft obstacle, slack >= 0.
  const int m = 6 * N + N * n_obs + n_s;
  Eigen::MatrixXd A = Eigen::MatrixXd::Zero(m, nx);
  Eigen::VectorXd lower(m);
  Eigen::VectorXd upper(m);
  int row = 0;

  const Eigen::Vector3d u_min(chassis_.v_min, -chassis_.vy_max, -chassis_.omega_max);
  const Eigen::Vector3d u_max(chassis_.v_max, chassis_.vy_max, chassis_.omega_max);
  const Eigen::Vector3d du_max(
    chassis_.acc_max * dt, chassis_.acc_max * dt, chassis_.alpha_max * dt);

  for (int k = 0; k < N; ++k) {
    for (int i = 0; i < 3; ++i) {
      A(row, 3 * k + i) = 1.0;
      lower(row) = u_min[i];
      upper(row) = u_max[i];
      ++row;
    }
  }

  for (int k = 0; k < N; ++k) {
    for (int i = 0; i < 3; ++i) {
      A(row, 3 * k + i) = 1.0;
      if (k > 0) {
        A(row, 3 * (k - 1) + i) = -1.0;
        lower(row) = -du_max[i];
        upper(row) = du_max[i];
      } else {
        lower(row) = u_prev[i] - du_max[i];
        upper(row) = u_prev[i] + du_max[i];
      }
      ++row;
    }
  }

  for (int k = 1; k <= N; ++k) {
    const Eigen::Vector2d p_hat = zbar.row(k).head<2>().transpose();
    const double tau = k * dt;
    for (int j = 0; j < n_obs; ++j) {
      const Clearance cl = clearance(p_hat, obstacles[j], tau, chassis_.radius);
      const double rhs = cfg_.d_safe - cl.d + cl.grad.dot(p_hat - xi_b[k].head<2>());
      A.row(row).head(n_u) = cl.grad.transpose() * xi_u[k].topRows(2);
      if (n_s > 0) {
        A(row, n_u + (k - 1) * n_obs + j) = 1.0;
      }
      lower(row) = rhs;
      upper(row) = kInf;
      ++row;
    }
  }

  for (int idx = 0; idx < n_s; ++idx) {
    A(row, n_u + idx) = 1.0;
    lower(row) = 0.0;
    upper(row) = kInf;
    ++row;
  }
  if (row != m) {
    throw std::runtime_error("MpcCore::buildQp: constraint row count mismatch");
  }

  QpProblem qp;
  qp.n_u = n_u;
  qp.n_s = n_s;
  qp.n_obs = n_obs;
  qp.P = Eigen::MatrixXd::Zero(nx, nx);
  qp.P.topLeftCorner(n_u, n_u) = 2.0 * H;
  if (n_s > 0) {
    qp.P.bottomRightCorner(n_s, n_s) =
      1e-8 * Eigen::MatrixXd::Identity(n_s, n_s);
  }
  qp.P = 0.5 * (qp.P + qp.P.transpose());
  qp.q = f;
  qp.A = std::move(A);
  qp.l = std::move(lower);
  qp.u = std::move(upper);
  return qp;
}

QpSolution MpcCore::solveQp(const QpProblem & qp) const
{
  QpSolution solution;
  const int nx = static_cast<int>(qp.P.rows());
  const int m = static_cast<int>(qp.A.rows());
  if (qp.P.cols() != nx || qp.q.size() != nx || qp.A.cols() != nx ||
    qp.l.size() != m || qp.u.size() != m)
  {
    solution.status = "dimension_mismatch";
    return solution;
  }

  CscData P_csc = denseToCsc(qp.P, true);
  CscData A_csc = denseToCsc(qp.A, false);
  std::vector<c_float> q(qp.q.data(), qp.q.data() + qp.q.size());
  std::vector<c_float> lower(m);
  std::vector<c_float> upper(m);
  for (int i = 0; i < m; ++i) {
    lower[i] = std::isinf(qp.l[i]) ?
      (qp.l[i] < 0.0 ? -static_cast<c_float>(OSQP_INFTY) : static_cast<c_float>(OSQP_INFTY)) :
      static_cast<c_float>(qp.l[i]);
    upper[i] = std::isinf(qp.u[i]) ?
      (qp.u[i] < 0.0 ? -static_cast<c_float>(OSQP_INFTY) : static_cast<c_float>(OSQP_INFTY)) :
      static_cast<c_float>(qp.u[i]);
  }

  csc * P = csc_matrix(
    nx, nx, static_cast<c_int>(P_csc.x.size()), P_csc.x.data(), P_csc.i.data(), P_csc.p.data());
  csc * A = csc_matrix(
    m, nx, static_cast<c_int>(A_csc.x.size()), A_csc.x.data(), A_csc.i.data(), A_csc.p.data());

  OSQPData data;
  data.n = nx;
  data.m = m;
  data.P = P;
  data.q = q.data();
  data.A = A;
  data.l = lower.data();
  data.u = upper.data();

  OSQPSettings settings;
  osqp_set_default_settings(&settings);
  settings.verbose = 0;
  settings.eps_abs = static_cast<c_float>(cfg_.eps_abs);
  settings.eps_rel = static_cast<c_float>(cfg_.eps_rel);
  settings.max_iter = cfg_.max_iter;
  settings.polish = 1;
  settings.warm_start = 1;

  OSQPWorkspace * work = nullptr;
  const c_int setup_status = osqp_setup(&work, &data, &settings);
  if (setup_status != 0 || work == nullptr) {
    solution.status = "setup_error_" + std::to_string(static_cast<long long>(setup_status));
    c_free(P);
    c_free(A);
    return solution;
  }

  const c_int solve_status = osqp_solve(work);
  solution.status = (work->info != nullptr) ? work->info->status : "unknown";
  solution.iterations = (work->info != nullptr) ? static_cast<int>(work->info->iter) : 0;
  const c_int status_val = (work->info != nullptr) ? work->info->status_val : OSQP_UNSOLVED;
  solution.success = solve_status == 0 &&
    (status_val == OSQP_SOLVED || status_val == OSQP_SOLVED_INACCURATE);
  if (solution.success && work->x != nullptr) {
    solution.x.resize(nx);
    for (int i = 0; i < nx; ++i) {
      solution.x[i] = work->x[i];
    }
  } else {
    solution.x = Eigen::VectorXd::Zero(nx);
  }

  osqp_cleanup(work);
  c_free(P);
  c_free(A);
  return solution;
}

MpcResult MpcCore::solve(
  const Eigen::Vector3d & z0,
  const Eigen::MatrixXd & z_ref,
  const Eigen::MatrixXd & u_ref,
  const std::vector<Obstacle> & obstacles)
{
  const int N = cfg_.N;
  if (z_ref.rows() != N + 1 || z_ref.cols() != 3 ||
    u_ref.rows() != N || u_ref.cols() != 3)
  {
    throw std::invalid_argument("MpcCore::solve: reference dimensions do not match N");
  }

  Eigen::MatrixXd ubar = u_ref;
  Eigen::MatrixXd zbar = rollout(z0, ubar, cfg_.dt);
  QpSolution result;
  QpSolution last_ok;
  bool has_last_ok = false;
  const int n_sqp = std::max(1, cfg_.n_sqp);

  for (int iteration = 0; iteration < n_sqp; ++iteration) {
    try {
      const QpProblem qp = buildQp(z0, zbar, ubar, z_ref, obstacles, u_prev_);
      result = solveQp(qp);
    } catch (const std::exception & ex) {
      result = QpSolution();
      result.status = std::string("exception:") + ex.what();
      break;
    }
    if (!result.success) {
      break;
    }
    has_last_ok = true;
    last_ok = result;
    for (int k = 0; k < N; ++k) {
      for (int i = 0; i < 3; ++i) {
        ubar(k, i) = result.x[3 * k + i];
      }
    }
    zbar = rollout(z0, ubar, cfg_.dt);
  }

  if (has_last_ok && !result.success) {
    result = last_ok;
  }

  MpcResult out;
  out.status = result.status;
  out.qp_iterations = result.iterations;
  if (result.success && result.x.size() >= 3 * N) {
    u_prev_ = result.x.head<3>();
    fallback_streak_ = 0;
    out.success = true;
    out.u = u_prev_;
    const int n_s = cfg_.slack ? N * static_cast<int>(obstacles.size()) : 0;
    if (n_s > 0 && result.x.size() >= 3 * N + n_s) {
      out.max_slack = result.x.tail(n_s).maxCoeff();
    }
    out.fallback_streak = 0;
    out.alarm = alarm_;
    return out;
  }

  ++fallback_streak_;
  u_prev_.setZero();
  if (fallback_streak_ >= 5) {
    alarm_ = true;
  }
  out.success = false;
  out.fallback = true;
  out.u = Eigen::Vector3d::Zero();
  out.fallback_streak = fallback_streak_;
  out.alarm = alarm_;
  return out;
}

}  // namespace mpc_controller
