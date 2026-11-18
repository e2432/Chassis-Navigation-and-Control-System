#include "mpc_controller/reference.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace mpc_controller
{
namespace
{

constexpr double kPi = 3.14159265358979323846;

double wrapAngle(double angle)
{
  // Mirrors numpy's modulo semantics: result in [-pi, pi).
  double wrapped = std::fmod(angle + kPi, 2.0 * kPi);
  if (wrapped < 0.0) {
    wrapped += 2.0 * kPi;
  }
  return wrapped - kPi;
}

}  // namespace

ReferencePath::ReferencePath(
  const std::vector<Eigen::Vector2d> & waypoints, double goal_yaw,
  double v_ref, int N, double dt)
: waypoints_(waypoints), goal_yaw_(goal_yaw), v_ref_(v_ref), N_(N), dt_(dt)
{
  if (waypoints_.size() < 2) {
    throw std::invalid_argument("ReferencePath: at least two waypoints are required");
  }
  if (v_ref_ <= 0.0 || N_ <= 0 || dt_ <= 0.0) {
    throw std::invalid_argument("ReferencePath: v_ref, N and dt must be positive");
  }

  const std::size_t segments = waypoints_.size() - 1;
  seg_len_.resize(segments);
  seg_dir_.resize(segments);
  cum_.resize(segments + 1);
  cum_[0] = 0.0;

  for (std::size_t i = 0; i < segments; ++i) {
    const Eigen::Vector2d delta = waypoints_[i + 1] - waypoints_[i];
    seg_len_[i] = delta.norm();
    seg_dir_[i] = seg_len_[i] > 1e-12 ? delta / seg_len_[i] : Eigen::Vector2d(1.0, 0.0);
    cum_[i + 1] = cum_[i] + seg_len_[i];
  }
  length_ = cum_.back();
}

Eigen::Vector3d ReferencePath::poseAt(double s) const
{
  if (s >= length_ - 1e-12) {
    const Eigen::Vector2d & goal = waypoints_.back();
    return Eigen::Vector3d(goal.x(), goal.y(), goal_yaw_);
  }

  // numpy.searchsorted(cum, s, side="right") - 1
  auto it = std::upper_bound(cum_.begin(), cum_.end(), s);
  int index = static_cast<int>(it - cum_.begin()) - 1;
  const int last = static_cast<int>(seg_len_.size()) - 1;
  index = std::clamp(index, 0, last);

  const double seg_len = seg_len_[index];
  const double frac = seg_len > 1e-12 ? (s - cum_[index]) / seg_len : 0.0;
  const Eigen::Vector2d xy = waypoints_[index] + seg_dir_[index] * (frac * seg_len);
  const double theta = std::atan2(seg_dir_[index].y(), seg_dir_[index].x());
  return Eigen::Vector3d(xy.x(), xy.y(), theta);
}

void ReferencePath::sequence(
  double t_now, Eigen::MatrixXd & z_ref, Eigen::MatrixXd & u_ref) const
{
  z_ref.setZero(N_ + 1, 3);
  u_ref.setZero(N_, 3);

  std::vector<double> thetas(N_ + 1, 0.0);
  std::vector<bool> pinned(N_ + 1, false);
  for (int k = 0; k <= N_; ++k) {
    const double s_k = std::min(v_ref_ * (t_now + k * dt_), length_);
    const Eigen::Vector3d pose = poseAt(s_k);
    z_ref.row(k) = pose.transpose();
    thetas[k] = pose.z();
    pinned[k] = s_k >= length_ - 1e-12;
  }

  for (int k = 0; k < N_; ++k) {
    if (pinned[k]) {
      u_ref.row(k).setZero();
    } else {
      u_ref(k, 0) = v_ref_;
      u_ref(k, 1) = 0.0;
      u_ref(k, 2) = wrapAngle(thetas[k + 1] - thetas[k]) / dt_;
    }
  }
}

}  // namespace mpc_controller
