#ifndef MPC_CONTROLLER__REFERENCE_HPP_
#define MPC_CONTROLLER__REFERENCE_HPP_

#include <Eigen/Dense>

#include <vector>

namespace mpc_controller
{

// Time-varying reference from a polyline path:
//   s(t) = min(v_ref * t, L), truncated at the goal.
// Port of prototype/mpc_numpy/mpc_proto/reference.py (W4 MPC formulation).
class ReferencePath
{
public:
  // waypoints: (M, 2) polyline, M >= 2, including the goal.
  ReferencePath(
    const std::vector<Eigen::Vector2d> & waypoints, double goal_yaw,
    double v_ref, int N, double dt);

  // Returns (N+1) x 3 z_ref and N x 3 u_ref; z_ref row k is at t_now + k*dt.
  void sequence(double t_now, Eigen::MatrixXd & z_ref, Eigen::MatrixXd & u_ref) const;

  double length() const {return length_;}
  int N() const {return N_;}
  double dt() const {return dt_;}
  double vRef() const {return v_ref_;}

private:
  Eigen::Vector3d poseAt(double s) const;

  std::vector<Eigen::Vector2d> waypoints_;
  std::vector<Eigen::Vector2d> seg_dir_;
  std::vector<double> seg_len_;
  std::vector<double> cum_;
  double goal_yaw_{0.0};
  double v_ref_{0.0};
  int N_{0};
  double dt_{0.0};
  double length_{0.0};
};

}  // namespace mpc_controller

#endif  // MPC_CONTROLLER__REFERENCE_HPP_
