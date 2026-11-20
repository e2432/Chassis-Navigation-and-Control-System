#ifndef MPC_CONTROLLER__MPC_CORE_HPP_
#define MPC_CONTROLLER__MPC_CORE_HPP_

#include <Eigen/Dense>

#include <string>
#include <vector>

namespace mpc_controller
{

struct ChassisParams
{
  double radius{0.30};
  double v_min{-1.0};
  double v_max{1.0};
  double vy_max{1.0};
  double omega_max{1.5};
  double acc_max{1.0};
  double alpha_max{2.0};
};

struct MpcConfig
{
  int N{40};
  double dt{0.05};
  double q_lat{4.0};
  double q_long{0.4};
  double q_theta{1.5};
  double r_v{0.05};
  double r_omega{0.02};
  double r_dv{0.20};
  double r_domega{0.10};
  double terminal_factor{2.0};
  double rho_slack{80.0};
  double d_safe{0.30};
  int n_sqp{2};
  double eps_abs{1e-3};
  double eps_rel{1e-3};
  int max_iter{10000};
  bool slack{true};
  bool terminal{true};
};

enum class ObstacleType
{
  Circle,
  Wall,
};

struct Obstacle
{
  ObstacleType type{ObstacleType::Circle};
  int id{0};
  Eigen::Vector2d p0{Eigen::Vector2d::Zero()};
  Eigen::Vector2d v{Eigen::Vector2d::Zero()};
  double radius{0.0};
  double x0{0.0};
  double y0{0.0};
  double x1{0.0};
  double y1{0.0};

  static Obstacle circle(
    int id, const Eigen::Vector2d & p0, const Eigen::Vector2d & v, double radius);
  static Obstacle wall(double x0, double y0, double x1, double y1);
};

struct QpProblem
{
  Eigen::MatrixXd P;
  Eigen::VectorXd q;
  Eigen::MatrixXd A;
  Eigen::VectorXd l;
  Eigen::VectorXd u;
  int n_u{0};
  int n_s{0};
  int n_obs{0};
};

struct QpSolution
{
  bool success{false};
  Eigen::VectorXd x;
  std::string status;
  int iterations{0};
};

struct MpcResult
{
  bool success{false};
  bool fallback{false};
  Eigen::Vector3d u{Eigen::Vector3d::Zero()};
  std::string status;
  int qp_iterations{0};
  double max_slack{0.0};
  int fallback_streak{0};
  bool alarm{false};
};

class MpcCore
{
public:
  MpcCore(const MpcConfig & cfg, const ChassisParams & chassis);

  QpProblem buildQp(
    const Eigen::Vector3d & z0,
    const Eigen::MatrixXd & zbar,
    const Eigen::MatrixXd & ubar,
    const Eigen::MatrixXd & z_ref,
    const std::vector<Obstacle> & obstacles,
    const Eigen::Vector3d & u_prev) const;

  QpSolution solveQp(const QpProblem & qp) const;

  MpcResult solve(
    const Eigen::Vector3d & z0,
    const Eigen::MatrixXd & z_ref,
    const Eigen::MatrixXd & u_ref,
    const std::vector<Obstacle> & obstacles);

  const MpcConfig & config() const {return cfg_;}
  const ChassisParams & chassis() const {return chassis_;}

private:
  MpcConfig cfg_;
  ChassisParams chassis_;
  Eigen::Vector3d u_prev_{Eigen::Vector3d::Zero()};
  int fallback_streak_{0};
  bool alarm_{false};
};

}  // namespace mpc_controller

#endif  // MPC_CONTROLLER__MPC_CORE_HPP_
