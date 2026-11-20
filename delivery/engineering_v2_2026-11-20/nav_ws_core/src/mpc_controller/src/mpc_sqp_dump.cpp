// W6: run one SQP solve (n_sqp iterations) for Python/C++ consistency tests.

#include "mpc_controller/mpc_core.hpp"

#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

int main(int argc, char ** argv)
{
  if (argc != 3) {
    std::cerr << "usage: mpc_sqp_dump <input> <output>\n";
    return 2;
  }

  std::ifstream input(argv[1]);
  if (!input) {
    std::cerr << "cannot open input: " << argv[1] << '\n';
    return 2;
  }

  int N = 0;
  int n_obs = 0;
  double dt = 0.0;
  input >> N >> dt >> n_obs;
  if (!input || N <= 0 || n_obs < 0) {
    std::cerr << "invalid header\n";
    return 2;
  }

  mpc_controller::MpcConfig cfg;
  cfg.N = N;
  cfg.dt = dt;
  int slack = 1;
  int terminal = 1;
  input >> cfg.q_lat >> cfg.q_long >> cfg.q_theta >> cfg.r_v >> cfg.r_omega >>
    cfg.r_dv >> cfg.r_domega >> cfg.terminal_factor >> cfg.rho_slack >>
    cfg.d_safe >> cfg.eps_abs >> cfg.eps_rel >> cfg.max_iter >> cfg.n_sqp >>
    slack >> terminal;
  cfg.slack = slack != 0;
  cfg.terminal = terminal != 0;

  mpc_controller::ChassisParams chassis;
  input >> chassis.radius >> chassis.v_min >> chassis.v_max >> chassis.vy_max >>
    chassis.omega_max >> chassis.acc_max >> chassis.alpha_max;

  Eigen::Vector3d z0;
  input >> z0.x() >> z0.y() >> z0.z();

  Eigen::MatrixXd z_ref(N + 1, 3);
  for (int k = 0; k <= N; ++k) {
    input >> z_ref(k, 0) >> z_ref(k, 1) >> z_ref(k, 2);
  }
  Eigen::MatrixXd u_ref(N, 3);
  for (int k = 0; k < N; ++k) {
    input >> u_ref(k, 0) >> u_ref(k, 1) >> u_ref(k, 2);
  }

  std::vector<mpc_controller::Obstacle> obstacles;
  obstacles.reserve(n_obs);
  for (int i = 0; i < n_obs; ++i) {
    std::string type;
    input >> type;
    if (type == "circle") {
      int id = 0;
      Eigen::Vector2d p0;
      Eigen::Vector2d v;
      double radius = 0.0;
      input >> id >> p0.x() >> p0.y() >> v.x() >> v.y() >> radius;
      obstacles.push_back(mpc_controller::Obstacle::circle(id, p0, v, radius));
    } else if (type == "wall") {
      double x0 = 0.0;
      double y0 = 0.0;
      double x1 = 0.0;
      double y1 = 0.0;
      input >> x0 >> y0 >> x1 >> y1;
      obstacles.push_back(mpc_controller::Obstacle::wall(x0, y0, x1, y1));
    } else {
      std::cerr << "unknown obstacle type: " << type << '\n';
      return 2;
    }
  }
  if (!input) {
    std::cerr << "input ended early\n";
    return 2;
  }

  mpc_controller::MpcCore core(cfg, chassis);
  const mpc_controller::MpcResult result = core.solve(z0, z_ref, u_ref, obstacles);

  std::ofstream output(argv[2]);
  if (!output) {
    std::cerr << "cannot open output\n";
    return 2;
  }
  output << std::setprecision(17);
  output << result.status << '\n';
  output << (result.success ? 1 : 0) << ' ' << (result.fallback ? 1 : 0) << ' ' <<
    result.fallback_streak << ' ' << (result.alarm ? 1 : 0) << ' ' <<
    result.qp_iterations << ' ' << result.max_slack << '\n';
  output << result.u.x() << ' ' << result.u.y() << ' ' << result.u.z() << '\n';
  return 0;
}
