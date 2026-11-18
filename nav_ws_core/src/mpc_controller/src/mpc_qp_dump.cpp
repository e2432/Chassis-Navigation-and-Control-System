// W6: dump one QP (and its OSQP solution) for Python/C++ consistency tests.

#include "mpc_controller/mpc_core.hpp"

#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

namespace
{

void writeMatrix(std::ostream & out, const Eigen::MatrixXd & matrix)
{
  for (int row = 0; row < matrix.rows(); ++row) {
    for (int col = 0; col < matrix.cols(); ++col) {
      out << matrix(row, col) << ' ';
    }
  }
  out << '\n';
}

void writeVector(std::ostream & out, const Eigen::VectorXd & vector)
{
  for (int i = 0; i < vector.size(); ++i) {
    out << vector[i] << ' ';
  }
  out << '\n';
}

}  // namespace

int main(int argc, char ** argv)
{
  if (argc != 4) {
    std::cerr << "usage: mpc_qp_dump <input> <qp_out> <solution_out>\n";
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
  Eigen::MatrixXd zbar(N + 1, 3);
  for (int k = 0; k <= N; ++k) {
    input >> zbar(k, 0) >> zbar(k, 1) >> zbar(k, 2);
  }
  Eigen::MatrixXd ubar(N, 3);
  for (int k = 0; k < N; ++k) {
    input >> ubar(k, 0) >> ubar(k, 1) >> ubar(k, 2);
  }
  Eigen::Vector3d u_prev;
  input >> u_prev.x() >> u_prev.y() >> u_prev.z();

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

  // Optional static field block:
  //   field <w> <h> <res> <ox> <oy> <r00> <r01> <r10> <r11> <tx> <ty>
  //   <w*h occupancy values>
  mpc_controller::GridDistanceField static_field;
  bool has_field = false;
  Eigen::Matrix2d map_to_odom_rotation = Eigen::Matrix2d::Identity();
  Eigen::Vector2d map_to_odom_translation = Eigen::Vector2d::Zero();
  input.clear();
  std::string extra;
  if (input >> extra) {
    if (extra != "field") {
      std::cerr << "unknown trailing block: " << extra << '\n';
      return 2;
    }
    int field_width = 0;
    int field_height = 0;
    double resolution = 0.0;
    double origin_x = 0.0;
    double origin_y = 0.0;
    input >> field_width >> field_height >> resolution >> origin_x >> origin_y;
    input >> map_to_odom_rotation(0, 0) >> map_to_odom_rotation(0, 1) >>
      map_to_odom_rotation(1, 0) >> map_to_odom_rotation(1, 1) >>
      map_to_odom_translation.x() >> map_to_odom_translation.y();
    if (!input || field_width <= 0 || field_height <= 0 || resolution <= 0.0) {
      std::cerr << "invalid field header\n";
      return 2;
    }
    std::vector<uint8_t> occupied(
      static_cast<std::size_t>(field_width) * field_height, 0);
    for (auto & cell : occupied) {
      int value = 0;
      input >> value;
      cell = value != 0 ? 1 : 0;
    }
    if (!input) {
      std::cerr << "field data ended early\n";
      return 2;
    }
    static_field.build(
      field_width, field_height, resolution, origin_x, origin_y, occupied, 5.0);
    has_field = true;
  }

  const mpc_controller::MpcCore core(cfg, chassis);
  const mpc_controller::QpProblem qp = has_field ?
    core.buildQp(
    z0, zbar, ubar, z_ref, obstacles, u_prev, &static_field,
    map_to_odom_rotation, map_to_odom_translation) :
    core.buildQp(z0, zbar, ubar, z_ref, obstacles, u_prev);
  const mpc_controller::QpSolution solution = core.solveQp(qp);

  std::ofstream qp_out(argv[2]);
  std::ofstream solution_out(argv[3]);
  if (!qp_out || !solution_out) {
    std::cerr << "cannot open output files\n";
    return 2;
  }
  qp_out << std::setprecision(17);
  solution_out << std::setprecision(17);

  qp_out << qp.n_u << ' ' << qp.n_s << ' ' << qp.n_obs << ' ' <<
    qp.P.rows() << ' ' << qp.A.rows() << '\n';
  writeMatrix(qp_out, qp.P);
  writeVector(qp_out, qp.q);
  writeMatrix(qp_out, qp.A);
  writeVector(qp_out, qp.l);
  writeVector(qp_out, qp.u);

  solution_out << solution.status << '\n';
  writeVector(solution_out, solution.x);
  return solution.success ? 0 : 1;
}
