// W6: dump the C++ reference sequence for Python/C++ consistency tests.

#include "mpc_controller/reference.hpp"

#include <fstream>
#include <iomanip>
#include <iostream>
#include <vector>

int main(int argc, char ** argv)
{
  if (argc != 3) {
    std::cerr << "usage: mpc_ref_dump <input> <output>\n";
    return 2;
  }

  std::ifstream input(argv[1]);
  if (!input) {
    std::cerr << "cannot open input: " << argv[1] << '\n';
    return 2;
  }

  int num_waypoints = 0;
  input >> num_waypoints;
  if (!input || num_waypoints < 2) {
    std::cerr << "invalid waypoint count\n";
    return 2;
  }

  std::vector<Eigen::Vector2d> waypoints;
  waypoints.reserve(num_waypoints);
  for (int i = 0; i < num_waypoints; ++i) {
    Eigen::Vector2d point;
    input >> point.x() >> point.y();
    waypoints.push_back(point);
  }

  double goal_yaw = 0.0;
  double v_ref = 0.0;
  int N = 0;
  double dt = 0.0;
  double t_now = 0.0;
  input >> goal_yaw >> v_ref >> N >> dt >> t_now;
  if (!input || N <= 0) {
    std::cerr << "invalid reference header\n";
    return 2;
  }

  mpc_controller::ReferencePath reference(waypoints, goal_yaw, v_ref, N, dt);
  Eigen::MatrixXd z_ref;
  Eigen::MatrixXd u_ref;
  reference.sequence(t_now, z_ref, u_ref);

  std::ofstream output(argv[2]);
  if (!output) {
    std::cerr << "cannot open output\n";
    return 2;
  }
  output << std::setprecision(17);
  output << z_ref.rows() << '\n';
  for (int k = 0; k < z_ref.rows(); ++k) {
    output << z_ref(k, 0) << ' ' << z_ref(k, 1) << ' ' << z_ref(k, 2) << '\n';
  }
  output << u_ref.rows() << '\n';
  for (int k = 0; k < u_ref.rows(); ++k) {
    output << u_ref(k, 0) << ' ' << u_ref(k, 1) << ' ' << u_ref(k, 2) << '\n';
  }
  return 0;
}
