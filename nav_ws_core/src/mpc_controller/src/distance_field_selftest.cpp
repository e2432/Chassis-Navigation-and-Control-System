// Self-test for GridDistanceField and its MpcCore constraint rows.

#include "mpc_controller/distance_field.hpp"
#include "mpc_controller/mpc_core.hpp"

#include <cmath>
#include <iostream>
#include <vector>

namespace
{

bool check(bool condition, const char * name)
{
  std::cout << (condition ? "PASS " : "FAIL ") << name << '\n';
  return condition;
}

}  // namespace

int main()
{
  const int width = 40;
  const int height = 40;
  const double resolution = 0.1;
  std::vector<uint8_t> occupied(width * height, 0);
  for (int y = 0; y < height; ++y) {
    occupied[y * width + 20] = 1;  // wall at x = 2.05 m
  }

  mpc_controller::GridDistanceField field;
  field.build(width, height, resolution, 0.0, 0.0, occupied, 5.0);

  bool ok = true;
  double distance = 0.0;
  Eigen::Vector2d gradient = Eigen::Vector2d::Zero();

  ok &= check(field.sample(Eigen::Vector2d(1.0, 1.5), distance, gradient), "sample left");
  ok &= check(std::abs(distance - 1.05) < 0.12, "distance left");
  ok &= check(gradient.x() < -0.9 && std::abs(gradient.y()) < 0.1, "gradient left (away)");

  ok &= check(field.sample(Eigen::Vector2d(3.0, 1.5), distance, gradient), "sample right");
  ok &= check(std::abs(distance - 0.95) < 0.12, "distance right");
  ok &= check(gradient.x() > 0.9 && std::abs(gradient.y()) < 0.1, "gradient right (away)");

  ok &= check(!field.sample(Eigen::Vector2d(10.0, 10.0), distance, gradient), "outside");

  mpc_controller::MpcConfig cfg;
  cfg.N = 4;
  cfg.dt = 0.05;
  mpc_controller::ChassisParams chassis;
  mpc_controller::MpcCore core(cfg, chassis);

  Eigen::MatrixXd zbar(cfg.N + 1, 3);
  Eigen::MatrixXd z_ref(cfg.N + 1, 3);
  Eigen::MatrixXd ubar(cfg.N, 3);
  for (int k = 0; k <= cfg.N; ++k) {
    zbar.row(k) << 0.03 * k, 0.0, 0.0;
    z_ref.row(k) = zbar.row(k);
  }
  for (int k = 0; k < cfg.N; ++k) {
    ubar.row(k) << 0.6, 0.0, 0.0;
  }

  const mpc_controller::QpProblem without = core.buildQp(
    Eigen::Vector3d::Zero(), zbar, ubar, z_ref, {}, Eigen::Vector3d::Zero());
  const mpc_controller::QpProblem with = core.buildQp(
    Eigen::Vector3d::Zero(), zbar, ubar, z_ref, {}, Eigen::Vector3d::Zero(), &field);

  ok &= check(
    with.n_s == without.n_s + cfg.N,
    "field adds N slack variables");
  ok &= check(
    with.A.rows() == without.A.rows() + 2 * cfg.N,
    "field adds N constraint rows + N slack rows");
  const double field_row_magnitude =
    with.A.block(6 * cfg.N, 0, cfg.N, 3 * cfg.N).cwiseAbs().maxCoeff();
  ok &= check(field_row_magnitude > 1e-6, "field rows are active");

  std::cout << (ok ? "RESULT: PASS" : "RESULT: FAIL") << '\n';
  return ok ? 0 : 1;
}
