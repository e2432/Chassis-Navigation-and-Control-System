#include "mpc_controller/distance_field.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace mpc_controller
{

void GridDistanceField::build(
  int width, int height, double resolution,
  double origin_x, double origin_y,
  const std::vector<uint8_t> & occupied, double max_distance)
{
  if (width <= 0 || height <= 0 || resolution <= 0.0) {
    throw std::invalid_argument("GridDistanceField: invalid grid dimensions");
  }
  if (occupied.size() != static_cast<std::size_t>(width) * height) {
    throw std::invalid_argument("GridDistanceField: occupied size mismatch");
  }

  width_ = width;
  height_ = height;
  resolution_ = resolution;
  origin_x_ = origin_x;
  origin_y_ = origin_y;

  const double cap = std::max(0.0, max_distance);
  distance_.assign(static_cast<std::size_t>(width) * height, cap);
  for (std::size_t index = 0; index < occupied.size(); ++index) {
    if (occupied[index] != 0) {
      distance_[index] = 0.0;
    }
  }

  // Chamfer distance transform (1 / sqrt(2) weights), sufficient for the
  // MPC clearance constraints; not an exact Euclidean DT.
  const double diagonal = resolution * std::sqrt(2.0);
  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      const std::size_t index = static_cast<std::size_t>(y) * width + x;
      double value = distance_[index];
      if (x > 0) {
        value = std::min(value, distance_[index - 1] + resolution);
      }
      if (y > 0) {
        value = std::min(value, distance_[index - width] + resolution);
        if (x > 0) {
          value = std::min(value, distance_[index - width - 1] + diagonal);
        }
        if (x + 1 < width) {
          value = std::min(value, distance_[index - width + 1] + diagonal);
        }
      }
      distance_[index] = std::min(value, cap);
    }
  }
  for (int y = height - 1; y >= 0; --y) {
    for (int x = width - 1; x >= 0; --x) {
      const std::size_t index = static_cast<std::size_t>(y) * width + x;
      double value = distance_[index];
      if (x + 1 < width) {
        value = std::min(value, distance_[index + 1] + resolution);
      }
      if (y + 1 < height) {
        value = std::min(value, distance_[index + width] + resolution);
        if (x + 1 < width) {
          value = std::min(value, distance_[index + width + 1] + diagonal);
        }
        if (x > 0) {
          value = std::min(value, distance_[index + width - 1] + diagonal);
        }
      }
      distance_[index] = std::min(value, cap);
    }
  }
}

double GridDistanceField::at(int ix, int iy) const
{
  ix = std::clamp(ix, 0, width_ - 1);
  iy = std::clamp(iy, 0, height_ - 1);
  return distance_[static_cast<std::size_t>(iy) * width_ + ix];
}

double GridDistanceField::gradientX(int ix, int iy) const
{
  const int left = std::max(ix - 1, 0);
  const int right = std::min(ix + 1, width_ - 1);
  const int span = right - left;
  if (span == 0) {
    return 0.0;
  }
  return (at(right, iy) - at(left, iy)) / (span * resolution_);
}

double GridDistanceField::gradientY(int ix, int iy) const
{
  const int lower = std::max(iy - 1, 0);
  const int upper = std::min(iy + 1, height_ - 1);
  const int span = upper - lower;
  if (span == 0) {
    return 0.0;
  }
  return (at(ix, upper) - at(ix, lower)) / (span * resolution_);
}

bool GridDistanceField::sample(
  const Eigen::Vector2d & point, double & distance,
  Eigen::Vector2d & gradient) const
{
  if (empty()) {
    return false;
  }

  // Cell centers sit at origin + (index + 0.5) * resolution.
  const double fx = (point.x() - origin_x_) / resolution_ - 0.5;
  const double fy = (point.y() - origin_y_) / resolution_ - 0.5;
  if (fx < -0.5 || fx > width_ - 0.5 || fy < -0.5 || fy > height_ - 0.5) {
    return false;
  }

  const double cx = std::clamp(fx, 0.0, static_cast<double>(width_ - 1));
  const double cy = std::clamp(fy, 0.0, static_cast<double>(height_ - 1));
  const int ix = static_cast<int>(std::floor(cx));
  const int iy = static_cast<int>(std::floor(cy));
  const int ix1 = std::min(ix + 1, width_ - 1);
  const int iy1 = std::min(iy + 1, height_ - 1);
  const double tx = cx - ix;
  const double ty = cy - iy;

  auto bilinear = [&](double v00, double v10, double v01, double v11) {
      const double top = v00 * (1.0 - tx) + v10 * tx;
      const double bottom = v01 * (1.0 - tx) + v11 * tx;
      return top * (1.0 - ty) + bottom * ty;
    };

  distance = bilinear(
    at(ix, iy), at(ix1, iy), at(ix, iy1), at(ix1, iy1));
  gradient.x() = bilinear(
    gradientX(ix, iy), gradientX(ix1, iy),
    gradientX(ix, iy1), gradientX(ix1, iy1));
  gradient.y() = bilinear(
    gradientY(ix, iy), gradientY(ix1, iy),
    gradientY(ix, iy1), gradientY(ix1, iy1));

  const double norm = gradient.norm();
  if (norm > 1e-9) {
    gradient /= norm;
  }
  return true;
}

}  // namespace mpc_controller
