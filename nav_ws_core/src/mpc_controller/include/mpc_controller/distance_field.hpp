#ifndef MPC_CONTROLLER__DISTANCE_FIELD_HPP_
#define MPC_CONTROLLER__DISTANCE_FIELD_HPP_

#include <Eigen/Dense>

#include <vector>

namespace mpc_controller
{

// Static obstacle distance field built from the frozen static map (/map).
// Distances are meters to the nearest occupied cell; the grid is stored in
// the map frame. Sampling uses bilinear interpolation for distance and
// gradient (central differences).
class GridDistanceField
{
public:
  GridDistanceField() = default;

  void build(
    int width, int height, double resolution,
    double origin_x, double origin_y,
    const std::vector<uint8_t> & occupied, double max_distance);

  bool empty() const {return width_ <= 0 || height_ <= 0 || distance_.empty();}

  // Returns false when the point is outside the field.
  bool sample(
    const Eigen::Vector2d & point, double & distance,
    Eigen::Vector2d & gradient) const;

  int width() const {return width_;}
  int height() const {return height_;}
  double resolution() const {return resolution_;}

private:
  double at(int ix, int iy) const;
  double gradientX(int ix, int iy) const;
  double gradientY(int ix, int iy) const;

  int width_{0};
  int height_{0};
  double resolution_{0.0};
  double origin_x_{0.0};
  double origin_y_{0.0};
  std::vector<double> distance_;
};

}  // namespace mpc_controller

#endif  // MPC_CONTROLLER__DISTANCE_FIELD_HPP_
