#ifndef MPC_CONTROLLER__MPC_CONTROLLER_HPP_
#define MPC_CONTROLLER__MPC_CONTROLLER_HPP_

#include <memory>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_core/controller.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "tf2_ros/buffer.h"

namespace mpc_controller
{

class MpcController : public nav2_core::Controller
{
public:
  MpcController() = default;
  ~MpcController() override = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void cleanup() override;
  void activate() override;
  void deactivate() override;

  void setPlan(const nav_msgs::msg::Path & path) override;

  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker) override;

  void setSpeedLimit(const double & speed_limit, const bool & percentage) override;

private:
  bool findLookaheadPoint(
    const geometry_msgs::msg::Pose & robot_pose,
    double & look_x, double & look_y) const;
  double effectiveMaxLinearSpeed() const;

  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::string plugin_name_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  nav_msgs::msg::Path global_plan_;

  // Chassis parameters (contract schema, plan A: FollowPath subkeys).
  std::string chassis_type_{"omni"};
  double robot_radius_{0.30};
  double v_max_{1.0};
  double v_min_{-1.0};
  double vy_max_{1.0};
  double omega_max_{1.5};
  double acc_max_{1.0};
  double alpha_max_{2.0};

  // Minimal passthrough follower parameters (replaced by MPC in W6).
  double desired_linear_vel_{0.6};
  double lookahead_dist_{0.5};
  double rotation_gain_{2.0};

  double speed_limit_{0.0};  // nav2_costmap_2d::NO_SPEED_LIMIT
  bool speed_limit_is_percentage_{false};
  rclcpp::Logger logger_{rclcpp::get_logger("MpcController")};
};

}  // namespace mpc_controller

#endif  // MPC_CONTROLLER__MPC_CONTROLLER_HPP_
