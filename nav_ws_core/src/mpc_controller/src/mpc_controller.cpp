#include "mpc_controller/mpc_controller.hpp"

#include <memory>
#include <stdexcept>
#include <string>
#include <utility>

#include "pluginlib/class_list_macros.hpp"

namespace mpc_controller
{

void MpcController::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent;
  plugin_name_ = std::move(name);
  tf_ = std::move(tf);
  costmap_ros_ = std::move(costmap_ros);

  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error("MpcController: failed to lock parent lifecycle node");
  }
  logger_ = node->get_logger();
  RCLCPP_INFO(
    logger_, "[%s] configure: D1 skeleton only (follower/MPC not implemented yet)",
    plugin_name_.c_str());
}

void MpcController::cleanup()
{
  RCLCPP_INFO(logger_, "[%s] cleanup", plugin_name_.c_str());
  global_plan_ = nav_msgs::msg::Path();
}

void MpcController::activate()
{
  RCLCPP_INFO(logger_, "[%s] activate", plugin_name_.c_str());
}

void MpcController::deactivate()
{
  RCLCPP_INFO(logger_, "[%s] deactivate", plugin_name_.c_str());
}

void MpcController::setPlan(const nav_msgs::msg::Path & path)
{
  global_plan_ = path;
  RCLCPP_DEBUG(
    logger_, "[%s] setPlan: %zu poses", plugin_name_.c_str(), global_plan_.poses.size());
}

geometry_msgs::msg::TwistStamped MpcController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & velocity,
  nav2_core::GoalChecker * goal_checker)
{
  (void)pose;
  (void)velocity;
  (void)goal_checker;

  geometry_msgs::msg::TwistStamped cmd_vel;
  if (costmap_ros_) {
    cmd_vel.header.frame_id = costmap_ros_->getBaseFrameID();
  }
  if (auto node = node_.lock()) {
    cmd_vel.header.stamp = node->now();
  }

  // D1 skeleton: emit zero velocity. Pure-pursuit passthrough arrives in D2,
  // the real MPC in W6.
  return cmd_vel;
}

void MpcController::setSpeedLimit(const double & speed_limit, const bool & percentage)
{
  speed_limit_ = speed_limit;
  speed_limit_is_percentage_ = percentage;
  RCLCPP_INFO(
    logger_, "[%s] setSpeedLimit: %.3f (%s)", plugin_name_.c_str(), speed_limit_,
    speed_limit_is_percentage_ ? "percentage" : "absolute");
}

}  // namespace mpc_controller

PLUGINLIB_EXPORT_CLASS(mpc_controller::MpcController, nav2_core::Controller)
