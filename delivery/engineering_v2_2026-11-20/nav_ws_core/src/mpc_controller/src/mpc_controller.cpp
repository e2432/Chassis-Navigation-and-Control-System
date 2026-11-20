#include "mpc_controller/mpc_controller.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>

#include "nav2_costmap_2d/costmap_filters/filter_values.hpp"
#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "tf2/exceptions.h"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

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

  const std::string prefix = plugin_name_ + ".";
  nav2_util::declare_parameter_if_not_declared(
    node, prefix + "type", rclcpp::ParameterValue("omni"));
  nav2_util::declare_parameter_if_not_declared(
    node, prefix + "footprint.radius", rclcpp::ParameterValue(0.30));
  nav2_util::declare_parameter_if_not_declared(
    node, prefix + "limits.v_max", rclcpp::ParameterValue(1.0));
  nav2_util::declare_parameter_if_not_declared(
    node, prefix + "limits.v_min", rclcpp::ParameterValue(-1.0));
  nav2_util::declare_parameter_if_not_declared(
    node, prefix + "limits.vy_max", rclcpp::ParameterValue(1.0));
  nav2_util::declare_parameter_if_not_declared(
    node, prefix + "limits.omega_max", rclcpp::ParameterValue(1.5));
  nav2_util::declare_parameter_if_not_declared(
    node, prefix + "limits.acc_max", rclcpp::ParameterValue(1.0));
  nav2_util::declare_parameter_if_not_declared(
    node, prefix + "limits.alpha_max", rclcpp::ParameterValue(2.0));
  nav2_util::declare_parameter_if_not_declared(
    node, prefix + "desired_linear_vel", rclcpp::ParameterValue(0.6));
  nav2_util::declare_parameter_if_not_declared(
    node, prefix + "lookahead_dist", rclcpp::ParameterValue(0.5));
  nav2_util::declare_parameter_if_not_declared(
    node, prefix + "rotation_gain", rclcpp::ParameterValue(2.0));
  nav2_util::declare_parameter_if_not_declared(
    node, prefix + "obstacle_timeout", rclcpp::ParameterValue(0.5));
  nav2_util::declare_parameter_if_not_declared(
    node, prefix + "transform_tolerance", rclcpp::ParameterValue(0.2));

  chassis_type_ = node->get_parameter(prefix + "type").as_string();
  robot_radius_ = node->get_parameter(prefix + "footprint.radius").as_double();
  v_max_ = node->get_parameter(prefix + "limits.v_max").as_double();
  v_min_ = node->get_parameter(prefix + "limits.v_min").as_double();
  vy_max_ = node->get_parameter(prefix + "limits.vy_max").as_double();
  omega_max_ = node->get_parameter(prefix + "limits.omega_max").as_double();
  acc_max_ = node->get_parameter(prefix + "limits.acc_max").as_double();
  alpha_max_ = node->get_parameter(prefix + "limits.alpha_max").as_double();
  desired_linear_vel_ = node->get_parameter(prefix + "desired_linear_vel").as_double();
  lookahead_dist_ = node->get_parameter(prefix + "lookahead_dist").as_double();
  rotation_gain_ = node->get_parameter(prefix + "rotation_gain").as_double();
  obstacle_timeout_ = node->get_parameter(prefix + "obstacle_timeout").as_double();
  transform_tolerance_ = node->get_parameter(prefix + "transform_tolerance").as_double();

  if (chassis_type_ != "omni" && chassis_type_ != "mecanum" && chassis_type_ != "diff") {
    throw std::runtime_error(
      "MpcController: unsupported chassis type '" + chassis_type_ +
      "' (expected omni | mecanum | diff)");
  }
  if (chassis_type_ == "diff") {
    vy_max_ = 0.0;
  }
  if (v_max_ <= 0.0 || vy_max_ < 0.0 || omega_max_ <= 0.0 ||
    desired_linear_vel_ <= 0.0 || lookahead_dist_ <= 0.0 || rotation_gain_ <= 0.0 ||
    obstacle_timeout_ <= 0.0 || transform_tolerance_ <= 0.0)
  {
    throw std::runtime_error("MpcController: non-positive value in chassis/follower parameters");
  }

  obstacle_sub_ = node->create_subscription<mpc_controller::msg::ObstacleStateArray>(
    "/mpc/obstacles", rclcpp::QoS(10),
    std::bind(&MpcController::obstacleCallback, this, std::placeholders::_1));
  has_valid_obstacle_msg_ = false;
  last_obstacle_received_ = node->now();

  RCLCPP_INFO(
    logger_,
    "[%s] chassis type=%s radius=%.2f v=[%.2f, %.2f] vy_max=%.2f omega_max=%.2f",
    plugin_name_.c_str(), chassis_type_.c_str(), robot_radius_, v_min_, v_max_, vy_max_,
    omega_max_);
  RCLCPP_INFO(
    logger_,
    "[%s] follower: desired_linear_vel=%.2f lookahead_dist=%.2f rotation_gain=%.2f "
    "obstacle_timeout=%.2f transform_tolerance=%.2f",
    plugin_name_.c_str(), desired_linear_vel_, lookahead_dist_, rotation_gain_,
    obstacle_timeout_, transform_tolerance_);
}

void MpcController::cleanup()
{
  RCLCPP_INFO(logger_, "[%s] cleanup", plugin_name_.c_str());
  {
    std::lock_guard<std::mutex> lock(obstacle_mutex_);
    has_valid_obstacle_msg_ = false;
  }
  obstacle_sub_.reset();
  global_plan_ = nav_msgs::msg::Path();
}

void MpcController::activate()
{
  RCLCPP_INFO(logger_, "[%s] activate", plugin_name_.c_str());
  std::lock_guard<std::mutex> lock(obstacle_mutex_);
  has_valid_obstacle_msg_ = false;
}

void MpcController::deactivate()
{
  RCLCPP_INFO(logger_, "[%s] deactivate", plugin_name_.c_str());
  std::lock_guard<std::mutex> lock(obstacle_mutex_);
  has_valid_obstacle_msg_ = false;
}

void MpcController::setPlan(const nav_msgs::msg::Path & path)
{
  // Keep the raw global path and its frame. Transform per control cycle:
  // map->odom may change when localization corrects the pose.
  global_plan_ = path;
  RCLCPP_DEBUG(
    logger_, "[%s] setPlan: %zu poses in frame '%s'", plugin_name_.c_str(),
    global_plan_.poses.size(), global_plan_.header.frame_id.c_str());
}

geometry_msgs::msg::TwistStamped MpcController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & velocity,
  nav2_core::GoalChecker * goal_checker)
{
  (void)velocity;
  (void)goal_checker;

  geometry_msgs::msg::TwistStamped cmd_vel;
  if (costmap_ros_) {
    cmd_vel.header.frame_id = costmap_ros_->getBaseFrameID();
  }

  auto node = node_.lock();
  if (!node) {
    return cmd_vel;
  }
  const rclcpp::Time now = node->now();
  cmd_vel.header.stamp = now;

  if (!obstacleHeartbeatValid(now)) {
    RCLCPP_WARN_THROTTLE(
      logger_, *node->get_clock(), 1000,
      "[%s] dynamic obstacle heartbeat invalid (no valid message within %.2f s); "
      "zero velocity", plugin_name_.c_str(), obstacle_timeout_);
    return cmd_vel;
  }

  if (!costmap_ros_) {
    RCLCPP_WARN_THROTTLE(
      logger_, *node->get_clock(), 2000, "[%s] costmap is not available; zero velocity",
      plugin_name_.c_str());
    return cmd_vel;
  }
  const std::string costmap_frame = costmap_ros_->getGlobalFrameID();
  if (!pose.header.frame_id.empty() && pose.header.frame_id != costmap_frame) {
    RCLCPP_WARN_THROTTLE(
      logger_, *node->get_clock(), 1000,
      "[%s] robot pose frame '%s' != costmap frame '%s'; zero velocity",
      plugin_name_.c_str(), pose.header.frame_id.c_str(), costmap_frame.c_str());
    return cmd_vel;
  }

  if (global_plan_.poses.empty()) {
    RCLCPP_WARN_THROTTLE(
      logger_, *node->get_clock(), 2000, "[%s] empty global plan; zero velocity",
      plugin_name_.c_str());
    return cmd_vel;
  }

  const rclcpp::Time plan_stamp =
    (pose.header.stamp.sec == 0 && pose.header.stamp.nanosec == 0) ? now :
    rclcpp::Time(pose.header.stamp, now.get_clock_type());
  nav_msgs::msg::Path plan_in_costmap;
  if (!transformPlanToCostmap(plan_stamp, plan_in_costmap)) {
    RCLCPP_WARN_THROTTLE(
      logger_, *node->get_clock(), 1000,
      "[%s] cannot transform plan '%s' -> '%s'; zero velocity",
      plugin_name_.c_str(), global_plan_.header.frame_id.c_str(), costmap_frame.c_str());
    return cmd_vel;
  }

  double look_x = 0.0;
  double look_y = 0.0;
  if (!findLookaheadPoint(plan_in_costmap, pose.pose, look_x, look_y)) {
    return cmd_vel;
  }

  const double yaw = tf2::getYaw(pose.pose.orientation);
  const double dx = look_x - pose.pose.position.x;
  const double dy = look_y - pose.pose.position.y;
  const double cos_yaw = std::cos(yaw);
  const double sin_yaw = std::sin(yaw);
  const double x_r = cos_yaw * dx + sin_yaw * dy;
  const double y_r = -sin_yaw * dx + cos_yaw * dy;
  const double heading_error = std::atan2(y_r, x_r);

  const double speed = std::min(desired_linear_vel_, effectiveMaxLinearSpeed());
  const double vx = speed * std::cos(heading_error);
  cmd_vel.twist.linear.x = std::clamp(vx, v_min_, speed);
  if (chassis_type_ == "diff") {
    cmd_vel.twist.linear.y = 0.0;
  } else {
    cmd_vel.twist.linear.y = std::clamp(
      speed * std::sin(heading_error), -vy_max_, vy_max_);
  }
  cmd_vel.twist.angular.z = std::clamp(
    rotation_gain_ * heading_error, -omega_max_, omega_max_);
  return cmd_vel;
}

void MpcController::setSpeedLimit(const double & speed_limit, const bool & percentage)
{
  speed_limit_ = speed_limit;
  speed_limit_is_percentage_ = percentage;
  if (speed_limit_ == nav2_costmap_2d::NO_SPEED_LIMIT) {
    RCLCPP_INFO(logger_, "[%s] setSpeedLimit: no limit", plugin_name_.c_str());
    return;
  }
  RCLCPP_INFO(
    logger_, "[%s] setSpeedLimit: %.3f (%s)", plugin_name_.c_str(), speed_limit_,
    speed_limit_is_percentage_ ? "percentage" : "absolute");
}

void MpcController::obstacleCallback(
  const mpc_controller::msg::ObstacleStateArray::SharedPtr msg)
{
  auto node = node_.lock();
  if (!node) {
    return;
  }
  if (msg->header.frame_id != "odom") {
    RCLCPP_WARN_THROTTLE(
      logger_, *node->get_clock(), 2000,
      "[%s] obstacle message frame_id='%s' (expected 'odom'); ignored",
      plugin_name_.c_str(), msg->header.frame_id.c_str());
    return;
  }

  const rclcpp::Time now = node->now();
  const rclcpp::Time stamp(msg->header.stamp, now.get_clock_type());
  if (stamp.nanoseconds() == 0 ||
    (now - stamp) > rclcpp::Duration::from_seconds(obstacle_timeout_))
  {
    RCLCPP_WARN_THROTTLE(
      logger_, *node->get_clock(), 2000,
      "[%s] stale obstacle message (age > %.2f s); ignored",
      plugin_name_.c_str(), obstacle_timeout_);
    return;
  }

  std::lock_guard<std::mutex> lock(obstacle_mutex_);
  has_valid_obstacle_msg_ = true;
  last_obstacle_received_ = now;
}

bool MpcController::obstacleHeartbeatValid(const rclcpp::Time & now) const
{
  std::lock_guard<std::mutex> lock(obstacle_mutex_);
  if (!has_valid_obstacle_msg_) {
    return false;
  }
  return (now - last_obstacle_received_) <= rclcpp::Duration::from_seconds(obstacle_timeout_);
}

bool MpcController::transformPlanToCostmap(
  const rclcpp::Time & stamp,
  nav_msgs::msg::Path & transformed_plan) const
{
  if (!costmap_ros_ || !tf_) {
    return false;
  }
  const std::string target_frame = costmap_ros_->getGlobalFrameID();
  const std::string source_frame = global_plan_.header.frame_id;
  if (source_frame.empty()) {
    return false;
  }

  transformed_plan = global_plan_;
  transformed_plan.header.frame_id = target_frame;
  transformed_plan.header.stamp = stamp;
  if (source_frame == target_frame) {
    return true;
  }

  geometry_msgs::msg::TransformStamped transform;
  try {
    transform = tf_->lookupTransform(
      target_frame, source_frame, stamp,
      rclcpp::Duration::from_seconds(transform_tolerance_));
  } catch (const tf2::TransformException & ex) {
    auto node = node_.lock();
    if (node) {
      RCLCPP_WARN_THROTTLE(
        logger_, *node->get_clock(), 1000,
        "[%s] plan transform %s -> %s failed: %s; zero velocity",
        plugin_name_.c_str(), source_frame.c_str(), target_frame.c_str(), ex.what());
    }
    return false;
  }

  for (auto & pose : transformed_plan.poses) {
    geometry_msgs::msg::PoseStamped transformed;
    tf2::doTransform(pose, transformed, transform);
    transformed.header.frame_id = target_frame;
    transformed.header.stamp = stamp;
    pose = transformed;
  }
  return true;
}

bool MpcController::findLookaheadPoint(
  const nav_msgs::msg::Path & plan,
  const geometry_msgs::msg::Pose & robot_pose,
  double & look_x, double & look_y) const
{
  const auto & poses = plan.poses;
  if (poses.empty()) {
    return false;
  }

  std::size_t nearest = 0;
  double nearest_distance_sq = std::numeric_limits<double>::max();
  for (std::size_t i = 0; i < poses.size(); ++i) {
    const double dx = poses[i].pose.position.x - robot_pose.position.x;
    const double dy = poses[i].pose.position.y - robot_pose.position.y;
    const double distance_sq = dx * dx + dy * dy;
    if (distance_sq < nearest_distance_sq) {
      nearest_distance_sq = distance_sq;
      nearest = i;
    }
  }

  if (nearest + 1 >= poses.size()) {
    look_x = poses.back().pose.position.x;
    look_y = poses.back().pose.position.y;
    return true;
  }

  double remaining = lookahead_dist_;
  for (std::size_t i = nearest; i + 1 < poses.size(); ++i) {
    const double x0 = poses[i].pose.position.x;
    const double y0 = poses[i].pose.position.y;
    const double x1 = poses[i + 1].pose.position.x;
    const double y1 = poses[i + 1].pose.position.y;
    const double segment = std::hypot(x1 - x0, y1 - y0);
    if (segment < 1e-9) {
      continue;
    }
    if (segment >= remaining) {
      const double ratio = remaining / segment;
      look_x = x0 + ratio * (x1 - x0);
      look_y = y0 + ratio * (y1 - y0);
      return true;
    }
    remaining -= segment;
  }

  look_x = poses.back().pose.position.x;
  look_y = poses.back().pose.position.y;
  return true;
}

double MpcController::effectiveMaxLinearSpeed() const
{
  if (speed_limit_ == nav2_costmap_2d::NO_SPEED_LIMIT) {
    return v_max_;
  }
  const double limited =
    speed_limit_is_percentage_ ? v_max_ * speed_limit_ / 100.0 : speed_limit_;
  return std::clamp(limited, 0.0, v_max_);
}

}  // namespace mpc_controller

PLUGINLIB_EXPORT_CLASS(mpc_controller::MpcController, nav2_core::Controller)
