#include "mpc_controller/mpc_controller.hpp"

#include <algorithm>
#include <chrono>
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
    node, prefix + "limits.v_max", rclcpp::ParameterValue(0.6));
  nav2_util::declare_parameter_if_not_declared(
    node, prefix + "limits.v_min", rclcpp::ParameterValue(-0.6));
  nav2_util::declare_parameter_if_not_declared(
    node, prefix + "limits.vy_max", rclcpp::ParameterValue(0.6));
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

  // W6 MPC parameters (defaults mirror prototype/mpc_numpy/mpc_proto/mpc.py).
  auto declare = [&node, &prefix](
    const std::string & suffix, const rclcpp::ParameterValue & value) {
      nav2_util::declare_parameter_if_not_declared(node, prefix + suffix, value);
    };
  declare("mpc.N", rclcpp::ParameterValue(40));
  declare("mpc.dt", rclcpp::ParameterValue(0.05));
  declare("mpc.v_ref", rclcpp::ParameterValue(0.6));
  declare("mpc.q_lat", rclcpp::ParameterValue(4.0));
  declare("mpc.q_long", rclcpp::ParameterValue(0.4));
  declare("mpc.q_theta", rclcpp::ParameterValue(1.5));
  declare("mpc.r_v", rclcpp::ParameterValue(0.05));
  declare("mpc.r_omega", rclcpp::ParameterValue(0.02));
  declare("mpc.r_dv", rclcpp::ParameterValue(0.20));
  declare("mpc.r_domega", rclcpp::ParameterValue(0.10));
  declare("mpc.terminal_factor", rclcpp::ParameterValue(2.0));
  declare("mpc.rho_slack", rclcpp::ParameterValue(80.0));
  declare("mpc.d_safe", rclcpp::ParameterValue(0.30));
  declare("mpc.n_sqp", rclcpp::ParameterValue(2));
  declare("mpc.eps_abs", rclcpp::ParameterValue(1e-3));
  declare("mpc.eps_rel", rclcpp::ParameterValue(1e-3));
  declare("mpc.max_iter", rclcpp::ParameterValue(10000));
  declare("mpc.slack", rclcpp::ParameterValue(true));
  declare("mpc.terminal", rclcpp::ParameterValue(true));
  declare("use_static_distance_field", rclcpp::ParameterValue(true));
  declare("static_map_topic", rclcpp::ParameterValue("/map"));
  declare("static_field_source", rclcpp::ParameterValue("map"));
  declare("static_field_topic", rclcpp::ParameterValue(""));
  declare("static_field_max_distance", rclcpp::ParameterValue(5.0));
  declare("diagnostics_topic", rclcpp::ParameterValue("/mpc/diagnostics"));

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

  mpc_config_.N = static_cast<int>(node->get_parameter(prefix + "mpc.N").as_int());
  mpc_config_.dt = node->get_parameter(prefix + "mpc.dt").as_double();
  v_ref_ = node->get_parameter(prefix + "mpc.v_ref").as_double();
  mpc_config_.q_lat = node->get_parameter(prefix + "mpc.q_lat").as_double();
  mpc_config_.q_long = node->get_parameter(prefix + "mpc.q_long").as_double();
  mpc_config_.q_theta = node->get_parameter(prefix + "mpc.q_theta").as_double();
  mpc_config_.r_v = node->get_parameter(prefix + "mpc.r_v").as_double();
  mpc_config_.r_omega = node->get_parameter(prefix + "mpc.r_omega").as_double();
  mpc_config_.r_dv = node->get_parameter(prefix + "mpc.r_dv").as_double();
  mpc_config_.r_domega = node->get_parameter(prefix + "mpc.r_domega").as_double();
  mpc_config_.terminal_factor =
    node->get_parameter(prefix + "mpc.terminal_factor").as_double();
  mpc_config_.rho_slack = node->get_parameter(prefix + "mpc.rho_slack").as_double();
  mpc_config_.d_safe = node->get_parameter(prefix + "mpc.d_safe").as_double();
  mpc_config_.n_sqp = static_cast<int>(node->get_parameter(prefix + "mpc.n_sqp").as_int());
  mpc_config_.eps_abs = node->get_parameter(prefix + "mpc.eps_abs").as_double();
  mpc_config_.eps_rel = node->get_parameter(prefix + "mpc.eps_rel").as_double();
  mpc_config_.max_iter =
    static_cast<int>(node->get_parameter(prefix + "mpc.max_iter").as_int());
  mpc_config_.slack = node->get_parameter(prefix + "mpc.slack").as_bool();
  mpc_config_.terminal = node->get_parameter(prefix + "mpc.terminal").as_bool();
  use_static_distance_field_ =
    node->get_parameter(prefix + "use_static_distance_field").as_bool();
  static_map_topic_ = node->get_parameter(prefix + "static_map_topic").as_string();
  static_field_source_ = node->get_parameter(prefix + "static_field_source").as_string();
  static_field_topic_ = node->get_parameter(prefix + "static_field_topic").as_string();
  if (static_field_source_ != "map" && static_field_source_ != "local_costmap") {
    throw std::runtime_error(
      "MpcController: static_field_source must be 'map' or 'local_costmap', got '" +
      static_field_source_ + "'");
  }
  if (static_field_topic_.empty()) {
    static_field_topic_ = static_field_source_ == "local_costmap" ?
      "/debug/static_local_grid" : static_map_topic_;
  }
  static_field_max_distance_ =
    node->get_parameter(prefix + "static_field_max_distance").as_double();
  diagnostics_topic_ = node->get_parameter(prefix + "diagnostics_topic").as_string();

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
  if (mpc_config_.N <= 0 || mpc_config_.dt <= 0.0 || v_ref_ <= 0.0 ||
    mpc_config_.q_lat <= 0.0 || mpc_config_.d_safe < 0.0 || mpc_config_.n_sqp <= 0 ||
    mpc_config_.max_iter <= 0 || mpc_config_.eps_abs <= 0.0 || mpc_config_.eps_rel <= 0.0 ||
    static_field_max_distance_ <= 0.0)
  {
    throw std::runtime_error("MpcController: invalid MPC parameter");
  }

  chassis_params_.radius = robot_radius_;
  chassis_params_.v_min = v_min_;
  chassis_params_.v_max = v_max_;
  chassis_params_.vy_max = vy_max_;
  chassis_params_.omega_max = omega_max_;
  chassis_params_.acc_max = acc_max_;
  chassis_params_.alpha_max = alpha_max_;
  mpc_ = std::make_unique<MpcCore>(mpc_config_, chassis_params_);

  obstacle_sub_ = node->create_subscription<mpc_controller::msg::ObstacleStateArray>(
    "/mpc/obstacles", rclcpp::QoS(10),
    std::bind(&MpcController::obstacleCallback, this, std::placeholders::_1));
  has_valid_obstacle_msg_ = false;
  last_obstacle_received_ = node->now();

  diagnostics_pub_ =
    node->create_publisher<mpc_controller::msg::MpcDiagnostics>(
    diagnostics_topic_, rclcpp::QoS(10));
  if (use_static_distance_field_) {
    // `/map` is latched (transient_local); the rolling local window is 10 Hz volatile.
    // A transient_local subscription cannot receive from a volatile publisher.
    const auto static_qos = static_field_source_ == "local_costmap" ?
      rclcpp::QoS(1).reliable() :
      rclcpp::QoS(1).transient_local().reliable();
    map_sub_ = node->create_subscription<nav_msgs::msg::OccupancyGrid>(
      static_field_topic_, static_qos,
      std::bind(&MpcController::mapCallback, this, std::placeholders::_1));
  }

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
  RCLCPP_INFO(
    logger_,
    "[%s] MPC: N=%d dt=%.3f v_ref=%.2f d_safe=%.2f n_sqp=%d slack=%d terminal=%d "
    "eps=%.1e/%.1e max_iter=%d",
    plugin_name_.c_str(), mpc_config_.N, mpc_config_.dt, v_ref_, mpc_config_.d_safe,
    mpc_config_.n_sqp, static_cast<int>(mpc_config_.slack),
    static_cast<int>(mpc_config_.terminal), mpc_config_.eps_abs, mpc_config_.eps_rel,
    mpc_config_.max_iter);
  RCLCPP_INFO(
    logger_,
    "[%s] static distance field=%d source='%s' topic='%s' max=%.2f; "
    "diagnostics topic='%s'",
    plugin_name_.c_str(), static_cast<int>(use_static_distance_field_),
    static_field_source_.c_str(), static_field_topic_.c_str(),
    static_field_max_distance_, diagnostics_topic_.c_str());
}

void MpcController::cleanup()
{
  RCLCPP_INFO(logger_, "[%s] cleanup", plugin_name_.c_str());
  {
    std::lock_guard<std::mutex> lock(obstacle_mutex_);
    has_valid_obstacle_msg_ = false;
    obstacles_.clear();
  }
  obstacle_sub_.reset();
  map_sub_.reset();
  diagnostics_pub_.reset();
  {
    std::lock_guard<std::mutex> lock(field_mutex_);
    static_field_.reset();
    static_field_frame_.clear();
  }
  reference_.reset();
  mpc_.reset();
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
  auto node = node_.lock();
  if (node) {
    plan_start_ = node->now();
    has_plan_start_ = true;
  }
  reference_.reset();
  if (mpc_) {
    // New plan: reset warm start and fallback streak.
    mpc_ = std::make_unique<MpcCore>(mpc_config_, chassis_params_);
  }
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

  if (!buildReference(plan_in_costmap)) {
    RCLCPP_WARN_THROTTLE(
      logger_, *node->get_clock(), 2000,
      "[%s] cannot build reference from the transformed plan; zero velocity",
      plugin_name_.c_str());
    return cmd_vel;
  }

  // Reference time law is anchored at setPlan(): s(t) = min(v_ref * t, L).
  const double t_elapsed = has_plan_start_ ?
    std::max(0.0, (now - plan_start_).seconds()) : 0.0;
  Eigen::MatrixXd z_ref;
  Eigen::MatrixXd u_ref;
  reference_->sequence(t_elapsed, z_ref, u_ref);

  std::vector<Obstacle> obstacles;
  {
    std::lock_guard<std::mutex> lock(obstacle_mutex_);
    obstacles = obstacles_;
  }

  std::shared_ptr<GridDistanceField> static_field;
  Eigen::Matrix2d map_to_odom_rotation = Eigen::Matrix2d::Identity();
  Eigen::Vector2d map_to_odom_translation = Eigen::Vector2d::Zero();
  if (use_static_distance_field_) {
    std::string field_frame;
    {
      std::lock_guard<std::mutex> lock(field_mutex_);
      static_field = static_field_;
      field_frame = static_field_frame_;
    }
    if (static_field && !static_field->empty() && !field_frame.empty() &&
      field_frame != costmap_frame)
    {
      try {
        const auto transform = tf_->lookupTransform(
          costmap_frame, field_frame, now,
          rclcpp::Duration::from_seconds(transform_tolerance_));
        const double yaw = tf2::getYaw(transform.transform.rotation);
        map_to_odom_rotation = Eigen::Rotation2Dd(yaw).toRotationMatrix();
        map_to_odom_translation = Eigen::Vector2d(
          transform.transform.translation.x, transform.transform.translation.y);
      } catch (const tf2::TransformException & ex) {
        RCLCPP_WARN_THROTTLE(
          logger_, *node->get_clock(), 2000,
          "[%s] static map transform '%s' -> '%s' failed: %s; field disabled this cycle",
          plugin_name_.c_str(), field_frame.c_str(), costmap_frame.c_str(), ex.what());
        static_field.reset();
      }
    }
  }

  const Eigen::Vector3d z0(
    pose.pose.position.x, pose.pose.position.y, tf2::getYaw(pose.pose.orientation));
  MpcResult result;
  double solve_ms = 0.0;
  try {
    const auto solve_start = std::chrono::steady_clock::now();
    result = mpc_->solve(
      z0, z_ref, u_ref, obstacles, static_field.get(),
      map_to_odom_rotation, map_to_odom_translation);
    solve_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - solve_start).count();
  } catch (const std::exception & ex) {
    result.status = std::string("exception:") + ex.what();
  }
  RCLCPP_INFO_THROTTLE(
    logger_, *node->get_clock(), 1000,
    "[%s] MPC solve %.2f ms (status=%s qp_iters=%d obstacles=%zu field=%d)",
    plugin_name_.c_str(), solve_ms, result.status.c_str(), result.qp_iterations,
    obstacles.size(), static_cast<int>(static_field != nullptr));

  mpc_controller::msg::MpcDiagnostics diagnostics;
  diagnostics.header.stamp = now;
  diagnostics.header.frame_id = costmap_frame;
  diagnostics.success = result.success;
  diagnostics.fallback = result.fallback;
  diagnostics.alarm = result.alarm;
  diagnostics.solve_ms = solve_ms;
  diagnostics.qp_iterations =
    static_cast<uint32_t>(std::max(0, result.qp_iterations));
  diagnostics.fallback_streak =
    static_cast<uint32_t>(std::max(0, result.fallback_streak));
  diagnostics.max_slack = result.max_slack;
  diagnostics.obstacle_count = static_cast<uint32_t>(obstacles.size());
  diagnostics.static_field_used = static_field != nullptr;
  diagnostics.status = result.status;
  diagnostics_pub_->publish(diagnostics);

  if (!result.success) {
    RCLCPP_WARN_THROTTLE(
      logger_, *node->get_clock(), 1000,
      "[%s] MPC solve failed (status=%s fallback_streak=%d alarm=%d); zero velocity",
      plugin_name_.c_str(), result.status.c_str(), result.fallback_streak,
      static_cast<int>(result.alarm));
    return cmd_vel;
  }

  Eigen::Vector3d u = result.u;
  if (chassis_type_ == "diff") {
    u.y() = 0.0;
  }
  u.x() = std::clamp(u.x(), v_min_, v_max_);
  u.y() = std::clamp(u.y(), -vy_max_, vy_max_);
  u.z() = std::clamp(u.z(), -omega_max_, omega_max_);

  // The shared execution layer owns the task-level planar speed limit;
  // only an explicit Nav2 speed limit is applied inside the controller.
  if (speed_limit_ != nav2_costmap_2d::NO_SPEED_LIMIT) {
    const double max_speed = effectiveMaxLinearSpeed();
    const double planar = std::hypot(u.x(), u.y());
    if (max_speed > 0.0 && planar > max_speed) {
      const double scale = max_speed / planar;
      u.x() *= scale;
      u.y() *= scale;
    }
  }

  cmd_vel.twist.linear.x = u.x();
  cmd_vel.twist.linear.y = u.y();
  cmd_vel.twist.angular.z = u.z();
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

  const rclcpp::Time header_stamp(msg->header.stamp, now.get_clock_type());
  std::vector<Obstacle> obstacles;
  obstacles.reserve(msg->obstacles.size());
  for (const auto & obstacle : msg->obstacles) {
    const rclcpp::Time stamp(obstacle.stamp, now.get_clock_type());
    const rclcpp::Time estimate_stamp =
      (stamp.nanoseconds() == 0) ? header_stamp : stamp;
    if (estimate_stamp.nanoseconds() == 0) {
      continue;
    }
    // Single-obstacle staleness: drop it, keep the heartbeat valid.
    const double age = std::max(0.0, (now - estimate_stamp).seconds());
    if (age > obstacle_timeout_ || obstacle.radius <= 0.0) {
      continue;
    }
    const Eigen::Vector2d velocity(obstacle.velocity.x, obstacle.velocity.y);
    // The message reports the estimate at `stamp`; advance it to now so that
    // MpcCore's tau=0 obstacle position is current.
    const Eigen::Vector2d position(
      obstacle.position.x + velocity.x() * age,
      obstacle.position.y + velocity.y() * age);
    obstacles.push_back(
      Obstacle::circle(
        static_cast<int>(obstacle.id), position, velocity,
        static_cast<double>(obstacle.radius)));
  }

  std::lock_guard<std::mutex> lock(obstacle_mutex_);
  has_valid_obstacle_msg_ = true;
  last_obstacle_received_ = now;
  obstacles_ = std::move(obstacles);
}

void MpcController::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  if (!msg || msg->info.width == 0 || msg->info.height == 0 ||
    msg->info.resolution <= 0.0)
  {
    return;
  }
  const std::size_t cells =
    static_cast<std::size_t>(msg->info.width) * msg->info.height;
  if (msg->data.size() != cells) {
    RCLCPP_WARN(
      logger_, "[%s] static map data size mismatch (%zu != %zu); ignored",
      plugin_name_.c_str(), msg->data.size(), cells);
    return;
  }

  std::vector<uint8_t> occupied(cells, 0);
  for (std::size_t index = 0; index < cells; ++index) {
    // Unknown (-1) is conservative-occupied; otherwise the rolling local window
    // edge would be treated as free space.
    occupied[index] = (msg->data[index] < 0 || msg->data[index] >= 50) ? 1 : 0;
  }

  auto field = std::make_shared<GridDistanceField>();
  try {
    field->build(
      static_cast<int>(msg->info.width), static_cast<int>(msg->info.height),
      msg->info.resolution, msg->info.origin.position.x, msg->info.origin.position.y,
      occupied, static_field_max_distance_);
  } catch (const std::exception & ex) {
    RCLCPP_WARN(
      logger_, "[%s] static distance field build failed: %s",
      plugin_name_.c_str(), ex.what());
    return;
  }

  {
    std::lock_guard<std::mutex> lock(field_mutex_);
    static_field_ = std::move(field);
    static_field_frame_ = msg->header.frame_id;
  }
  // 10 Hz local grids would flood INFO; keep this on the debug channel.
  RCLCPP_DEBUG(
    logger_, "[%s] static distance field built: %dx%d res=%.3f frame='%s'",
    plugin_name_.c_str(), msg->info.width, msg->info.height, msg->info.resolution,
    msg->header.frame_id.c_str());
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

bool MpcController::buildReference(const nav_msgs::msg::Path & plan) const
{
  if (plan.poses.size() < 2) {
    return false;
  }

  std::vector<Eigen::Vector2d> waypoints;
  waypoints.reserve(plan.poses.size());
  for (const auto & pose_stamped : plan.poses) {
    waypoints.emplace_back(
      pose_stamped.pose.position.x, pose_stamped.pose.position.y);
  }

  try {
    const double goal_yaw = tf2::getYaw(plan.poses.back().pose.orientation);
    reference_ = std::make_unique<ReferencePath>(
      waypoints, goal_yaw, v_ref_, mpc_config_.N, mpc_config_.dt);
  } catch (const std::exception & ex) {
    auto node = node_.lock();
    if (node) {
      RCLCPP_WARN_THROTTLE(
        logger_, *node->get_clock(), 2000,
        "[%s] reference construction failed: %s", plugin_name_.c_str(), ex.what());
    }
    return false;
  }
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
