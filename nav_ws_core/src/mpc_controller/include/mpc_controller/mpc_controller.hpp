#ifndef MPC_CONTROLLER__MPC_CONTROLLER_HPP_
#define MPC_CONTROLLER__MPC_CONTROLLER_HPP_

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "mpc_controller/msg/mpc_diagnostics.hpp"
#include "mpc_controller/msg/obstacle_state_array.hpp"
#include "mpc_controller/mpc_core.hpp"
#include "mpc_controller/reference.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_core/controller.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
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
  void obstacleCallback(const mpc_controller::msg::ObstacleStateArray::SharedPtr msg);
  void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  bool obstacleHeartbeatValid(const rclcpp::Time & now) const;
  bool transformPlanToCostmap(
    const rclcpp::Time & stamp,
    nav_msgs::msg::Path & transformed_plan) const;
  bool buildReference(const nav_msgs::msg::Path & plan) const;
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

  // Legacy W5 passthrough parameters (kept declared for smoke compatibility).
  double desired_linear_vel_{0.6};
  double lookahead_dist_{0.5};
  double rotation_gain_{2.0};
  double transform_tolerance_{0.2};

  // W6 MPC core (QP builder + SQP + reference generator).
  MpcConfig mpc_config_;
  ChassisParams chassis_params_;
  double v_ref_{0.6};
  std::unique_ptr<MpcCore> mpc_;
  mutable std::unique_ptr<ReferencePath> reference_;
  rclcpp::Time plan_start_{0, 0, RCL_SYSTEM_TIME};
  bool has_plan_start_{false};

  // Dynamic obstacle heartbeat (W5 D3 semantics; real tracker arrives in W6).
  rclcpp::Subscription<mpc_controller::msg::ObstacleStateArray>::SharedPtr obstacle_sub_;
  mutable std::mutex obstacle_mutex_;
  bool has_valid_obstacle_msg_{false};
  rclcpp::Time last_obstacle_received_{0, 0, RCL_SYSTEM_TIME};
  std::vector<Obstacle> obstacles_;
  double obstacle_timeout_{0.5};

  // W6 static distance field: `/map` (transient_local) or the rolling
  // `/debug/static_local_grid` window (volatile, 10 Hz). A 口径 uses the latter.
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Publisher<mpc_controller::msg::MpcDiagnostics>::SharedPtr diagnostics_pub_;
  mutable std::mutex field_mutex_;
  std::shared_ptr<GridDistanceField> static_field_;
  std::string static_field_frame_;
  bool use_static_distance_field_{true};
  double static_field_max_distance_{5.0};
  std::string static_map_topic_{"/map"};
  std::string static_field_source_{"map"};
  std::string static_field_topic_{"/map"};
  std::string diagnostics_topic_{"/mpc/diagnostics"};

  double speed_limit_{0.0};  // nav2_costmap_2d::NO_SPEED_LIMIT
  bool speed_limit_is_percentage_{false};
  rclcpp::Logger logger_{rclcpp::get_logger("MpcController")};
};

}  // namespace mpc_controller

#endif  // MPC_CONTROLLER__MPC_CONTROLLER_HPP_
