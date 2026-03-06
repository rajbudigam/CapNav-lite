#pragma once

#include <memory>
#include <string>
#include <vector>

#include "nav2_core/controller.hpp"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"

namespace capnav_lite
{

struct CapabilityProfile
{
  double turning_cost = 0.0;
  double backtracking_cost = 0.0;
  double slip = 0.0;

  // Safety limits that the controller should respect
  double max_v_m_s = 0.35;
  double max_omega_rad_s = 0.6;
};

class CapNavLiteController : public nav2_core::Controller
{
public:
  CapNavLiteController() = default;
  ~CapNavLiteController() override = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    const std::shared_ptr<tf2_ros::Buffer> tf,
    const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

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
  CapabilityProfile loadProfileOrDefault(const std::string & path);

  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  std::string name_;
  nav_msgs::msg::Path global_path_;

  CapabilityProfile profile_;
  std::string profile_path_;

  // Controller parameters
  double base_max_v_m_s_ = 0.35;
  double base_max_omega_rad_s_ = 0.6;

  // Placeholder: hook for capability-aware action scoring
  geometry_msgs::msg::TwistStamped computeHeuristicCommand(
    const geometry_msgs::msg::PoseStamped & pose) const;
};

}  // namespace capnav_lite
