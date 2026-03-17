#pragma once

#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav2_core/controller.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "tf2_ros/buffer.h"

#include "capnav_lite/profile.hpp"

namespace capnav_lite {

struct CandidateAction
{
  std::string label;
  double linear_mps;
  double angular_rad_s;
  double score;
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
  CandidateAction evaluate_action(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    const std::string & label,
    double linear,
    double angular) const;
  geometry_msgs::msg::Point pick_lookahead_point(const geometry_msgs::msg::PoseStamped & pose) const;
  double clearance_ahead(const geometry_msgs::msg::PoseStamped & pose, double distance_m) const;
  geometry_msgs::msg::PoseStamped simulate_step(
    const geometry_msgs::msg::PoseStamped & pose,
    double linear,
    double angular,
    double dt) const;
  double progress_score(
    const geometry_msgs::msg::PoseStamped & before,
    const geometry_msgs::msg::PoseStamped & after,
    const geometry_msgs::msg::Point & target) const;

  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  nav_msgs::msg::Path global_path_;
  std::string name_;
  std::string profile_path_;
  CapabilityProfile profile_;
  double base_max_linear_mps_{0.45};
  double base_max_angular_rad_s_{0.75};
  double simulate_dt_s_{0.5};
};

}  // namespace capnav_lite
