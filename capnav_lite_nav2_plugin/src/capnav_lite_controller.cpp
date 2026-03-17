#include "capnav_lite/capnav_lite_controller.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace capnav_lite {

void CapNavLiteController::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name,
  const std::shared_ptr<tf2_ros::Buffer> /*tf*/,
  const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent.lock();
  costmap_ros_ = costmap_ros;
  name_ = std::move(name);

  nav2_util::declare_parameter_if_not_declared(node_, name_ + ".profile_path", rclcpp::ParameterValue(""));
  nav2_util::declare_parameter_if_not_declared(node_, name_ + ".base_max_linear_mps", rclcpp::ParameterValue(0.45));
  nav2_util::declare_parameter_if_not_declared(node_, name_ + ".base_max_angular_rad_s", rclcpp::ParameterValue(0.75));
  nav2_util::declare_parameter_if_not_declared(node_, name_ + ".simulate_dt_s", rclcpp::ParameterValue(0.5));

  node_->get_parameter(name_ + ".profile_path", profile_path_);
  node_->get_parameter(name_ + ".base_max_linear_mps", base_max_linear_mps_);
  node_->get_parameter(name_ + ".base_max_angular_rad_s", base_max_angular_rad_s_);
  node_->get_parameter(name_ + ".simulate_dt_s", simulate_dt_s_);

  try {
    profile_ = load_profile_from_json(profile_path_);
  } catch (const std::exception & e) {
    RCLCPP_WARN(node_->get_logger(), "Falling back to defaults because profile load failed: %s", e.what());
    profile_ = CapabilityProfile();
  }

  profile_.max_linear_speed_mps = clamp(profile_.max_linear_speed_mps, 0.0, base_max_linear_mps_);
  profile_.max_angular_speed_rad_s = clamp(profile_.max_angular_speed_rad_s, 0.0, base_max_angular_rad_s_);

  RCLCPP_INFO(
    node_->get_logger(),
    "CapNav-Lite configured for user=%s turning_cost=%.2f backtracking_cost=%.2f slip=%.2f",
    profile_.user_id.c_str(), profile_.turning_cost, profile_.backtracking_cost, profile_.slip_probability);
}

void CapNavLiteController::cleanup()
{
  global_path_.poses.clear();
  costmap_ros_.reset();
}

void CapNavLiteController::activate() {}
void CapNavLiteController::deactivate() {}

void CapNavLiteController::setPlan(const nav_msgs::msg::Path & path)
{
  global_path_ = path;
}

geometry_msgs::msg::Point CapNavLiteController::pick_lookahead_point(const geometry_msgs::msg::PoseStamped & pose) const
{
  geometry_msgs::msg::Point fallback = pose.pose.position;
  if (global_path_.poses.empty()) {
    return fallback;
  }
  std::size_t nearest = 0;
  double best_dist = std::numeric_limits<double>::max();
  for (std::size_t i = 0; i < global_path_.poses.size(); ++i) {
    const auto & p = global_path_.poses[i].pose.position;
    const double d = std::hypot(p.x - pose.pose.position.x, p.y - pose.pose.position.y);
    if (d < best_dist) {
      best_dist = d;
      nearest = i;
    }
  }
  return global_path_.poses[std::min(nearest + std::size_t(6), global_path_.poses.size() - std::size_t(1))].pose.position;
}

geometry_msgs::msg::PoseStamped CapNavLiteController::simulate_step(
  const geometry_msgs::msg::PoseStamped & pose,
  double linear,
  double angular,
  double dt) const
{
  geometry_msgs::msg::PoseStamped out = pose;
  const double yaw = tf2::getYaw(pose.pose.orientation) + angular * dt;
  out.pose.position.x += linear * std::cos(yaw) * dt;
  out.pose.position.y += linear * std::sin(yaw) * dt;
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, yaw);
  out.pose.orientation = tf2::toMsg(q);
  return out;
}

double CapNavLiteController::clearance_ahead(const geometry_msgs::msg::PoseStamped & pose, double distance_m) const
{
  auto * costmap = costmap_ros_->getCostmap();
  const double yaw = tf2::getYaw(pose.pose.orientation);
  const double step = 0.05;
  for (double d = 0.0; d <= distance_m; d += step) {
    const double wx = pose.pose.position.x + std::cos(yaw) * d;
    const double wy = pose.pose.position.y + std::sin(yaw) * d;
    unsigned int mx = 0;
    unsigned int my = 0;
    if (!costmap->worldToMap(wx, wy, mx, my)) {
      return std::max(0.0, d - step);
    }
    const auto cost = costmap->getCost(mx, my);
    if (cost >= nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
      return std::max(0.0, d - step);
    }
  }
  return distance_m;
}

double CapNavLiteController::progress_score(
  const geometry_msgs::msg::PoseStamped & before,
  const geometry_msgs::msg::PoseStamped & after,
  const geometry_msgs::msg::Point & target) const
{
  const double d_before = std::hypot(before.pose.position.x - target.x, before.pose.position.y - target.y);
  const double d_after = std::hypot(after.pose.position.x - target.x, after.pose.position.y - target.y);
  return d_before - d_after;
}

CandidateAction CapNavLiteController::evaluate_action(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & /*velocity*/,
  const std::string & label,
  double linear,
  double angular) const
{
  const auto target = pick_lookahead_point(pose);
  const auto simulated = simulate_step(pose, linear, angular, simulate_dt_s_);
  const double clearance_now = clearance_ahead(pose, 1.5);
  const double clearance_future = clearance_ahead(simulated, 1.0);
  const double yaw_now = tf2::getYaw(pose.pose.orientation);
  const double heading_now = std::atan2(target.y - pose.pose.position.y, target.x - pose.pose.position.x);
  const double heading_future = std::atan2(target.y - simulated.pose.position.y, target.x - simulated.pose.position.x);
  auto wrap = [](double a) {
    while (a > M_PI) a -= 2.0 * M_PI;
    while (a < -M_PI) a += 2.0 * M_PI;
    return a;
  };
  const double heading_gain = std::abs(wrap(heading_now - yaw_now)) - std::abs(wrap(heading_future - tf2::getYaw(simulated.pose.orientation)));
  double score = 0.0;
  score += 4.0 * progress_score(pose, simulated, target);
  score += 1.25 * heading_gain;
  score += 0.8 * std::min(clearance_future, 1.2);
  if (label == "FL" || label == "FR" || label == "L" || label == "R") {
    score -= 0.9 * profile_.turning_cost * std::abs(angular) / std::max(profile_.max_angular_speed_rad_s, 1e-3);
  }
  if (label == "B") {
    score -= 1.1 * profile_.backtracking_cost;
  }
  if (clearance_now < profile_.clearance_margin_m + 0.1 && linear > 0.0) {
    score -= 6.0;
  }
  if (clearance_future <= profile_.clearance_margin_m) {
    score -= 12.0;
  }
  return CandidateAction{label, linear, angular, score};
}

geometry_msgs::msg::TwistStamped CapNavLiteController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & velocity,
  nav2_core::GoalChecker * /*goal_checker*/)
{
  std::vector<CandidateAction> candidates = {
    evaluate_action(pose, velocity, "F", profile_.comfort_linear_speed_mps, 0.0),
    evaluate_action(pose, velocity, "SLOW", 0.55 * profile_.comfort_linear_speed_mps, 0.0),
    evaluate_action(pose, velocity, "FL", 0.75 * profile_.comfort_linear_speed_mps, 0.65 * profile_.comfort_angular_speed_rad_s),
    evaluate_action(pose, velocity, "FR", 0.75 * profile_.comfort_linear_speed_mps, -0.65 * profile_.comfort_angular_speed_rad_s),
    evaluate_action(pose, velocity, "L", 0.0, profile_.comfort_angular_speed_rad_s),
    evaluate_action(pose, velocity, "R", 0.0, -profile_.comfort_angular_speed_rad_s),
    evaluate_action(pose, velocity, "B", -0.35 * profile_.comfort_linear_speed_mps, 0.0),
    evaluate_action(pose, velocity, "S", 0.0, 0.0),
  };

  const auto best = std::max_element(candidates.begin(), candidates.end(), [](const auto & a, const auto & b) {
    return a.score < b.score;
  });

  geometry_msgs::msg::TwistStamped cmd;
  cmd.header.stamp = node_->now();
  cmd.header.frame_id = costmap_ros_->getBaseFrameID();
  cmd.twist.linear.x = best->linear_mps;
  cmd.twist.angular.z = best->angular_rad_s;
  return cmd;
}

void CapNavLiteController::setSpeedLimit(const double & speed_limit, const bool & percentage)
{
  if (percentage) {
    profile_.max_linear_speed_mps = clamp(base_max_linear_mps_ * speed_limit / 100.0, 0.0, base_max_linear_mps_);
  } else {
    profile_.max_linear_speed_mps = clamp(speed_limit, 0.0, base_max_linear_mps_);
  }
}

}  // namespace capnav_lite

PLUGINLIB_EXPORT_CLASS(capnav_lite::CapNavLiteController, nav2_core::Controller)
