#include "capnav_lite/capnav_lite_controller.hpp"

#include <fstream>
#include <algorithm>

#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace capnav_lite
{

static double clamp(double x, double lo, double hi)
{
  return std::min(std::max(x, lo), hi);
}

void CapNavLiteController::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name,
  const std::shared_ptr<tf2_ros::Buffer> /*tf*/,
  const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> /*costmap_ros*/)
{
  node_ = parent.lock();
  name_ = name;

  nav2_util::declare_parameter_if_not_declared(node_, name_ + ".profile_path", rclcpp::ParameterValue(""));
  nav2_util::declare_parameter_if_not_declared(node_, name_ + ".base_max_v_m_s", rclcpp::ParameterValue(0.35));
  nav2_util::declare_parameter_if_not_declared(node_, name_ + ".base_max_omega_rad_s", rclcpp::ParameterValue(0.6));

  node_->get_parameter(name_ + ".profile_path", profile_path_);
  node_->get_parameter(name_ + ".base_max_v_m_s", base_max_v_m_s_);
  node_->get_parameter(name_ + ".base_max_omega_rad_s", base_max_omega_rad_s_);

  profile_ = loadProfileOrDefault(profile_path_);

  RCLCPP_INFO(node_->get_logger(),
    "CapNav-Lite configured. turning_cost=%.3f backtracking_cost=%.3f slip=%.3f",
    profile_.turning_cost, profile_.backtracking_cost, profile_.slip);
}

void CapNavLiteController::cleanup()
{
  global_path_.poses.clear();
}

void CapNavLiteController::activate()
{
}

void CapNavLiteController::deactivate()
{
}

void CapNavLiteController::setPlan(const nav_msgs::msg::Path & path)
{
  global_path_ = path;
}

geometry_msgs::msg::TwistStamped CapNavLiteController::computeHeuristicCommand(
  const geometry_msgs::msg::PoseStamped & /*pose*/) const
{
  geometry_msgs::msg::TwistStamped cmd;
  cmd.header.stamp = node_->now();
  cmd.header.frame_id = "base_link";

  // Placeholder heuristic:
  // - move forward slowly
  // - scale angular velocity down for users with high turning cost
  const double turning_scale = 1.0 / (1.0 + profile_.turning_cost);

  cmd.twist.linear.x = clamp(0.20, 0.0, profile_.max_v_m_s);
  cmd.twist.angular.z = clamp(0.20 * turning_scale, -profile_.max_omega_rad_s, profile_.max_omega_rad_s);
  return cmd;
}

geometry_msgs::msg::TwistStamped CapNavLiteController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & /*velocity*/,
  nav2_core::GoalChecker * /*goal_checker*/)
{
  // Skeleton behavior:
  // A full implementation would score candidate actions using:
  // - the capability-aware cost model (turning, backtracking)
  // - a tabular Q prior for the local state
  // - online updates nudging Q toward the calibrated profile
  return computeHeuristicCommand(pose);
}

void CapNavLiteController::setSpeedLimit(const double & speed_limit, const bool & percentage)
{
  // Respect Nav2 speed limit requests while keeping within safety bounds.
  double limit = speed_limit;
  if (percentage) {
    limit = base_max_v_m_s_ * clamp(speed_limit / 100.0, 0.0, 1.0);
  }
  profile_.max_v_m_s = clamp(limit, 0.0, base_max_v_m_s_);
}

CapabilityProfile CapNavLiteController::loadProfileOrDefault(const std::string & path)
{
  CapabilityProfile p;
  p.max_v_m_s = base_max_v_m_s_;
  p.max_omega_rad_s = base_max_omega_rad_s_;

  if (path.empty()) {
    RCLCPP_WARN(node_->get_logger(), "No profile_path set. Using conservative defaults.");
    p.turning_cost = 2.0;
    p.backtracking_cost = 1.5;
    p.slip = 0.15;
    return p;
  }

  // Minimal JSON parsing without external deps (skeleton).
  // The v0.1 implementation should use a small JSON library (nlohmann/json) for correctness.
  std::ifstream f(path);
  if (!f.good()) {
    RCLCPP_WARN(node_->get_logger(), "Profile file not found. Using defaults: %s", path.c_str());
    p.turning_cost = 2.0;
    p.backtracking_cost = 1.5;
    p.slip = 0.15;
    return p;
  }
  std::string s((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>());

  auto find_number = [&](const std::string & key, double fallback) -> double {
    const auto pos = s.find("\"" + key + "\"");
    if (pos == std::string::npos) return fallback;
    const auto colon = s.find(":", pos);
    if (colon == std::string::npos) return fallback;
    const auto end = s.find_first_of(",}", colon + 1);
    const auto token = s.substr(colon + 1, end - (colon + 1));
    try { return std::stod(token); } catch (...) { return fallback; }
  };

  p.turning_cost = clamp(find_number("turning_cost", 2.0), 0.0, 10.0);
  p.backtracking_cost = clamp(find_number("backtracking_cost", 1.5), 0.0, 10.0);
  p.slip = clamp(find_number("slip", 0.15), 0.0, 1.0);

  p.max_v_m_s = clamp(find_number("max_v_m_s", base_max_v_m_s_), 0.0, base_max_v_m_s_);
  p.max_omega_rad_s = clamp(find_number("max_omega_rad_s", base_max_omega_rad_s_), 0.0, base_max_omega_rad_s_);

  return p;
}

}  // namespace capnav_lite

PLUGINLIB_EXPORT_CLASS(capnav_lite::CapNavLiteController, nav2_core::Controller)
