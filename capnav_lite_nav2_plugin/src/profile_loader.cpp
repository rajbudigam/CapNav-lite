#include "capnav_lite/profile.hpp"

#include <fstream>
#include <stdexcept>

#include <nlohmann/json.hpp>

namespace capnav_lite {

using json = nlohmann::json;

double clamp(double x, double lo, double hi)
{
  return std::max(lo, std::min(hi, x));
}

CapabilityProfile load_profile_from_json(const std::string & path)
{
  CapabilityProfile profile;
  if (path.empty()) {
    return profile;
  }
  std::ifstream handle(path);
  if (!handle.good()) {
    throw std::runtime_error("could not open profile json: " + path);
  }
  json j;
  handle >> j;
  profile.user_id = j.value("user_id", profile.user_id);
  profile.max_linear_speed_mps = j.value("max_linear_speed_mps", profile.max_linear_speed_mps);
  profile.max_angular_speed_rad_s = j.value("max_angular_speed_rad_s", profile.max_angular_speed_rad_s);
  profile.comfort_linear_speed_mps = j.value("comfort_linear_speed_mps", profile.comfort_linear_speed_mps);
  profile.comfort_angular_speed_rad_s = j.value("comfort_angular_speed_rad_s", profile.comfort_angular_speed_rad_s);
  profile.turning_cost = j.value("turning_cost", profile.turning_cost);
  profile.backtracking_cost = j.value("backtracking_cost", profile.backtracking_cost);
  profile.slip_probability = clamp(j.value("slip_probability", profile.slip_probability), 0.0, 0.6);
  profile.reaction_time_s = j.value("reaction_time_s", profile.reaction_time_s);
  profile.clearance_margin_m = j.value("clearance_margin_m", profile.clearance_margin_m);
  return profile;
}

}  // namespace capnav_lite
