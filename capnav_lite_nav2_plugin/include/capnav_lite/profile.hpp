#pragma once

#include <string>

namespace capnav_lite {

struct CapabilityProfile
{
  std::string user_id{"unknown"};
  double max_linear_speed_mps{0.45};
  double max_angular_speed_rad_s{0.75};
  double comfort_linear_speed_mps{0.28};
  double comfort_angular_speed_rad_s{0.45};
  double turning_cost{1.2};
  double backtracking_cost{1.0};
  double slip_probability{0.08};
  double reaction_time_s{0.35};
  double clearance_margin_m{0.30};
};

CapabilityProfile load_profile_from_json(const std::string & path);

double clamp(double x, double lo, double hi);

}  // namespace capnav_lite
