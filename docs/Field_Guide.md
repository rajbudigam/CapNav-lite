# CapNav-Lite Field Guide (v0.1 draft)

This guide is written for rehabilitation engineers and robotics labs who already run ROS 2 Navigation (Nav2) and want user-specific control behavior without heavy compute.

## What CapNav-Lite does
CapNav-Lite personalizes navigation behavior using three interpretable parameters:
- turning_cost: how expensive turns are for the user
- backtracking_cost: how expensive reversing is for the user
- slip: how noisy actions are for the user (0 to 1)

These parameters are stored in a small JSON profile that can be created in minutes.

## Safety first
Before any live run:
- Use simulation first.
- Use conservative limits in the profile.
- Use an E-stop that cuts motion power.
- Keep a clinician or trained operator present.

See `docs/Micro_Pilot_Protocol.md` for the default pilot rules.

## System overview
CapNav-Lite has three parts:
1) Nav2 controller plugin that reads the profile and shapes action scoring.
2) Calibration tool that writes the profile.
3) Safety shell that enforces limits and can trigger fallback.

## Install and build
See root `README.md`.

## Create a user profile
### Option A: Quick simulation calibration (today)
```bash
ros2 run capnav_lite_calibration calibrate_profile --ros-args \
  -p duration_s:=180.0 \
  -p output_path:=/tmp/profile.json \
  -p user_id:=demo_user
```

### Option B: Wizard calibration (v0.1 target)
The target wizard is a 3 minute routine with three tasks:
1) Turn task: two 90 degree turns and one U-turn at low speed.
2) Back-up task: reverse 1.5 m into a marked box.
3) Straight task: drive 3 m and stop on a line.

The wizard records joystick intent and odometry response. It then estimates turning_cost, backtracking_cost, and slip with confidence bounds.

## Use the profile in Nav2
Set the controller plugin in your Nav2 params:
- plugin name: `capnav_lite/CapNavLiteController`
- parameter: `<controller_name>.profile_path: /path/to/profile.json`

Example:
```yaml
controller_server:
  ros__parameters:
    use_sim_time: true
    controller_plugins: ["FollowPath"]
    FollowPath:
      plugin: "capnav_lite/CapNavLiteController"
      profile_path: "/tmp/profile.json"
```

## Run the safety shell
Route Nav2 output through the safety shell:
- Nav2 publishes to `/cmd_vel`
- Safety shell outputs `/cmd_vel_safe`
- The base listens to `/cmd_vel_safe`

Example:
```bash
ros2 run capnav_lite_safety_shell safety_shell_node --ros-args \
  -p profile_path:=/tmp/profile.json \
  -p cmd_in:=/cmd_vel \
  -p cmd_out:=/cmd_vel_safe
```

## Suggested conservative defaults
For new users and first sessions:
- max_v_m_s: 0.25 to 0.35
- max_omega_rad_s: 0.4 to 0.6
- min_clearance_m: 0.25 or higher
- max_turns_per_meter: 0.8 to 1.0

## Troubleshooting
- No motion: check E-stop circuit, mux, and whether safety shell is publishing stops.
- Oscillation: reduce max_omega, increase turning_cost in small steps, verify costmap inflation.
- Getting stuck: reduce backtracking_cost slightly, confirm recovery behaviors enabled.
- Poor tracking in clutter: verify costmap resolution, voxel layer, and controller frequency.

## Data handling
For pilots, store:
- profile JSON
- Nav2 logs
- rosbag (optional, recommended)
Keep identifiers opaque. Do not store names in the profile.

## Next steps for v0.1 completion
- Replace proxy calibration with final estimator and bootstrap confidence bounds.
- Replace controller heuristic with capability-aware candidate scoring and tabular Q prior.
- Integrate a cmd mux for shared-control fallback.
- Add a minimal GUI wizard.
