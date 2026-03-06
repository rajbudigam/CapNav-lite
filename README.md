# CapNav-Lite (repo skeleton)

CapNav-Lite is a lightweight personalization layer for ROS 2 Navigation (Nav2) that adapts wheelchair-style navigation to an individual user's motor profile in minutes, using simple tabular reinforcement learning and a capability-aware prior.

This repository is a starter skeleton that matches the deliverables described in the CapNav-Lite v0.1 pack:
- Nav2 plugin (controller-first, planner optional)
- 3-minute calibration tool (CLI and wizard-ready)
- Safety shell (runtime checks + shared-control fallback hooks)
- Open release docs (field guide, pilot protocol, reproducibility checklist)

## Status
This is an implementation scaffold with interfaces, message flow, and build layout in place.
It is intended to be the foundation for the v0.1 build and micro-pilot.

## Tested assumptions
- ROS 2 Humble on Ubuntu 22.04
- Nav2 released for Humble
- No GPU required

## Packages
- `capnav_lite_nav2_plugin`: Nav2 controller plugin that consumes a user profile JSON and applies capability-aware costs.
- `capnav_lite_calibration`: calibration routines that estimate turning cost, backtracking cost, and slip (action noise) and write a profile JSON.
- `capnav_lite_safety_shell`: safety monitor node that checks action validity and triggers fallback / E-stop interface hooks.
- `schemas`: JSON schema for the user profile.
- `docs`: field guide and micro-pilot protocol.

## Quick start (build)
```bash
mkdir -p ~/capnav_ws/src
cd ~/capnav_ws/src
# copy this repo here
cd ..
rosdep update
rosdep install --from-paths src -y --ignore-src
colcon build --symlink-install
source install/setup.bash
```

## Run (simulation first)
1. Launch Nav2 with your preferred robot or wheelchair base model.
2. Start the safety shell:
```bash
ros2 run capnav_lite_safety_shell safety_shell_node --ros-args -p profile_path:=/path/to/profile.json
```
3. Use CapNav-Lite controller by setting the Nav2 controller plugin to `capnav_lite/CapNavLiteController`.

## Profile format
See `schemas/user_profile.schema.json` and `docs/Field_Guide.md`.

## Safety notice
CapNav-Lite is not a certified medical device. For real hardware trials, use clinician oversight, physical E-stop hardware, conservative limits, and a stop checklist. See `docs/Micro_Pilot_Protocol.md`.

## License
Apache-2.0 (permissive open source).
