# CapNav-Lite

CapNav-Lite is a capability-aware assistive navigation stack for powered mobility platforms. I built it to turn the original grid-world prior-learning idea into a practical software system for continuous local planning, fast per-user calibration, and safety-constrained shared control on top of ROS 2 Humble and Nav2.

This repository is the result of roughly ten iterations of design, debugging, evaluation, and controller refinement. I am especially grateful for the support that made this work possible, particularly the funding from **Emergent Ventures at the Mercatus Center**. Their support gave me the freedom to keep pushing this system from a simple research idea toward something far more useful and rigorous.

## What CapNav-Lite does

CapNav-Lite has four main parts:

1. **Offline capability-aware prior training** on synthetic navigation tasks  
2. **Short user calibration** from joystick and odometry traces  
3. **Runtime local planning** that scores safe velocity commands using path progress, clearance, comfort, and learned priors  
4. **A safety and shared-control shell** that estimates collision risk, limits unsafe motion, and balances user intent with autonomy  

In practice, the system is meant to support staged validation:

- synthetic benchmark
- simulator validation
- supervised indoor hardware pilot
- only then formal clinical or commercial translation

## Why I built this

The CoRL 2025 paper behind CapNav-Lite shows that a capability-aware prior learned with tabular Q-learning can improve fast adaptation on hard navigation tasks. But that original setup is a grid-world proxy. Real assistive navigation needs continuous control, local planning, calibration, moving obstacles, and explicit safety handling.

This repo separates those two layers cleanly:

- a **paper-faithful benchmark path** for reproducing the tabular result
- a **richer runtime stack** for continuous shared control and practical navigation experiments

That makes the project useful both as a research reproduction and as a stronger translational prototype.

## What is in the repo

### Python core
- `capnav_lite_core/` — profiles, priors, maps, calibration, local planning, safety logic, and benchmarks

### ROS 2 packages
- `capnav_lite_nav2_plugin/` — C++ Nav2 controller plugin
- `capnav_lite_runtime/` — Python runtime nodes for shared control, safety, and calibration
- `capnav_lite_bringup/` — launch files and configuration

### Research and support material
- `schemas/` — JSON schemas for profiles and priors
- `profiles/` — example user capability profiles
- `priors/` — example learned priors
- `tools/` — training and evaluation scripts
- `tests/` — unit tests
- `docs/` — architecture, validation, and deployment notes

## Benchmark paths

### 1. Paper-faithful benchmark
This reproduces the original tabular setup from the paper using grid-cell state, four actions, meta-training tasks, and hard adaptation episodes. This is the right path for any claim about reproducing the published benchmark.

### 2. Dynamic shared-control benchmark
This is the more realistic benchmark path. It includes:

- continuous pose updates with unicycle dynamics
- moving obstacles
- noisy pilot commands
- fixed-blend and selective-intervention policies
- safety filtering and shared-control logic

### 3. Rigorous held-out evaluation
This expands the test set, uses multiple layout families and pilot models, and reports stronger held-out results for controller comparison.

## Quick start

### Python benchmark
```bash
python -m venv .venv
source .venv/bin/activate
pip install -e .[dev]
pytest
python tools/train_capability_prior.py --output priors/sample_prior.json
python tools/evaluate_synthetic_benchmark.py --output artifacts/paper_hard_summary.json
python tools/evaluate_rigorous_shared_control.py --out artifacts/rigorous_shared_control_report.json
```

### ROS 2
```bash
mkdir -p ~/capnav_ws/src
cd ~/capnav_ws/src
# copy this repo into the workspace
cd ..
rosdep update
rosdep install --from-paths src -y --ignore-src
colcon build --symlink-install
source install/setup.bash
ros2 launch capnav_lite_bringup sim_demo.launch.py
```

## Real trace support

The repo includes trace replay support for velocity traces, joystick-axis CSVs, and best-effort WheelSim-style exports through `capnav_lite_core.trace_replay`.

## Safety note

CapNav-Lite is **not** a certified medical device. It should not be deployed unsupervised on real users without staged simulator validation, hardware safety interlocks, supervised pilots, and appropriate review.

## Suggested one-line description

**CapNav-Lite is a capability-aware assistive navigation stack for fast per-user calibration, continuous local planning, and safety-constrained shared control on powered mobility platforms.**
