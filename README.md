# CapNav-Lite Research Grade Starter

CapNav-Lite is a capability-aware assistive navigation stack for powered mobility platforms.
This repository turns the original grid-world idea into a practical software system with four layers:

1. **Offline capability-aware prior training** over diverse synthetic maps
2. **Short user calibration** from joystick and odometry traces
3. **Runtime local planning** that scores safe velocity commands using path progress, clearance, comfort, and learned priors
4. **Safety and shared-control shell** that clamps speed, estimates collision risk, and blends user intent with autonomy

It is designed for **ROS 2 Humble + Nav2** and for staged validation:

- synthetic benchmark
- simulator validation
- supervised indoor hardware pilot
- only then formal clinical or commercial programs

## Why this repo exists

The CoRL 2025 paper behind the project shows that a capability-aware prior learned with tabular Q-learning can dramatically improve fast adaptation on hard navigation tasks. The paper models the state as a grid cell `(r, c)` with four actions, 20 meta-training tasks, 150 episodes per task, and a 60-episode hard adaptation budget. This repo now separates two concerns cleanly: a paper-faithful benchmark path that reproduces that tabular result, and a richer runtime stack for continuous local planning, calibration, and safety on top of Nav2. The paper also openly notes its main limitation: it uses a grid proxy rather than real continuous control and real sensors. This repo addresses that gap by keeping the capability-aware idea, then wrapping it in a real local planner, a calibration pipeline, and a safety shell for continuous runtime use.

## What is included

### Pure Python core

- `capnav_lite_core/`: profile models, prior training, occupancy maps, local planner, safety envelope, calibration, and synthetic benchmark tools

### ROS 2 packages

- `capnav_lite_nav2_plugin/`: C++ Nav2 controller plugin with capability-aware action scoring
- `capnav_lite_runtime/`: ROS 2 Python nodes for shared control, safety monitoring, and calibration import/export
- `capnav_lite_bringup/`: launch files and runtime config

### Research and deployment material

- `schemas/`: JSON schema for capability profiles and prior tables
- `docs/`: architecture, validation plan, pilot protocol, and deployment roadmap
- `tools/`: scripts to train priors, generate maps, and run end-to-end benchmarks
- `tests/`: unit tests for the Python core

## Quick start for the Python benchmark

```bash
python -m venv .venv
source .venv/bin/activate
pip install -e .[dev]
pytest
python tools/train_capability_prior.py --output priors/sample_prior.json
python tools/evaluate_synthetic_benchmark.py --output artifacts/paper_hard_summary.json
python tools/evaluate_synthetic_benchmark.py --multiseed --output artifacts/paper_hard_multiseed.json
```


## Paper-faithful benchmark path

The repository now ships with a paper-faithful synthetic benchmark that uses the exact tabular state form described in the paper: grid-cell state, four primitive actions, random slip, turning and backtracking costs in the reward, 20 meta-training tasks, 150 episodes per task, and 60 adaptation episodes on the hard regime. Use this path for any claim about reproducing the paper.

The richer feature-based prior in `capnav_lite_core/priors.py` is still used by the runtime planner, but it is no longer used to justify the paper benchmark. That avoids a state-space mismatch between the benchmark and the paper.


## Adaptive shared-control benchmark

To move beyond the original grid proxy, the repo now includes a more realistic dynamic clinic benchmark in `capnav_lite_core/adaptive_shared_control.py`.
This benchmark adds:

- continuous pose updates with unicycle dynamics
- moving obstacles that cross corridors and doorways
- noisy pilot commands that simulate slip and oversteer
- a baseline fixed-blend shared-control policy
- an improved selective intervention policy with short-horizon trajectory rollouts, online reliability estimation, chance-constrained dynamic-risk scoring, blind-corner slowdown, trap-escape behavior, and a final emergency safety filter

Run it with:

```bash
python tools/evaluate_dynamic_shared_control.py --out artifacts/dynamic_shared_control_report.json
```

The default evaluator now uses a more practical "pragmatic" setting for the upgraded controller:

- horizon = 6
- samples = 20
- risk_penalty = 7.0
- margin = 0.32
- agreement = 0.4
- train scenarios = [0, 1, 2]
- held-out test scenarios = [100, 101, 102, 103]
- user seeds = [3, 7, 11]

This path is still a simulator benchmark, not a clinical claim. Its purpose is to pressure-test whether the shared-control algorithm remains useful when the environment is dynamic, partially blind at corners, and the user is imperfect.

For a stricter held-out result, use the rigorous evaluator below. That benchmark expands the held-out set, uses multiple layout families, includes an independent autoregressive pilot model, and reports confidence intervals instead of relying on a small lucky seed.


## Rigorous held-out evaluation

The more defensible benchmark path is `tools/evaluate_rigorous_shared_control.py`.
It now defaults to:

- 8 training scenarios
- 10 held-out test scenarios
- 5 layout families
- 2 pilot models
- 3 user seeds
- 60 held-out episodes total

Run it with:

```bash
python tools/evaluate_rigorous_shared_control.py --out artifacts/rigorous_shared_control_report.json
```

This benchmark is the right place to compare controller revisions because it reduces small-sample noise and makes generalization failures visible.

## Real trace ingestion

The repo now has a real-trace ingestion path in `capnav_lite_core.trace_replay` and notes in `docs/REAL_TRACE_INTEGRATION.md`.
This supports strict velocity traces, flexible joystick-axis CSVs, and best-effort WheelSim-style Unity exports.

## Quick start for ROS 2

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

## Safety note

This repo is **not** a certified medical device and should **not** be deployed unsupervised on real users without staged validation, hardware interlocks, institutional review where needed, and regulatory review. The software is intended to help you build a serious research or translational robotics system, not to skip safety validation.

## Suggested paper framing

For a paper or release, describe this codebase as:

> A research-grade, capability-aware assistive navigation stack that operationalizes the CapNav-Lite prior-learning idea for continuous local planning, fast per-user calibration, and safety-constrained shared control.

## Repository map

```text
capnav_lite_core/              Python core logic and benchmark
capnav_lite_nav2_plugin/       C++ Nav2 controller plugin
capnav_lite_runtime/           ROS 2 runtime nodes in Python
capnav_lite_bringup/           Launch files and configs
schemas/                       JSON schemas
profiles/                      Example user profiles
priors/                        Example learned prior
sim/                           Example simulator maps and routes
tools/                         Training and evaluation scripts
tests/                         Python tests
docs/                          Technical and validation docs
```
