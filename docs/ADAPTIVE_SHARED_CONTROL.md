# Adaptive Shared Control Upgrade

This repo now includes a more deployable shared-control layer in `capnav_lite_core/adaptive_shared_control.py`.

## What changed

The original paper benchmark is intentionally grid based. It is useful for showing fast adaptation, but it is not a runtime controller.

The adaptive shared-control module moves closer to a real wheelchair stack by adding:

- continuous pose updates with unicycle dynamics
- dynamic obstacles that move through corridors and doorways
- noisy pilot commands that simulate slip and oversteer
- a baseline fixed-blend shared-control controller
- an upgraded controller with:
  - selective intervention instead of always-on override
  - short-horizon sampled trajectory rollouts
  - chance-constrained dynamic-risk scoring over predicted obstacle motion
  - online user reliability estimation from slip, oscillation, stalls, and intervention history
  - blind-corner slowdown to handle doorway and corridor occlusions
  - local-minima escape behavior when the robot stalls near obstacles
  - a final emergency safety filter that clamps unsafe commands

## Recommended simulator setting

The current recommended simulator-oriented setting is:

- `horizon = 6`
- `samples = 20`
- `risk_penalty = 7.0`
- `margin = 0.32`
- `agreement = 0.4`
- `max_steps = 160`

The evaluator script now uses this setting by default.

## Held-out benchmark snapshot

Using held-out scenarios `[100, 101, 102, 103]` and user seeds `[3, 7, 11]`:

- baseline success: `0.0833`
- upgraded success: `0.7500`
- baseline collision rate: `0.0000`
- upgraded collision rate: `0.0000`
- baseline mean clearance: `0.2333 m`
- upgraded mean clearance: `0.2876 m`

These results are stored in `artifacts/dynamic_shared_control_v5_report.json`.

## Benchmark interpretation

This benchmark is still synthetic. It is more realistic than the paper grid task, but it is not a substitute for:

- ROS 2 integration tests on the target wheelchair base
- LiDAR or depth sensor latency tests
- hardware emergency-stop validation
- supervised indoor pilot trials
- regulatory and clinical safety work

Use it as a pressure test for algorithm design, not as proof of deployment readiness.
