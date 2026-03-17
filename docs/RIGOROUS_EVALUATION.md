# Rigorous Shared-Control Evaluation

This repo now includes a stricter held-out evaluation path intended to answer four common reviewer concerns.

## What changed

1. **Larger held-out sample**
   The default rigorous benchmark evaluates 30 held-out episodes:
   - 5 scenario seeds
   - 3 user seeds
   - 2 user models

2. **Multiple layout families**
   Held-out scenarios no longer come from one wall-and-doorway template. The evaluator cycles through five clinic-style families:
   - double corridor
   - loop ward
   - slalom corridor
   - crossing
   - open clutter

3. **Independent synthetic user model**
   In addition to the original pursuit-style pilot, the evaluator includes an autoregressive pilot model with delayed pose, input inertia, bias drift, and stochastic perturbations. This is still synthetic. It is more independent than the original pilot model, but it is not a substitute for real joystick traces.

4. **Sensitivity probe on controller constants**
   The controller constants that were previously hard-coded are now exposed in `SharedControlParams` and can be perturbed directly.

## Default report

The latest saved report is in `artifacts/rigorous_shared_control_report.json`.

Aggregate held-out result over 30 episodes:
- baseline success: 0.30
- improved success: 0.3333
- baseline collision: 0.3333
- improved collision: 0.40
- baseline path efficiency: 0.30
- improved path efficiency: 0.3333
- baseline mean clearance: 0.2533 m
- improved mean clearance: 0.3495 m

Per-user-model result:
- pursuit pilot: success improves from 0.20 to 0.3333 while collision stays at 0.40
- autoregressive pilot: success decreases from 0.40 to 0.3333 and collision increases from 0.2667 to 0.40

## Honest interpretation

The stricter evaluation is more credible than the earlier single-template benchmark, but the controller is still not robust enough across user models. The new evaluation should be treated as a reviewer-facing stress test, not as evidence of deployment readiness.
