# CapNav-Lite publishable implementation pass

This repo revision implements the requested publishability plan in code and evaluation.

Implemented items:
- fixed four controller bugs in post-smoothing safety, dynamic-obstacle timing, intervention accounting, and stale clearance use
- added `WAIT` primitive and adaptive 2-step / 3-step horizon logic
- separated intervention metrics into safety, comfort, and override rates
- added three explicit baselines: `user_safety`, `fixed_blend`, and `pure_autonomy`
- added five ablations: no lattice scoring, fixed threshold, no reliability estimation, no blind-corner slowdown, and no dynamic prediction
- expanded held-out evaluation to 20 scenario seeds x 2 pilot models x 3 user seeds = 120 episodes per policy
- added McNemar exact test, paired bootstrap deltas, and per-family breakdown rows
- exercised the trace-replay path with a synthetic CSV round-trip experiment

Key held-out results from `artifacts/publishable/publishable_benchmark_report.json`:
- fixed_blend success: 0.2667
- fixed_blend collision: 0.3417
- improved success: 0.3667
- improved collision: 0.4167
- paired delta success mean: +0.1000
- McNemar exact p-value vs fixed_blend: 0.00418
- synthetic trace round-trip exact match rate: 1.0

Interpretation:
- the revised controller clears the main statistical hurdle on success versus the fixed-blend baseline in this synthetic held-out benchmark
- the controller is not yet better on collision rate, so the success gain is coming with a safety tradeoff that still needs another iteration before a strong robotics venue submission
