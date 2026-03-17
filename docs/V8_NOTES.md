# V8 Notes

## What changed

This revision replaces the previous heavy sampled controller with a simpler two-step lattice scorer.

At each control step the controller:
1. scores a small discrete action set plus the user's proposed command
2. simulates one step plus a path-following second step
3. rewards path progress and heading alignment
4. penalizes low clearance and short time-to-collision
5. applies a final emergency override if the chosen command is still unsafe

## Why this changed

The stricter held-out evaluation exposed that the older controller was too brittle across pilot models.
The new controller is simpler, easier to audit, and performed better on the 30-episode held-out benchmark.

## Real trace status

The repo already includes a trace replay interface in `capnav_lite_core/trace_replay.py`.
In this environment no public wheelchair trace dataset was downloaded into the repo, so the V8 benchmark still uses synthetic pilots.
A good public candidate for later integration is WheelSimPhysio-2023, which reports simulator performance CSV data from 58 participants.
