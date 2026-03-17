# V10 Safety Shield Update

This revision pushes the controller toward a harder safety-first shared-control design.

## What changed

- Added a trajectory safety shield that re-checks smoothed commands over a short horizon before execution.
- Added frontal dynamic-conflict detection in the robot frame.
- Added stronger same-lane side-pass primitives (`SFL`, `SFR`) and a temporary avoidance target inside the local lookahead logic.
- Added a persistent safe-fail wait mode for stacked same-lane plus cross-traffic conflicts.
- Increased risky-state lookahead for same-lane dynamic conflicts.
- Disabled smoothing in risky states so the shielded command is the command that actually gets executed.

## Held-out benchmark result

20 held-out scenario seeds x 2 pilot models x 3 user seeds = 120 episodes per policy.

Fixed blend:
- success: 27.5%
- collision: 33.3%
- path efficiency: 0.275

Improved controller:
- success: 35.8%
- collision: 14.2%
- path efficiency: 0.358

Paired deltas vs fixed blend:
- success: +8.3 points
- collision: -19.2 points
- path efficiency: +0.083
- McNemar exact p-value on success: 0.01294

## Family-level behavior

- Crossing: collisions collapse from 100% to 4.2%, but success is still 0%
- Double corridor: success improves from 4.2% to 16.7%
- Loop ward: success improves from 37.5% to 66.7%
- Open clutter: both stay at 95.8% success and 0% collision
- Slalom: collisions improve from 66.7% to 58.3%, but success is still 0%

## Honest take

This is a meaningfully better result than the earlier publishable benchmark because it is now better on the three top-line metrics that matter most: success, collision, and path efficiency.

It is still not perfect. The remaining weakness is slalom-style dynamic weaving, and the controller is still using a synthetic pilot model rather than real wheelchair traces.
