# Algorithm Upgrade Notes

This repo revision intentionally goes beyond the paper's grid-world benchmark.

The runtime shared-control layer now combines five ideas:

1. **Selective intervention**
   The autonomy intervenes only when the user command is materially less safe or materially less effective than the best sampled alternative.

2. **Chance-constrained risk scoring**
   Candidate trajectories are scored using an approximate joint collision probability over moving obstacles. Unsafe samples are heavily penalized.

3. **Blind-corner slowdown**
   The controller estimates corner risk from asymmetric side clearances and reduces speed near doorways and corridor intersections.

4. **Trap escape mode**
   When recent motion shows repeated stalls, the controller adds escape-biased turning candidates to break local minima.

5. **Last-line safety filter**
   Even after shared-control arbitration, the final command is filtered using front-clearance braking and dynamic time-to-collision checks.

This is still not a medical device stack. It is a stronger translational robotics baseline that is easier to port into Nav2 and easier to validate in simulation and on supervised hardware.
