# Architecture

CapNav-Lite in this repository is split into three technically distinct layers.

## 1. Capability model

A user profile stores both hard and soft limits.

Hard limits:
- maximum linear speed
- maximum angular speed
- clearance margin

Soft limits:
- turning cost
- backtracking cost
- slip probability
- reaction time

The planner never exceeds hard limits. Soft limits shape action scoring and online adaptation.

## 2. Capability-aware prior

The paper's core idea is preserved: build a reusable prior across many navigation tasks, then adapt quickly to a new user and a new environment.

This repo changes the state representation from exact grid coordinates to a compact local feature key:
- front clearance bucket
- left and right clearance buckets
- heading delta to goal
- distance bucket
- turning-cost bucket
- slip bucket
- reversing flag

That lets one prior transfer across maps and across local positions.

## 3. Runtime controller

The runtime controller does not execute raw tabular actions directly. Instead it scores velocity primitives using:
- progress along the global path
- heading improvement
- obstacle clearance
- turning discomfort penalty
- backtracking penalty
- prior bonus from the learned table
- command continuity bonus

This makes the system continuous enough for a Nav2 controller plugin while still grounded in the original prior-learning idea.

## 4. Safety and shared control

A separate safety shell applies independent checks before anything reaches the base controller:
- time-to-collision threshold
- hard-stop clearance threshold
- speed clamping
- shared-control blending under elevated risk
- optional emergency-stop topic

The safety shell is intentionally separate from the planner so that a planning bug cannot silently disable the last line of defense.
