# Paper Reproduction Notes

This repository now includes a paper-faithful benchmark path.

What changed:
- The benchmark state is now the grid cell `(x, y)`.
- The action space is now exactly four primitive moves: `U`, `D`, `L`, `R`.
- The hard benchmark uses the fixed evaluation profile described in the paper: `(turning_cost=1.5, backtracking_cost=1.2, slip_probability=0.15)`.
- Meta-training uses 20 source tasks and 150 episodes per task.
- Hard adaptation uses 60 episodes and a 300-step greedy rollout cap.
- The prior is formed by averaging task-specific Q-tables, which matches the wording of the paper more closely than a single giant shared table.

Why this matters:
A richer local-feature key is useful at runtime for a continuous local planner, but it expands the tabular state space far beyond what a small meta-learning budget can support. The paper does not claim that richer state. It claims a small, interpretable tabular prior over grid cells. This split keeps the repo honest.
