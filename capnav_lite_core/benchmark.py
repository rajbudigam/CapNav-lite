from __future__ import annotations

from dataclasses import dataclass, asdict
import json
from pathlib import Path
import random
import statistics
from typing import Dict, Iterable, List, Sequence

from .occupancy import GridMap
from .profile import CapabilityProfile

ACTIONS = ["U", "D", "L", "R"]
DIRS = {
    "U": (0, -1),
    "D": (0, 1),
    "L": (-1, 0),
    "R": (1, 0),
}
OPPOSITE = {
    "U": "D",
    "D": "U",
    "L": "R",
    "R": "L",
}


class PaperCapabilityAwarePrior:
    """Faithful tabular prior used for paper-style grid reproduction.

    This module intentionally keeps the state as a grid cell `(x, y)` with four
    primitive actions. That matches the paper's methodological assumption and
    avoids the state-space explosion that appears when rich local features are
    forced into a tiny tabular adaptation budget.
    """

    def __init__(self, q: Dict[str, Dict[str, float]] | None = None):
        self.q: Dict[str, Dict[str, float]] = q or {}

    @staticmethod
    def state_key(cell: tuple[int, int]) -> str:
        return f"{cell[0]},{cell[1]}"

    def get(self, state_key: str) -> Dict[str, float]:
        if state_key not in self.q:
            self.q[state_key] = {action: 0.0 for action in ACTIONS}
        return self.q[state_key]

    def best_action(self, state_key: str) -> str:
        q_state = self.get(state_key)
        return max(q_state, key=q_state.get)

    def update(self, state_key: str, action: str, target: float, alpha: float) -> None:
        q_state = self.get(state_key)
        q_state[action] += alpha * (target - q_state[action])

    def save(self, path: str | Path) -> None:
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        with path.open("w", encoding="utf-8") as handle:
            json.dump({"q": self.q}, handle, indent=2, sort_keys=True)
            handle.write("\n")

    @classmethod
    def load(cls, path: str | Path) -> "PaperCapabilityAwarePrior":
        path = Path(path)
        with path.open("r", encoding="utf-8") as handle:
            payload = json.load(handle)
        return cls(q={k: {ak: float(av) for ak, av in v.items()} for k, v in payload.get("q", {}).items()})


@dataclass(slots=True)
class EvalResult:
    reach: float
    final_return: float
    path_efficiency: float
    spl: float
    steps: int


@dataclass(slots=True)
class BenchmarkSummary:
    scratch: Dict[str, float]
    pretrained: Dict[str, float]
    delta_reach: float
    delta_efficiency: float
    config: Dict[str, float | int]


@dataclass(slots=True)
class MultiSeedBenchmarkSummary:
    seeds: list[int]
    per_seed: list[dict]
    scratch_mean: Dict[str, float]
    pretrained_mean: Dict[str, float]
    mean_delta_reach: float
    mean_delta_efficiency: float


def _random_profile(rng: random.Random) -> CapabilityProfile:
    return CapabilityProfile(
        user_id=f"meta-{rng.randint(0, 99999)}",
        turning_cost=rng.uniform(0.5, 2.0),
        backtracking_cost=rng.uniform(0.5, 2.0),
        slip_probability=rng.uniform(0.05, 0.20),
    )


def _fixed_hard_profile() -> CapabilityProfile:
    return CapabilityProfile(
        user_id="hard-eval",
        turning_cost=1.5,
        backtracking_cost=1.2,
        slip_probability=0.15,
    )


def _apply_action(
    grid: GridMap,
    cell: tuple[int, int],
    action: str,
    slip_probability: float,
    rng: random.Random,
) -> tuple[tuple[int, int], str, bool]:
    actual_action = action
    if rng.random() < slip_probability:
        actual_action = rng.choice(ACTIONS)
    dx, dy = DIRS[actual_action]
    candidate = (cell[0] + dx, cell[1] + dy)
    collision = not grid.is_free(candidate)
    if collision:
        candidate = cell
    return candidate, actual_action, collision


def _reward(
    previous_cell: tuple[int, int],
    next_cell: tuple[int, int],
    goal: tuple[int, int],
    actual_action: str,
    collision: bool,
    previous_action: str | None,
    profile: CapabilityProfile,
) -> float:
    reward = -1.0
    if collision:
        reward -= 4.0
    if previous_action is not None and actual_action != previous_action:
        reward -= profile.turning_cost
    if previous_action is not None and actual_action == OPPOSITE[previous_action]:
        reward -= profile.backtracking_cost
    if next_cell == goal:
        reward += 50.0
    return reward


def _clone_q(prior: PaperCapabilityAwarePrior | None) -> Dict[str, Dict[str, float]]:
    if prior is None:
        return {}
    return {state: dict(values) for state, values in prior.q.items()}


def _get_q(q: Dict[str, Dict[str, float]], state_key: str) -> Dict[str, float]:
    if state_key not in q:
        q[state_key] = {action: 0.0 for action in ACTIONS}
    return q[state_key]


def train_paper_prior(
    *,
    num_tasks: int = 20,
    episodes_per_task: int = 150,
    width: int = 12,
    height: int = 12,
    obstacle_probability: float = 0.30,
    alpha: float = 0.10,
    gamma: float = 0.99,
    epsilon_start: float = 1.0,
    epsilon_decay: float = 0.99,
    seed: int = 7,
) -> PaperCapabilityAwarePrior:
    rng = random.Random(seed)
    task_qs: list[dict[str, dict[str, float]]] = []

    for task_idx in range(num_tasks):
        grid, start, goal = GridMap.random_solvable(width, height, obstacle_probability, seed + task_idx)
        profile = _random_profile(rng)
        q: dict[str, dict[str, float]] = {}

        for episode in range(episodes_per_task):
            epsilon = epsilon_start * (epsilon_decay ** episode)
            cell = start
            previous_action: str | None = None
            for _step in range(300):
                state = PaperCapabilityAwarePrior.state_key(cell)
                q_state = _get_q(q, state)
                if rng.random() < epsilon:
                    action = rng.choice(ACTIONS)
                else:
                    action = max(q_state, key=q_state.get)
                next_cell, actual_action, collision = _apply_action(grid, cell, action, profile.slip_probability, rng)
                reward = _reward(cell, next_cell, goal, actual_action, collision, previous_action, profile)
                next_state = PaperCapabilityAwarePrior.state_key(next_cell)
                target = reward + gamma * max(_get_q(q, next_state).values())
                q_state[action] += alpha * (target - q_state[action])
                previous_action = actual_action
                cell = next_cell
                if cell == goal:
                    break
        task_qs.append(q)

    all_states = set().union(*(set(q.keys()) for q in task_qs))
    averaged_q: dict[str, dict[str, float]] = {}
    for state in all_states:
        averaged_q[state] = {
            action: statistics.fmean(task_q.get(state, {}).get(action, 0.0) for task_q in task_qs)
            for action in ACTIONS
        }
    return PaperCapabilityAwarePrior(averaged_q)


def _run_single_agent(
    grid: GridMap,
    start: tuple[int, int],
    goal: tuple[int, int],
    profile: CapabilityProfile,
    prior: PaperCapabilityAwarePrior | None,
    adaptation_episodes: int,
    alpha: float,
    gamma: float,
    epsilon_start: float,
    epsilon_decay: float,
    rollout_cap: int,
    seed: int,
) -> EvalResult:
    rng = random.Random(seed)
    q = _clone_q(prior)

    for episode in range(adaptation_episodes):
        epsilon = epsilon_start * (epsilon_decay ** episode)
        cell = start
        previous_action: str | None = None
        for _step in range(rollout_cap):
            state = PaperCapabilityAwarePrior.state_key(cell)
            q_state = _get_q(q, state)
            if rng.random() < epsilon:
                action = rng.choice(ACTIONS)
            else:
                action = max(q_state, key=q_state.get)
            next_cell, actual_action, collision = _apply_action(grid, cell, action, profile.slip_probability, rng)
            reward = _reward(cell, next_cell, goal, actual_action, collision, previous_action, profile)
            next_state = PaperCapabilityAwarePrior.state_key(next_cell)
            q_state[action] += alpha * (reward + gamma * max(_get_q(q, next_state).values()) - q_state[action])
            previous_action = actual_action
            cell = next_cell
            if cell == goal:
                break

    cell = start
    previous_action = None
    total_return = 0.0
    path = [cell]

    for step in range(rollout_cap):
        state = PaperCapabilityAwarePrior.state_key(cell)
        q_state = _get_q(q, state)
        action = max(q_state, key=q_state.get)
        next_cell, actual_action, collision = _apply_action(grid, cell, action, profile.slip_probability, rng)
        total_return += _reward(cell, next_cell, goal, actual_action, collision, previous_action, profile)
        previous_action = actual_action
        cell = next_cell
        if cell != path[-1]:
            path.append(cell)
        if cell == goal:
            shortest = max(1, len(grid.a_star(start, goal)) - 1)
            executed = max(1, len(path) - 1)
            efficiency = shortest / executed
            return EvalResult(1.0, total_return, efficiency, efficiency, step + 1)

    return EvalResult(0.0, total_return, 0.0, 0.0, rollout_cap)


def _aggregate(results: Sequence[EvalResult]) -> Dict[str, float]:
    return {
        "reach": round(statistics.fmean(result.reach for result in results), 4),
        "final_return": round(statistics.fmean(result.final_return for result in results), 4),
        "path_efficiency": round(statistics.fmean(result.path_efficiency for result in results), 4),
        "spl": round(statistics.fmean(result.spl for result in results), 4),
        "steps": round(statistics.fmean(result.steps for result in results), 4),
    }


def run_benchmark(
    *,
    eval_maps: int = 24,
    adaptation_episodes: int = 60,
    meta_tasks: int = 20,
    seed: int = 13,
    obstacle_probability: float = 0.30,
) -> BenchmarkSummary:
    prior = train_paper_prior(
        num_tasks=meta_tasks,
        episodes_per_task=150,
        width=12,
        height=12,
        obstacle_probability=obstacle_probability,
        seed=seed,
    )
    scratch_results: list[EvalResult] = []
    pretrained_results: list[EvalResult] = []
    profile = _fixed_hard_profile()

    for idx in range(eval_maps):
        grid, start, goal = GridMap.random_solvable(12, 12, obstacle_probability, seed + 100 + idx)
        scratch_results.append(
            _run_single_agent(
                grid,
                start,
                goal,
                profile,
                None,
                adaptation_episodes,
                0.10,
                0.99,
                1.0,
                0.99,
                300,
                seed + idx,
            )
        )
        pretrained_results.append(
            _run_single_agent(
                grid,
                start,
                goal,
                profile,
                prior,
                adaptation_episodes,
                0.10,
                0.99,
                1.0,
                0.99,
                300,
                seed + 500 + idx,
            )
        )

    scratch = _aggregate(scratch_results)
    pretrained = _aggregate(pretrained_results)
    return BenchmarkSummary(
        scratch=scratch,
        pretrained=pretrained,
        delta_reach=round(pretrained["reach"] - scratch["reach"], 4),
        delta_efficiency=round(pretrained["path_efficiency"] - scratch["path_efficiency"], 4),
        config={
            "eval_maps": eval_maps,
            "adaptation_episodes": adaptation_episodes,
            "meta_tasks": meta_tasks,
            "seed": seed,
            "obstacle_probability": obstacle_probability,
        },
    )


def run_multiseed_benchmark(
    *,
    seeds: Sequence[int] = (7, 13, 42, 99, 123),
    eval_maps: int = 24,
    adaptation_episodes: int = 60,
    meta_tasks: int = 20,
    obstacle_probability: float = 0.30,
) -> MultiSeedBenchmarkSummary:
    per_seed: list[dict] = []
    scratch_rows: list[dict[str, float]] = []
    pretrained_rows: list[dict[str, float]] = []

    for seed in seeds:
        summary = run_benchmark(
            eval_maps=eval_maps,
            adaptation_episodes=adaptation_episodes,
            meta_tasks=meta_tasks,
            seed=seed,
            obstacle_probability=obstacle_probability,
        )
        per_seed.append({
            "seed": seed,
            "scratch": summary.scratch,
            "pretrained": summary.pretrained,
            "delta_reach": summary.delta_reach,
            "delta_efficiency": summary.delta_efficiency,
        })
        scratch_rows.append(summary.scratch)
        pretrained_rows.append(summary.pretrained)

    def _mean_dict(rows: Sequence[Dict[str, float]]) -> Dict[str, float]:
        keys = rows[0].keys()
        return {key: round(statistics.fmean(row[key] for row in rows), 4) for key in keys}

    scratch_mean = _mean_dict(scratch_rows)
    pretrained_mean = _mean_dict(pretrained_rows)
    return MultiSeedBenchmarkSummary(
        seeds=list(seeds),
        per_seed=per_seed,
        scratch_mean=scratch_mean,
        pretrained_mean=pretrained_mean,
        mean_delta_reach=round(statistics.fmean(item["delta_reach"] for item in per_seed), 4),
        mean_delta_efficiency=round(statistics.fmean(item["delta_efficiency"] for item in per_seed), 4),
    )


def save_benchmark(summary: BenchmarkSummary | MultiSeedBenchmarkSummary, path: str | Path) -> None:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as handle:
        json.dump(asdict(summary), handle, indent=2, sort_keys=True)
        handle.write("\n")
