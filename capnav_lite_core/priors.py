from __future__ import annotations

from dataclasses import dataclass, field
import json
import math
from pathlib import Path
import random
from typing import Dict, Iterable, List, Tuple

from .occupancy import GridMap
from .profile import CapabilityProfile

ACTIONS = ["F", "FL", "FR", "L", "R", "B", "S"]
DIRS = [(1, 0), (0, 1), (-1, 0), (0, -1)]


def _turn_left(d: int) -> int:
    return (d + 1) % 4


def _turn_right(d: int) -> int:
    return (d - 1) % 4


@dataclass
class CapabilityAwarePrior:
    q: Dict[str, Dict[str, float]] = field(default_factory=dict)
    visits: Dict[str, int] = field(default_factory=dict)

    def state_key(
        self,
        grid: GridMap,
        cell: tuple[int, int],
        direction: int,
        goal: tuple[int, int],
        profile: CapabilityProfile,
        reversing: bool,
    ) -> str:
        front = self._clearance_bin(grid, cell, DIRS[direction])
        left = self._clearance_bin(grid, cell, DIRS[_turn_left(direction)])
        right = self._clearance_bin(grid, cell, DIRS[_turn_right(direction)])
        dx = goal[0] - cell[0]
        dy = goal[1] - cell[1]
        goal_dir = 0 if abs(dx) >= abs(dy) and dx >= 0 else 2 if abs(dx) >= abs(dy) else 1 if dy >= 0 else 3
        heading_delta = (goal_dir - direction) % 4
        if heading_delta == 3:
            heading_delta = -1
        elif heading_delta == 2:
            heading_delta = 2
        dist_bin = min(4, int((abs(dx) + abs(dy)) / 3))
        turn_bin = 0 if profile.turning_cost < 1.0 else 1 if profile.turning_cost < 1.8 else 2
        slip_bin = 0 if profile.slip_probability < 0.07 else 1 if profile.slip_probability < 0.15 else 2
        return f"f{front}|l{left}|r{right}|h{heading_delta}|d{dist_bin}|t{turn_bin}|s{slip_bin}|b{int(reversing)}"

    def _clearance_bin(self, grid: GridMap, cell: tuple[int, int], delta: tuple[int, int]) -> int:
        x, y = cell
        steps = 0
        while steps < 3:
            x += delta[0]
            y += delta[1]
            steps += 1
            if not grid.is_free((x, y)):
                return steps - 1
        return 3

    def get(self, state_key: str) -> Dict[str, float]:
        if state_key not in self.q:
            self.q[state_key] = {action: 0.0 for action in ACTIONS}
            self.visits[state_key] = 0
        return self.q[state_key]

    def best_action(self, state_key: str) -> str:
        q_state = self.get(state_key)
        return max(q_state, key=q_state.get)

    def update(self, state_key: str, action: str, target: float, alpha: float) -> None:
        q_state = self.get(state_key)
        q_state[action] += alpha * (target - q_state[action])
        self.visits[state_key] = self.visits.get(state_key, 0) + 1

    def to_dict(self) -> dict:
        return {"q": self.q, "visits": self.visits}

    @classmethod
    def from_dict(cls, payload: dict) -> "CapabilityAwarePrior":
        return cls(q={k: {ak: float(av) for ak, av in v.items()} for k, v in payload.get("q", {}).items()}, visits={k: int(v) for k, v in payload.get("visits", {}).items()})

    def save(self, path: str | Path) -> None:
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        with path.open("w", encoding="utf-8") as handle:
            json.dump(self.to_dict(), handle, indent=2, sort_keys=True)
            handle.write("\n")

    @classmethod
    def load(cls, path: str | Path) -> "CapabilityAwarePrior":
        path = Path(path)
        with path.open("r", encoding="utf-8") as handle:
            return cls.from_dict(json.load(handle))


def _apply_action(grid: GridMap, cell: tuple[int, int], direction: int, action: str, slip: float, rng: random.Random) -> tuple[tuple[int, int], int, bool, bool]:
    actual = action
    if rng.random() < slip and action not in {"S", "B"}:
        actual = rng.choice(["FL", "FR", "L", "R", "F"])
    new_dir = direction
    reversing = False
    if actual == "L":
        new_dir = _turn_left(direction)
    elif actual == "R":
        new_dir = _turn_right(direction)
    elif actual == "FL":
        new_dir = _turn_left(direction)
    elif actual == "FR":
        new_dir = _turn_right(direction)
    step_dir = new_dir if actual != "B" else (direction + 2) % 4
    target = cell
    if actual in {"F", "FL", "FR", "B"}:
        dx, dy = DIRS[step_dir]
        candidate = (cell[0] + dx, cell[1] + dy)
        if grid.is_free(candidate):
            target = candidate
        else:
            return cell, new_dir, actual == "B", True
    reversing = actual == "B"
    return target, new_dir, reversing, False


def _reward(
    previous_cell: tuple[int, int],
    next_cell: tuple[int, int],
    goal: tuple[int, int],
    action: str,
    hit_obstacle: bool,
    profile: CapabilityProfile,
) -> float:
    prev_dist = abs(goal[0] - previous_cell[0]) + abs(goal[1] - previous_cell[1])
    next_dist = abs(goal[0] - next_cell[0]) + abs(goal[1] - next_cell[1])
    reward = -1.0 + (prev_dist - next_dist) * 1.25
    if hit_obstacle:
        reward -= 12.0
    if action in {"L", "R", "FL", "FR"}:
        reward -= 0.35 * profile.turning_cost
    if action == "B":
        reward -= 0.75 * profile.backtracking_cost
    if next_cell == goal:
        reward += 40.0
    return reward


def _sample_profile(rng: random.Random) -> CapabilityProfile:
    return CapabilityProfile(
        user_id=f"synthetic-{rng.randint(0, 99999)}",
        turning_cost=rng.uniform(0.5, 2.2),
        backtracking_cost=rng.uniform(0.5, 2.0),
        slip_probability=rng.uniform(0.03, 0.2),
        max_linear_speed_mps=rng.uniform(0.3, 0.55),
        comfort_linear_speed_mps=rng.uniform(0.18, 0.35),
        max_angular_speed_rad_s=rng.uniform(0.45, 1.0),
        comfort_angular_speed_rad_s=rng.uniform(0.28, 0.65),
        clearance_margin_m=rng.uniform(0.22, 0.40),
    )


def train_prior_on_random_maps(
    *,
    num_tasks: int = 16,
    episodes_per_task: int = 80,
    width: int = 12,
    height: int = 12,
    obstacle_probability: float = 0.24,
    alpha: float = 0.12,
    gamma: float = 0.96,
    epsilon_start: float = 0.9,
    seed: int = 7,
) -> CapabilityAwarePrior:
    rng = random.Random(seed)
    prior = CapabilityAwarePrior()
    for task_idx in range(num_tasks):
        grid, start, goal = GridMap.random_solvable(width, height, obstacle_probability, seed + task_idx)
        profile = _sample_profile(rng)
        for episode in range(episodes_per_task):
            cell = start
            direction = 0
            reversing = False
            epsilon = epsilon_start * (0.985 ** episode)
            for _step in range(width * height * 3):
                state = prior.state_key(grid, cell, direction, goal, profile, reversing)
                if rng.random() < epsilon:
                    action = rng.choice(ACTIONS)
                else:
                    action = prior.best_action(state)
                next_cell, next_dir, reversing, hit_obstacle = _apply_action(grid, cell, direction, action, profile.slip_probability, rng)
                reward = _reward(cell, next_cell, goal, action, hit_obstacle, profile)
                next_state = prior.state_key(grid, next_cell, next_dir, goal, profile, reversing)
                target = reward + gamma * max(prior.get(next_state).values())
                prior.update(state, action, target, alpha)
                cell, direction = next_cell, next_dir
                if cell == goal:
                    break
    return prior
