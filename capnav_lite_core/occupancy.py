from __future__ import annotations

from dataclasses import dataclass
import heapq
import math
import random
from typing import Iterable, Iterator, List, Sequence, Tuple


@dataclass(slots=True)
class Pose2D:
    x: float
    y: float
    yaw: float


class GridMap:
    def __init__(self, width: int, height: int, resolution_m: float = 0.2, blocked: set[tuple[int, int]] | None = None):
        self.width = width
        self.height = height
        self.resolution_m = resolution_m
        self.blocked = blocked or set()

    @classmethod
    def from_ascii(cls, rows: Sequence[str], resolution_m: float = 0.2) -> tuple["GridMap", tuple[int, int], tuple[int, int]]:
        blocked: set[tuple[int, int]] = set()
        start = (0, 0)
        goal = (len(rows[0]) - 1, len(rows) - 1)
        for y, row in enumerate(rows):
            for x, char in enumerate(row):
                if char == "#":
                    blocked.add((x, y))
                elif char == "S":
                    start = (x, y)
                elif char == "G":
                    goal = (x, y)
        return cls(len(rows[0]), len(rows), resolution_m, blocked), start, goal

    @classmethod
    def random_solvable(
        cls,
        width: int,
        height: int,
        obstacle_probability: float,
        seed: int,
        resolution_m: float = 0.2,
    ) -> tuple["GridMap", tuple[int, int], tuple[int, int]]:
        rng = random.Random(seed)
        start = (0, 0)
        goal = (width - 1, height - 1)
        for _ in range(200):
            blocked = {
                (x, y)
                for y in range(height)
                for x in range(width)
                if (x, y) not in {start, goal} and rng.random() < obstacle_probability
            }
            grid = cls(width, height, resolution_m, blocked)
            if grid.a_star(start, goal):
                return grid, start, goal
        raise RuntimeError("failed to generate solvable map")

    def in_bounds(self, cell: tuple[int, int]) -> bool:
        x, y = cell
        return 0 <= x < self.width and 0 <= y < self.height

    def is_free(self, cell: tuple[int, int]) -> bool:
        return self.in_bounds(cell) and cell not in self.blocked

    def neighbors(self, cell: tuple[int, int]) -> Iterator[tuple[int, int]]:
        x, y = cell
        for nxt in ((x + 1, y), (x - 1, y), (x, y + 1), (x, y - 1)):
            if self.is_free(nxt):
                yield nxt

    def pose_to_cell(self, pose: Pose2D) -> tuple[int, int]:
        return (int(round(pose.x / self.resolution_m)), int(round(pose.y / self.resolution_m)))

    def cell_to_pose(self, cell: tuple[int, int], yaw: float = 0.0) -> Pose2D:
        return Pose2D(cell[0] * self.resolution_m, cell[1] * self.resolution_m, yaw)

    def heuristic(self, a: tuple[int, int], b: tuple[int, int]) -> int:
        return abs(a[0] - b[0]) + abs(a[1] - b[1])

    def a_star(self, start: tuple[int, int], goal: tuple[int, int]) -> list[tuple[int, int]]:
        frontier: list[tuple[int, tuple[int, int]]] = [(0, start)]
        came_from: dict[tuple[int, int], tuple[int, int] | None] = {start: None}
        cost_so_far: dict[tuple[int, int], int] = {start: 0}
        while frontier:
            _, current = heapq.heappop(frontier)
            if current == goal:
                break
            for nxt in self.neighbors(current):
                new_cost = cost_so_far[current] + 1
                if nxt not in cost_so_far or new_cost < cost_so_far[nxt]:
                    cost_so_far[nxt] = new_cost
                    priority = new_cost + self.heuristic(nxt, goal)
                    heapq.heappush(frontier, (priority, nxt))
                    came_from[nxt] = current
        if goal not in came_from:
            return []
        path = [goal]
        cur = goal
        while cur != start:
            cur = came_from[cur]  # type: ignore[assignment]
            assert cur is not None
            path.append(cur)
        return list(reversed(path))

    def front_clearance_m(self, pose: Pose2D, max_range_m: float = 2.0, step_m: float = 0.05) -> float:
        distance = 0.0
        while distance <= max_range_m:
            x = pose.x + math.cos(pose.yaw) * distance
            y = pose.y + math.sin(pose.yaw) * distance
            cell = (int(round(x / self.resolution_m)), int(round(y / self.resolution_m)))
            if not self.is_free(cell):
                return max(0.0, distance - step_m)
            distance += step_m
        return max_range_m

    def side_clearances_m(self, pose: Pose2D, max_range_m: float = 1.0, step_m: float = 0.05) -> tuple[float, float]:
        left_pose = Pose2D(pose.x, pose.y, pose.yaw + math.pi / 2.0)
        right_pose = Pose2D(pose.x, pose.y, pose.yaw - math.pi / 2.0)
        return self.front_clearance_m(left_pose, max_range_m, step_m), self.front_clearance_m(right_pose, max_range_m, step_m)

    def render_ascii(self, path: Iterable[tuple[int, int]] | None = None, start: tuple[int, int] | None = None, goal: tuple[int, int] | None = None) -> str:
        path_set = set(path or [])
        rows: List[str] = []
        for y in range(self.height):
            chars = []
            for x in range(self.width):
                cell = (x, y)
                if cell == start:
                    chars.append("S")
                elif cell == goal:
                    chars.append("G")
                elif cell in path_set:
                    chars.append("*")
                elif cell in self.blocked:
                    chars.append("#")
                else:
                    chars.append(".")
            rows.append("".join(chars))
        return "\n".join(rows)
