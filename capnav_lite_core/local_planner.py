from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Iterable, List, Sequence

from .occupancy import GridMap, Pose2D
from .priors import CapabilityAwarePrior
from .profile import CapabilityProfile


def _wrap_angle(theta: float) -> float:
    while theta > math.pi:
        theta -= 2.0 * math.pi
    while theta < -math.pi:
        theta += 2.0 * math.pi
    return theta


@dataclass(slots=True)
class VelocityCommand:
    linear_mps: float
    angular_rad_s: float
    score: float
    label: str


class LocalPlanner:
    def __init__(self, profile: CapabilityProfile, prior: CapabilityAwarePrior | None = None):
        self.profile = profile.normalized()
        self.prior = prior or CapabilityAwarePrior()
        self.previous_label = "S"

    def _candidate_actions(self) -> list[tuple[str, float, float]]:
        c = self.profile
        return [
            ("F", c.comfort_linear_speed_mps, 0.0),
            ("SLOW", 0.55 * c.comfort_linear_speed_mps, 0.0),
            ("FL", 0.75 * c.comfort_linear_speed_mps, 0.65 * c.comfort_angular_speed_rad_s),
            ("FR", 0.75 * c.comfort_linear_speed_mps, -0.65 * c.comfort_angular_speed_rad_s),
            ("L", 0.0, c.comfort_angular_speed_rad_s),
            ("R", 0.0, -c.comfort_angular_speed_rad_s),
            ("B", -0.35 * c.comfort_linear_speed_mps, 0.0),
            ("S", 0.0, 0.0),
        ]

    def _lookahead_goal(self, pose: Pose2D, global_path: Sequence[tuple[float, float]], lookahead_idx: int = 4) -> tuple[float, float]:
        if not global_path:
            return pose.x, pose.y
        distances = [(i, (pose.x - px) ** 2 + (pose.y - py) ** 2) for i, (px, py) in enumerate(global_path)]
        nearest = min(distances, key=lambda t: t[1])[0]
        return global_path[min(len(global_path) - 1, nearest + lookahead_idx)]

    def _simulate(self, pose: Pose2D, linear: float, angular: float, dt: float = 0.5) -> Pose2D:
        yaw = pose.yaw + angular * dt
        x = pose.x + linear * math.cos(yaw) * dt
        y = pose.y + linear * math.sin(yaw) * dt
        return Pose2D(x, y, yaw)

    def choose_command(self, grid: GridMap, pose: Pose2D, global_path: Sequence[tuple[float, float]], goal_cell: tuple[int, int]) -> VelocityCommand:
        goal_point = self._lookahead_goal(pose, global_path)
        front_clearance = grid.front_clearance_m(pose)
        left_clearance, right_clearance = grid.side_clearances_m(pose)
        best = VelocityCommand(0.0, 0.0, float("-inf"), "S")
        base_cell = grid.pose_to_cell(pose)
        direction = int(round(((pose.yaw % (2 * math.pi)) / (math.pi / 2)))) % 4
        state_key = self.prior.state_key(grid, base_cell, direction, goal_cell, self.profile, self.previous_label == "B")
        prior_scores = self.prior.get(state_key)
        for label, linear, angular in self._candidate_actions():
            candidate = self._simulate(pose, linear, angular)
            candidate_cell = grid.pose_to_cell(candidate)
            if not grid.is_free(candidate_cell):
                continue
            goal_heading = math.atan2(goal_point[1] - pose.y, goal_point[0] - pose.x)
            new_goal_heading = math.atan2(goal_point[1] - candidate.y, goal_point[0] - candidate.x)
            heading_error = abs(_wrap_angle(goal_heading - pose.yaw))
            new_heading_error = abs(_wrap_angle(new_goal_heading - candidate.yaw))
            progress = math.dist((pose.x, pose.y), goal_point) - math.dist((candidate.x, candidate.y), goal_point)
            clearance = grid.front_clearance_m(candidate)
            score = 0.0
            score += 4.2 * progress
            score += 1.3 * (heading_error - new_heading_error)
            score += 0.7 * min(clearance, 1.2)
            score += 0.4 * min(left_clearance + right_clearance, 1.5)
            score += 0.6 * prior_scores.get({"SLOW": "F"}.get(label, label), 0.0)
            if label in {"FL", "FR", "L", "R"}:
                score -= 0.9 * self.profile.turning_cost * abs(angular) / max(self.profile.max_angular_speed_rad_s, 1e-6)
            if label == "B":
                score -= 1.1 * self.profile.backtracking_cost
            if front_clearance < self.profile.clearance_margin_m + 0.1 and linear > 0:
                score -= 5.0
            if label == self.previous_label:
                score += 0.2
            if score > best.score:
                best = VelocityCommand(linear, angular, score, label)
        self.previous_label = best.label
        return best
