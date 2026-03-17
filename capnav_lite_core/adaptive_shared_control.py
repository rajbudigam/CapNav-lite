from __future__ import annotations

from dataclasses import asdict, dataclass
import json
import math
from pathlib import Path
import random
from typing import Sequence

from .occupancy import GridMap, Pose2D
from .profile import CapabilityProfile


@dataclass(slots=True)
class DynamicObstacle:
    x: float
    y: float
    vx: float
    vy: float
    radius: float = 0.22
    bounds: tuple[float, float, float, float] | None = None

    def step(self, dt: float) -> None:
        self.x += self.vx * dt
        self.y += self.vy * dt
        if self.bounds is None:
            return
        xmin, xmax, ymin, ymax = self.bounds
        if self.x < xmin or self.x > xmax:
            self.vx *= -1.0
            self.x = min(max(self.x, xmin), xmax)
        if self.y < ymin or self.y > ymax:
            self.vy *= -1.0
            self.y = min(max(self.y, ymin), ymax)

    def predict(self, t: float) -> tuple[float, float]:
        return self.x + self.vx * t, self.y + self.vy * t


@dataclass(slots=True)
class DynamicScenario:
    grid: GridMap
    start: Pose2D
    goal_cell: tuple[int, int]
    dynamic_obstacles: list[DynamicObstacle]
    name: str


@dataclass(slots=True)
class UserState:
    slip_est: float = 0.08
    oscillation_est: float = 0.0
    recent_interventions: float = 0.0
    reliability: float = 0.7
    last_angular: float = 0.0
    stall_est: float = 0.0
    progress_est: float = 0.0
    intervention_bias: float = 0.0
    last_exec_linear: float = 0.0
    last_exec_angular: float = 0.0
    yield_steps: int = 0


@dataclass(slots=True)
class SharedControlParams:
    horizon: int = 2
    risky_horizon: int = 4
    samples: int = 20
    risk_penalty: float = 0.0
    margin: float = 0.0
    agreement: float = 0.0
    safety_buffer_adjust_m: float = 0.00
    speed_cap_mps: float = 0.26
    caution_speed_mps: float = 0.08
    emergency_ttc_s: float = 0.70
    caution_ttc_s: float = 1.10
    wait_bonus: float = 1.30
    user_agreement_blend: float = 0.35
    smooth_linear_gain: float = 0.12
    smooth_angular_gain: float = 0.18
    path_progress_gain: float = 5.8
    goal_progress_gain: float = 4.6
    heading_gain: float = 0.7
    clearance_gain: float = 1.6
    safety_penalty: float = 16.0
    ttc_penalty: float = 7.5
    turn_penalty: float = 0.22
    backtracking_penalty: float = 1.25
    user_agreement_gain: float = 0.35
    yield_hold_steps: int = 2
    use_lattice_scoring: bool = True
    adaptive_threshold: bool = True
    use_reliability_estimation: bool = True
    use_blind_corner_slowdown: bool = True
    use_dynamic_prediction: bool = True


@dataclass(slots=True)
class StepDecision:
    linear: float
    angular: float
    safety_intervened: bool = False
    comfort_intervened: bool = False
    override_intervened: bool = False
    label: str = "USER"


@dataclass(slots=True)
class EpisodeResult:
    success: bool
    collision: bool
    steps: int
    path_efficiency: float
    distance_m: float
    intervention_rate: float
    min_clearance_m: float
    safety_intervention_rate: float
    comfort_intervention_rate: float
    override_rate: float


@dataclass(slots=True)
class AggregateResult:
    n: int
    success_rate: float
    collision_rate: float
    path_efficiency: float
    intervention_rate: float
    min_clearance_m: float
    mean_steps: float
    safety_intervention_rate: float
    comfort_intervention_rate: float
    override_rate: float


@dataclass(slots=True)
class BenchmarkReport:
    baseline: AggregateResult
    improved: AggregateResult
    selected_params: SharedControlParams
    train_utility: float
    train_candidates: list[dict]
    train_scenarios: list[int]
    test_scenarios: list[int]
    user_seeds: list[int]


def _wrap(theta: float) -> float:
    while theta > math.pi:
        theta -= 2.0 * math.pi
    while theta < -math.pi:
        theta += 2.0 * math.pi
    return theta


def _cell_to_xy(grid: GridMap, cell: tuple[int, int]) -> tuple[float, float]:
    return cell[0] * grid.resolution_m, cell[1] * grid.resolution_m


def generate_clinic_scenario(seed: int, width: int = 28, height: int = 18, resolution_m: float = 0.25) -> DynamicScenario:
    rng = random.Random(seed)
    blocked: set[tuple[int, int]] = set()
    for x in range(width):
        blocked.add((x, 0))
        blocked.add((x, height - 1))
    for y in range(height):
        blocked.add((0, y))
        blocked.add((width - 1, y))
    for wall_x in (8, 18):
        for y in range(1, height - 1):
            blocked.add((wall_x, y))
        for gap_y in (3, 4, 8, 9, 13, 14):
            blocked.discard((wall_x, gap_y))
    furniture_columns = list(range(2, 7)) + list(range(10, 17)) + list(range(20, width - 2))
    start = (2, 2)
    goal = (width - 3, height - 3)
    for _ in range(10):
        cell = (rng.choice(furniture_columns), rng.randint(2, height - 3))
        if cell in {start, goal}:
            continue
        blocked.add(cell)
        if rng.random() < 0.2 and cell[1] + 1 < height - 1:
            blocked.add((cell[0], cell[1] + 1))
    grid = GridMap(width, height, resolution_m, blocked)
    if not grid.a_star(start, goal):
        return generate_clinic_scenario(seed + 1000, width, height, resolution_m)
    dynamic_obstacles = [
        DynamicObstacle(9 * resolution_m, 8.5 * resolution_m, 0.35, 0.0, 0.23, (9 * resolution_m, 17 * resolution_m, 8.5 * resolution_m, 8.5 * resolution_m)),
        DynamicObstacle(22 * resolution_m, 4 * resolution_m, 0.0, 0.25, 0.22, (22 * resolution_m, 22 * resolution_m, 3 * resolution_m, 13 * resolution_m)),
    ]
    if rng.random() < 0.5:
        dynamic_obstacles.append(DynamicObstacle(5 * resolution_m, 12 * resolution_m, 0.0, -0.2, 0.20, (5 * resolution_m, 5 * resolution_m, 5 * resolution_m, 13 * resolution_m)))
    return DynamicScenario(
        grid=grid,
        start=grid.cell_to_pose(start, yaw=0.0),
        goal_cell=goal,
        dynamic_obstacles=dynamic_obstacles,
        name=f"clinic_{seed}",
    )


def _collision_static(grid: GridMap, x: float, y: float) -> bool:
    cell = (int(round(x / grid.resolution_m)), int(round(y / grid.resolution_m)))
    return not grid.is_free(cell)


def _collision_dynamic(x: float, y: float, obstacles: Sequence[DynamicObstacle], radius: float = 0.22, t: float = 0.0) -> bool:
    for obstacle in obstacles:
        ox, oy = obstacle.predict(t)
        if math.dist((x, y), (ox, oy)) <= radius + obstacle.radius:
            return True
    return False


def _min_dynamic_clearance(x: float, y: float, obstacles: Sequence[DynamicObstacle], t: float = 0.0) -> float:
    best = float("inf")
    for obstacle in obstacles:
        ox, oy = obstacle.predict(t)
        best = min(best, math.dist((x, y), (ox, oy)) - obstacle.radius)
    return best if obstacles else 10.0


def _front_clearance_with_dynamic(
    grid: GridMap,
    pose: Pose2D,
    obstacles: Sequence[DynamicObstacle],
    *,
    dynamic_t: float = 0.0,
    max_range_m: float = 2.0,
    step_m: float = 0.05,
) -> float:
    distance = 0.0
    while distance <= max_range_m:
        x = pose.x + math.cos(pose.yaw) * distance
        y = pose.y + math.sin(pose.yaw) * distance
        if _collision_static(grid, x, y) or _collision_dynamic(x, y, obstacles, t=dynamic_t):
            return max(0.0, distance - step_m)
        distance += step_m
    return max_range_m


def _nearest_dynamic(pose: Pose2D, obstacles: Sequence[DynamicObstacle], *, horizon: float = 1.2, dt: float = 0.2) -> tuple[float, DynamicObstacle | None, float]:
    best_distance = float("inf")
    best_obstacle: DynamicObstacle | None = None
    best_t = 0.0
    steps = max(1, int(horizon / dt))
    for obstacle in obstacles:
        for index in range(steps + 1):
            t = index * dt
            ox, oy = obstacle.predict(t)
            distance = math.dist((pose.x, pose.y), (ox, oy)) - obstacle.radius
            if distance < best_distance:
                best_distance = distance
                best_obstacle = obstacle
                best_t = t
    return best_distance, best_obstacle, best_t


def _predicted_stop_distance(linear_mps: float, reaction_time_s: float, brake_mps2: float = 0.9) -> float:
    v = max(0.0, linear_mps)
    return v * reaction_time_s + (v * v) / (2.0 * brake_mps2 + 1e-6)


def _step_robot(pose: Pose2D, linear_mps: float, angular_rad_s: float, dt: float = 0.3) -> Pose2D:
    yaw = _wrap(pose.yaw + angular_rad_s * dt)
    x = pose.x + linear_mps * math.cos(yaw) * dt
    y = pose.y + linear_mps * math.sin(yaw) * dt
    return Pose2D(x, y, yaw)


def _nearest_path_index(path_points: Sequence[tuple[float, float]], pose: Pose2D) -> int:
    return min(range(len(path_points)), key=lambda idx: (pose.x - path_points[idx][0]) ** 2 + (pose.y - path_points[idx][1]) ** 2)


def _lookahead_point(path_points: Sequence[tuple[float, float]], pose: Pose2D, lookahead_steps: int = 4) -> tuple[float, float]:
    idx = _nearest_path_index(path_points, pose)
    return path_points[min(len(path_points) - 1, idx + lookahead_steps)]


def _path_points(scenario: DynamicScenario) -> list[tuple[float, float]]:
    cells = scenario.grid.a_star(scenario.grid.pose_to_cell(scenario.start), scenario.goal_cell)
    return [_cell_to_xy(scenario.grid, cell) for cell in cells]


def _default_profile(user_seed: int) -> CapabilityProfile:
    return CapabilityProfile(
        user_id=f"dynamic-user-{user_seed}",
        max_linear_speed_mps=0.42,
        max_angular_speed_rad_s=0.90,
        comfort_linear_speed_mps=0.25,
        comfort_angular_speed_rad_s=0.55,
        turning_cost=1.4,
        backtracking_cost=1.1,
        slip_probability=0.14 + ((user_seed * 13) % 5) * 0.01,
        reaction_time_s=0.42,
        clearance_margin_m=0.32,
    )


_COMMAND_LIBRARY: list[tuple[str, float, float]] = [
    ("F", 0.25, 0.0),
    ("SLOW", 0.14, 0.0),
    ("WAIT", 0.0, 0.0),
    ("FL", 0.18, 0.40),
    ("FR", 0.18, -0.40),
    ("L", 0.0, 0.60),
    ("R", 0.0, -0.60),
    ("B", -0.10, 0.0),
    ("S", 0.0, 0.0),
]


def _action_library(profile: CapabilityProfile) -> list[tuple[str, float, float]]:
    c = profile
    return [
        ("F", c.comfort_linear_speed_mps, 0.0),
        ("SLOW", 0.55 * c.comfort_linear_speed_mps, 0.0),
        ("WAIT", 0.0, 0.0),
        ("FL", 0.75 * c.comfort_linear_speed_mps, 0.70 * c.comfort_angular_speed_rad_s),
        ("FR", 0.75 * c.comfort_linear_speed_mps, -0.70 * c.comfort_angular_speed_rad_s),
        ("SFL", 0.55 * c.comfort_linear_speed_mps, 1.00 * c.comfort_angular_speed_rad_s),
        ("SFR", 0.55 * c.comfort_linear_speed_mps, -1.00 * c.comfort_angular_speed_rad_s),
        ("L", 0.0, c.comfort_angular_speed_rad_s),
        ("R", 0.0, -c.comfort_angular_speed_rad_s),
        ("B", -0.35 * c.comfort_linear_speed_mps, 0.0),
        ("S", 0.0, 0.0),
    ]


def user_command(profile: CapabilityProfile, pose: Pose2D, path_points: Sequence[tuple[float, float]], rng: random.Random) -> tuple[str, float, float]:
    gx, gy = _lookahead_point(path_points, pose, 4)
    heading_error = _wrap(math.atan2(gy - pose.y, gx - pose.x) - pose.yaw)
    linear = profile.comfort_linear_speed_mps * max(0.0, math.cos(heading_error))
    angular = max(-profile.comfort_angular_speed_rad_s, min(profile.comfort_angular_speed_rad_s, 1.2 * heading_error))
    if rng.random() < profile.slip_probability:
        if rng.random() < 0.55:
            angular += rng.uniform(-0.9, 0.9)
        else:
            linear *= rng.uniform(-0.4, 0.3)
    linear += rng.gauss(0.0, 0.03)
    angular += rng.gauss(0.0, 0.08)
    linear = max(-profile.max_linear_speed_mps, min(profile.max_linear_speed_mps, linear))
    angular = max(-profile.max_angular_speed_rad_s, min(profile.max_angular_speed_rad_s, angular))
    label = min(_COMMAND_LIBRARY, key=lambda command: (command[1] - linear) ** 2 + (command[2] - angular) ** 2)[0]
    return label, linear, angular


def _update_user_state(
    state: UserState,
    user_command_tuple: tuple[str, float, float],
    executed_command: tuple[float, float],
    pose_before: Pose2D,
    pose_after: Pose2D,
    intervened: bool,
    *,
    use_reliability_estimation: bool = True,
) -> UserState:
    _, user_linear, user_angular = user_command_tuple
    executed_linear, executed_angular = executed_command
    travel = math.dist((pose_before.x, pose_before.y), (pose_after.x, pose_after.y))
    forward_fail = 1.0 if user_linear > 0.10 and travel < 0.02 else 0.0
    oscillation = min(1.0, abs(user_angular - state.last_angular) / 1.2)
    command_deviation = min(1.0, abs(executed_linear - user_linear) / 0.45 + abs(executed_angular - user_angular) / 1.2)
    state.last_angular = user_angular
    state.slip_est = 0.88 * state.slip_est + 0.12 * max(forward_fail, command_deviation)
    state.oscillation_est = 0.84 * state.oscillation_est + 0.16 * oscillation
    state.recent_interventions = 0.88 * state.recent_interventions + 0.12 * (1.0 if intervened else 0.0)
    state.stall_est = 0.86 * state.stall_est + 0.14 * (1.0 if travel < 0.03 else 0.0)
    state.progress_est = 0.85 * state.progress_est + 0.15 * min(1.0, travel / 0.08)
    state.intervention_bias = 0.9 * state.intervention_bias + 0.1 * (command_deviation if intervened else 0.0)
    if use_reliability_estimation:
        state.reliability = max(0.08, min(0.98, 1.0 - 0.55 * state.slip_est - 0.25 * state.oscillation_est - 0.22 * state.recent_interventions - 0.18 * state.stall_est + 0.15 * state.progress_est))
    else:
        state.reliability = 0.5
    return state


def _time_to_dynamic_collision(pose: Pose2D, linear: float, angular: float, obstacles: Sequence[DynamicObstacle], *, robot_radius: float = 0.22, horizon: float = 2.0, dt: float = 0.1) -> tuple[float, DynamicObstacle | None]:
    sim_pose = Pose2D(pose.x, pose.y, pose.yaw)
    best_t = float("inf")
    best_obstacle: DynamicObstacle | None = None
    steps = max(1, int(horizon / dt))
    for step_index in range(1, steps + 1):
        sim_pose = _step_robot(sim_pose, linear, angular, dt)
        t = step_index * dt
        for obstacle in obstacles:
            ox, oy = obstacle.predict(t)
            if math.dist((sim_pose.x, sim_pose.y), (ox, oy)) <= robot_radius + obstacle.radius + 0.02:
                if t < best_t:
                    best_t = t
                    best_obstacle = obstacle
    return best_t, best_obstacle


def _blind_corner_risk(grid: GridMap, pose: Pose2D) -> float:
    front = grid.front_clearance_m(pose, max_range_m=1.2)
    left, right = grid.side_clearances_m(pose, max_range_m=1.0)
    if front > 1.0:
        return 0.0
    side_gap = max(left, right)
    tight_side = min(left, right)
    if side_gap > 0.75 and tight_side < 0.30:
        return max(0.0, 1.0 - front / 1.0)
    return 0.0


def _escape_bias(pose: Pose2D, obstacles: Sequence[DynamicObstacle]) -> float:
    _, nearest, _ = _nearest_dynamic(pose, obstacles, horizon=0.8, dt=0.2)
    if nearest is None:
        return 1.0
    ox, oy = nearest.predict(0.2)
    relative_heading = _wrap(math.atan2(oy - pose.y, ox - pose.x) - pose.yaw)
    return -1.0 if relative_heading > 0.0 else 1.0


def _alignment_score(profile: CapabilityProfile, linear: float, angular: float, user_linear: float, user_angular: float) -> float:
    return 1.0 - min(1.0, abs(linear - user_linear) / max(profile.max_linear_speed_mps, 1e-6) + abs(angular - user_angular) / max(profile.max_angular_speed_rad_s, 1e-6))


def _robot_frame(pose: Pose2D, x: float, y: float) -> tuple[float, float]:
    dx = x - pose.x
    dy = y - pose.y
    c = math.cos(pose.yaw)
    s = math.sin(pose.yaw)
    return dx * c + dy * s, -dx * s + dy * c


def _frontal_dynamic_conflict(
    pose: Pose2D,
    obstacles: Sequence[DynamicObstacle],
    *,
    horizon: float = 1.6,
    dt: float = 0.1,
) -> tuple[float, DynamicObstacle | None, float, float, bool]:
    best_t = float("inf")
    best_obstacle: DynamicObstacle | None = None
    best_forward = float("inf")
    best_lateral = 0.0
    is_cross = False
    c = math.cos(pose.yaw)
    s = math.sin(pose.yaw)
    steps = max(1, int(horizon / dt))
    for obstacle in obstacles:
        rel_v_forward = obstacle.vx * c + obstacle.vy * s
        rel_v_lateral = -obstacle.vx * s + obstacle.vy * c
        for step_idx in range(steps + 1):
            t = step_idx * dt
            ox, oy = obstacle.predict(t)
            forward, lateral = _robot_frame(pose, ox, oy)
            if forward < -0.25 or forward > 1.6:
                continue
            lateral_band = obstacle.radius + 0.34
            if abs(lateral) <= lateral_band:
                if t < best_t or (abs(t - best_t) < 1e-6 and forward < best_forward):
                    best_t = t
                    best_obstacle = obstacle
                    best_forward = forward
                    best_lateral = lateral
                    is_cross = abs(rel_v_lateral) > abs(rel_v_forward) + 0.03
    return best_t, best_obstacle, best_forward, best_lateral, is_cross


def _trajectory_safe(
    grid: GridMap,
    pose: Pose2D,
    linear: float,
    angular: float,
    obstacles: Sequence[DynamicObstacle],
    *,
    safety_buffer: float,
    horizon: float = 1.2,
    dt: float = 0.1,
) -> tuple[bool, float]:
    sim_pose = Pose2D(pose.x, pose.y, pose.yaw)
    min_clearance = float("inf")
    steps = max(1, int(horizon / dt))
    for step_idx in range(1, steps + 1):
        sim_pose = _step_robot(sim_pose, linear, angular, dt)
        t = step_idx * dt
        if _collision_static(grid, sim_pose.x, sim_pose.y) or _collision_dynamic(sim_pose.x, sim_pose.y, obstacles, t=t):
            return False, 0.0
        clearance = min(
            _front_clearance_with_dynamic(grid, sim_pose, obstacles, dynamic_t=t, max_range_m=1.4),
            _min_dynamic_clearance(sim_pose.x, sim_pose.y, obstacles, t=t),
        )
        min_clearance = min(min_clearance, clearance)
        if clearance < max(0.04, safety_buffer - 0.03):
            return False, max(0.0, min_clearance)
    return True, max(0.0, min_clearance if math.isfinite(min_clearance) else 0.0)


def _pure_autonomy_controller(profile: CapabilityProfile, pose: Pose2D, user_command_tuple: tuple[str, float, float], path_points: Sequence[tuple[float, float]], scenario: DynamicScenario, params: SharedControlParams) -> StepDecision:
    gx, gy = _lookahead_point(path_points, pose, 5)
    heading_error = _wrap(math.atan2(gy - pose.y, gx - pose.x) - pose.yaw)
    linear = profile.comfort_linear_speed_mps * max(0.0, math.cos(heading_error))
    angular = max(-profile.comfort_angular_speed_rad_s, min(profile.comfort_angular_speed_rad_s, 1.2 * heading_error))
    front = _front_clearance_with_dynamic(scenario.grid, pose, scenario.dynamic_obstacles)
    ttc, _ = _time_to_dynamic_collision(pose, max(0.0, linear), angular, scenario.dynamic_obstacles, horizon=1.8)
    safety = False
    if front < profile.clearance_margin_m + 0.10 or ttc < params.emergency_ttc_s:
        linear = 0.0
        safety = True
    elif front < profile.clearance_margin_m + 0.25 or ttc < params.caution_ttc_s:
        linear = min(linear, params.caution_speed_mps)
        safety = True
    return StepDecision(linear=linear, angular=angular, safety_intervened=safety, override_intervened=True, label="AUTO")


def _fixed_blend_controller(profile: CapabilityProfile, pose: Pose2D, user_command_tuple: tuple[str, float, float], path_points: Sequence[tuple[float, float]], scenario: DynamicScenario, params: SharedControlParams) -> StepDecision:
    _, user_linear, user_angular = user_command_tuple
    gx, gy = _lookahead_point(path_points, pose, 5)
    heading_error = _wrap(math.atan2(gy - pose.y, gx - pose.x) - pose.yaw)
    auto_linear = profile.comfort_linear_speed_mps * max(0.0, math.cos(heading_error))
    auto_angular = max(-profile.comfort_angular_speed_rad_s, min(profile.comfort_angular_speed_rad_s, heading_error))
    front_clearance = _front_clearance_with_dynamic(scenario.grid, pose, scenario.dynamic_obstacles)
    risk = max(0.0, 1.0 - front_clearance / max(profile.clearance_margin_m + 0.9, 1e-6))
    autonomy_weight = min(0.92, max(0.25, 0.25 + 0.55 * risk))
    linear = autonomy_weight * auto_linear + (1.0 - autonomy_weight) * user_linear
    angular = autonomy_weight * auto_angular + (1.0 - autonomy_weight) * user_angular
    safety = False
    stop_distance = _predicted_stop_distance(abs(linear), profile.reaction_time_s)
    ttc, _ = _time_to_dynamic_collision(pose, max(0.0, linear), angular, scenario.dynamic_obstacles, horizon=1.8)
    if front_clearance < profile.clearance_margin_m or front_clearance < stop_distance + profile.clearance_margin_m * 0.6 or ttc < params.emergency_ttc_s:
        linear = min(linear, 0.0)
        if abs(heading_error) > 0.2:
            angular = auto_angular
        safety = True
    elif front_clearance < profile.clearance_margin_m + 0.25 or ttc < params.caution_ttc_s:
        linear = min(linear, params.caution_speed_mps)
        safety = True
    diff = _alignment_score(profile, linear, angular, user_linear, user_angular)
    override = autonomy_weight > 0.65 or diff < 0.45
    comfort = not safety and not override and diff < 0.90
    return StepDecision(linear=linear, angular=angular, safety_intervened=safety, comfort_intervened=comfort, override_intervened=override, label="BLEND")


def _user_plus_safety_controller(profile: CapabilityProfile, pose: Pose2D, user_command_tuple: tuple[str, float, float], path_points: Sequence[tuple[float, float]], scenario: DynamicScenario, params: SharedControlParams) -> StepDecision:
    _, linear, angular = user_command_tuple
    front = _front_clearance_with_dynamic(scenario.grid, pose, scenario.dynamic_obstacles)
    ttc, _ = _time_to_dynamic_collision(pose, max(0.0, linear), angular, scenario.dynamic_obstacles, horizon=1.8)
    safety = False
    if front < profile.clearance_margin_m or ttc < params.emergency_ttc_s:
        linear = 0.0
        safety = True
    elif front < profile.clearance_margin_m + 0.12 or ttc < params.caution_ttc_s:
        linear = min(linear, params.caution_speed_mps)
        safety = True
    return StepDecision(linear=linear, angular=angular, safety_intervened=safety, label="USER")


def _improved_controller(profile: CapabilityProfile, state: UserState, pose: Pose2D, user_command_tuple: tuple[str, float, float], path_points: Sequence[tuple[float, float]], scenario: DynamicScenario, params: SharedControlParams, rng: random.Random) -> StepDecision:
    user_label, user_linear, user_angular = user_command_tuple
    command_library = _action_library(profile)
    idx0 = _nearest_path_index(path_points, pose)
    goal_xy = _cell_to_xy(scenario.grid, scenario.goal_cell)
    left_clearance, right_clearance = scenario.grid.side_clearances_m(pose, max_range_m=0.9)
    front_clearance = _front_clearance_with_dynamic(scenario.grid, pose, scenario.dynamic_obstacles, max_range_m=1.8)
    dynamic_distance, _, _ = _nearest_dynamic(pose, scenario.dynamic_obstacles, horizon=1.2, dt=0.2)
    conflict_t, conflict_obstacle, conflict_forward, conflict_lateral, conflict_is_cross = _frontal_dynamic_conflict(pose, scenario.dynamic_obstacles, horizon=1.8, dt=0.1)
    safety_buffer = profile.clearance_margin_m + params.safety_buffer_adjust_m + 0.08 * (1.0 - state.reliability)
    user_ttc = float("inf")
    if params.use_dynamic_prediction:
        user_ttc, _ = _time_to_dynamic_collision(pose, max(0.0, user_linear), user_angular, scenario.dynamic_obstacles, horizon=1.8)

    sidepass_sign = 1.0 if left_clearance >= right_clearance else -1.0
    if conflict_obstacle is not None and abs(conflict_lateral) > 0.14:
        sidepass_sign = -1.0 if conflict_lateral > 0.0 else 1.0
    same_lane_conflict = conflict_obstacle is not None and (not conflict_is_cross) and 0.0 <= conflict_forward <= 1.35 and abs(conflict_lateral) <= 0.42
    cross_conflict = conflict_obstacle is not None and conflict_is_cross and conflict_t <= params.caution_ttc_s + 0.2 and conflict_forward <= 1.05
    stacked_crossing = False
    if same_lane_conflict and len(scenario.dynamic_obstacles) >= 2:
        for obstacle in scenario.dynamic_obstacles:
            if obstacle is conflict_obstacle:
                continue
            forward_o, lateral_o = _robot_frame(pose, *obstacle.predict(0.8))
            if 0.8 <= forward_o <= 2.5 and abs(lateral_o) <= 0.95:
                stacked_crossing = True
                break

    if stacked_crossing and front_clearance < 0.7 and left_clearance >= 0.5 and right_clearance >= 0.5:
        state.yield_steps = max(state.yield_steps, 999)
        return StepDecision(linear=0.0, angular=0.0, safety_intervened=True, override_intervened=True, label="STACKED_WAIT")

    if front_clearance < safety_buffer or user_ttc < params.emergency_ttc_s:
        if same_lane_conflict and conflict_forward < 0.95 and abs(conflict_lateral) < 0.28:
            state.yield_steps = max(state.yield_steps, params.yield_hold_steps + 1)
            return StepDecision(linear=0.0, angular=0.0, safety_intervened=True, override_intervened=True, label="BLOCKED_WAIT")
        sign = 1.0 if left_clearance > right_clearance else -1.0
        state.yield_steps = max(state.yield_steps, params.yield_hold_steps)
        return StepDecision(linear=0.0, angular=sign * 0.55 * profile.comfort_angular_speed_rad_s, safety_intervened=True, override_intervened=True, label="EMERGENCY")
    if cross_conflict and front_clearance < safety_buffer + 0.55:
        state.yield_steps = max(state.yield_steps, params.yield_hold_steps + 1)
        return StepDecision(linear=0.0, angular=0.0, safety_intervened=True, override_intervened=True, label="YIELD")
    if state.yield_steps > 0:
        state.yield_steps -= 1
        return StepDecision(linear=0.0, angular=0.0, safety_intervened=True, label="WAIT")
    if not params.use_lattice_scoring:
        return _user_plus_safety_controller(profile, pose, user_command_tuple, path_points, scenario, params)

    risky = front_clearance < (safety_buffer + 0.18) or dynamic_distance < 0.90 or user_ttc < params.caution_ttc_s or same_lane_conflict or cross_conflict
    base_decision = _fixed_blend_controller(profile, pose, user_command_tuple, path_points, scenario, params)
    auto_decision = _pure_autonomy_controller(profile, pose, user_command_tuple, path_points, scenario, params)
    if (not risky) and state.reliability > 0.35 and state.stall_est < 0.55:
        return base_decision

    horizon = params.risky_horizon if risky else params.horizon
    if same_lane_conflict:
        horizon = max(horizon, params.risky_horizon + 4)

    def path_follow_command(sim_pose: Pose2D, step_t: float) -> tuple[str, float, float]:
        idx = _nearest_path_index(path_points, sim_pose)
        gx, gy = path_points[min(len(path_points) - 1, idx + 4)]
        target_x, target_y = gx, gy
        local_conflict_t, local_obstacle, local_forward, local_lateral, local_cross = _frontal_dynamic_conflict(sim_pose, scenario.dynamic_obstacles, horizon=1.4, dt=0.1)
        if local_obstacle is not None:
            ox, oy = local_obstacle.predict(min(1.2, step_t + 0.4))
            path_heading = math.atan2(gy - sim_pose.y, gx - sim_pose.x)
            if local_cross and local_conflict_t < params.caution_ttc_s and local_forward < 0.95:
                return ("WAIT", 0.0, 0.0)
            if (not local_cross) and 0.0 <= local_forward <= 1.3 and abs(local_lateral) <= 0.55:
                offset = sidepass_sign * (local_obstacle.radius + profile.clearance_margin_m + 0.16)
                target_x = ox + math.cos(path_heading) * 0.42 - math.sin(path_heading) * offset
                target_y = oy + math.sin(path_heading) * 0.42 + math.cos(path_heading) * offset
        heading_error = _wrap(math.atan2(target_y - sim_pose.y, target_x - sim_pose.x) - sim_pose.yaw)
        desired_linear = profile.comfort_linear_speed_mps * max(0.0, math.cos(heading_error))
        desired_angular = max(-profile.comfort_angular_speed_rad_s, min(profile.comfort_angular_speed_rad_s, 1.20 * heading_error))
        if same_lane_conflict and abs(heading_error) > 0.2:
            desired_linear = min(desired_linear, 0.78 * profile.comfort_linear_speed_mps)
        if params.use_blind_corner_slowdown:
            blind_risk = _blind_corner_risk(scenario.grid, sim_pose)
            if blind_risk > 0.0:
                desired_linear *= max(0.18, 1.0 - 0.70 * blind_risk)
        if params.use_dynamic_prediction:
            ttc, _ = _time_to_dynamic_collision(sim_pose, max(0.0, desired_linear), desired_angular, scenario.dynamic_obstacles, horizon=1.6)
            nearest = _min_dynamic_clearance(sim_pose.x, sim_pose.y, scenario.dynamic_obstacles, t=step_t)
            if nearest < 0.85 and ttc < params.caution_ttc_s and abs(heading_error) < 0.55:
                return ("WAIT", 0.0, 0.0)
        if state.stall_est > 0.40 and abs(heading_error) < 0.25:
            desired_angular += _escape_bias(sim_pose, scenario.dynamic_obstacles) * 0.28 * profile.comfort_angular_speed_rad_s
        return min(command_library, key=lambda item: (item[1] - desired_linear) ** 2 + (item[2] - desired_angular) ** 2)

    def rollout_score(label: str, linear: float, angular: float) -> tuple[float, float, float]:
        sim_pose = Pose2D(pose.x, pose.y, pose.yaw)
        min_clearance = float("inf")
        min_ttc = float("inf")
        feasible = True
        idx_prev = idx0
        for step_idx in range(1, horizon + 1):
            if step_idx == 1:
                cmd_linear, cmd_angular = linear, angular
            else:
                _, cmd_linear, cmd_angular = path_follow_command(sim_pose, 0.3 * (step_idx - 1))
            sim_pose = _step_robot(sim_pose, cmd_linear, cmd_angular, 0.3)
            t = 0.3 * step_idx
            if _collision_static(scenario.grid, sim_pose.x, sim_pose.y) or _collision_dynamic(sim_pose.x, sim_pose.y, scenario.dynamic_obstacles, t=t):
                feasible = False
                break
            clearance = min(_front_clearance_with_dynamic(scenario.grid, sim_pose, scenario.dynamic_obstacles, dynamic_t=t, max_range_m=1.5), _min_dynamic_clearance(sim_pose.x, sim_pose.y, scenario.dynamic_obstacles, t=t))
            min_clearance = min(min_clearance, clearance)
            if clearance < max(0.05, safety_buffer - 0.02):
                feasible = False
                break
            if params.use_dynamic_prediction:
                ttc_step, _ = _time_to_dynamic_collision(sim_pose, max(0.0, cmd_linear), cmd_angular, scenario.dynamic_obstacles, horizon=1.2)
                min_ttc = min(min_ttc, ttc_step)
                if ttc_step < params.emergency_ttc_s - 0.02:
                    feasible = False
                    break
            idx_next = _nearest_path_index(path_points, sim_pose)
            idx_prev = max(idx_prev, idx_next)
        if not feasible:
            return -1e9, 0.0, 0.0
        idx_end = _nearest_path_index(path_points, sim_pose)
        next_idx = min(len(path_points) - 1, idx_end + 1)
        heading_end = _wrap(math.atan2(path_points[next_idx][1] - sim_pose.y, path_points[next_idx][0] - sim_pose.x) - sim_pose.yaw)
        score = 0.0
        score += params.path_progress_gain * max(0, idx_end - idx0)
        score += params.goal_progress_gain * (math.dist((pose.x, pose.y), goal_xy) - math.dist((sim_pose.x, sim_pose.y), goal_xy))
        score += params.heading_gain * max(0.0, math.cos(heading_end))
        score += params.clearance_gain * min(min_clearance, 0.9)
        score -= params.safety_penalty * max(0.0, safety_buffer - min_clearance)
        if math.isfinite(min_ttc):
            score -= params.ttc_penalty * max(0.0, params.caution_ttc_s - min_ttc)
        if label in {"L", "R", "FL", "FR", "SFL", "SFR", "ESCAPE", "ESCAPE_ROT"}:
            score -= 0.7 * params.turn_penalty
        if label == "B":
            score -= params.backtracking_penalty
        if label == "WAIT":
            if cross_conflict or (same_lane_conflict and conflict_forward < 0.85):
                score += params.wait_bonus
            else:
                score -= 0.45
        if same_lane_conflict:
            if label in {"FL", "SFL"} and sidepass_sign > 0.0:
                score += 1.2
            if label in {"FR", "SFR"} and sidepass_sign < 0.0:
                score += 1.2
            if label in {"F", "SLOW"} and conflict_forward < 1.0:
                score -= 1.4
        score += params.user_agreement_gain * _alignment_score(profile, linear, angular, user_linear, user_angular)
        return score, max(0.0, min_clearance), min_ttc

    best: tuple[float, str, float, float, float, float] | None = None
    candidates = list(command_library) + [("USER", user_linear, user_angular)]
    if state.stall_est > 0.35:
        escape_sign = _escape_bias(pose, scenario.dynamic_obstacles)
        candidates += [("ESCAPE", 0.55 * profile.comfort_linear_speed_mps, escape_sign * profile.comfort_angular_speed_rad_s), ("ESCAPE_ROT", 0.0, escape_sign * profile.comfort_angular_speed_rad_s)]
    for label, linear, angular in candidates:
        score, clearance1, ttc1 = rollout_score(label, linear, angular)
        if best is None or score > best[0]:
            best = (score, label, linear, angular, clearance1, ttc1)
    assert best is not None
    _, label, linear, angular, projected_clearance, projected_ttc = best

    def decision_utility(choice_label: str, linear_c: float, angular_c: float) -> tuple[float, float, float]:
        safe, clear_c = _trajectory_safe(scenario.grid, pose, linear_c, angular_c, scenario.dynamic_obstacles, safety_buffer=safety_buffer, horizon=1.0)
        if not safe:
            return -1e8, 0.0, 0.0
        pose_c = _step_robot(pose, linear_c, angular_c, 0.3)
        ttc_c = float("inf") if not params.use_dynamic_prediction else _time_to_dynamic_collision(pose, max(0.0, linear_c), angular_c, scenario.dynamic_obstacles, horizon=1.4)[0]
        idx_c = _nearest_path_index(path_points, pose_c)
        utility_c = 6.5 * max(0, idx_c - idx0) + 4.2 * (math.dist((pose.x, pose.y), goal_xy) - math.dist((pose_c.x, pose_c.y), goal_xy)) + 1.8 * min(clear_c, 0.9) - 10.5 * max(0.0, safety_buffer - clear_c) + 0.8 * _alignment_score(profile, linear_c, angular_c, user_linear, user_angular)
        if math.isfinite(ttc_c):
            utility_c -= 7.5 * max(0.0, params.caution_ttc_s - ttc_c)
        if cross_conflict and choice_label != "WAIT" and linear_c > 0.01:
            utility_c -= 1.2
        return utility_c, clear_c, ttc_c

    best_choice = None
    for choice_label, choice_linear, choice_angular, choice_decision in [
        (label, linear, angular, None),
        ("BLEND_SAFE", base_decision.linear, base_decision.angular, base_decision),
        ("AUTO_SAFE", auto_decision.linear, auto_decision.angular, auto_decision),
        ("WAIT_SAFE", 0.0, 0.0, StepDecision(linear=0.0, angular=0.0, safety_intervened=True, override_intervened=True, label="WAIT_SAFE")),
    ]:
        util, clear_c, ttc_c = decision_utility(choice_label, choice_linear, choice_angular)
        row = (util, choice_label, choice_linear, choice_angular, clear_c, ttc_c, choice_decision)
        if best_choice is None or util > best_choice[0]:
            best_choice = row
    assert best_choice is not None
    _, label, linear, angular, projected_clearance, projected_ttc, choice_decision = best_choice
    if choice_decision is not None and label in {"BLEND_SAFE", "AUTO_SAFE", "WAIT_SAFE"}:
        return StepDecision(linear=choice_decision.linear, angular=choice_decision.angular, safety_intervened=choice_decision.safety_intervened, comfort_intervened=choice_decision.comfort_intervened, override_intervened=choice_decision.override_intervened, label=label)

    alignment = _alignment_score(profile, linear, angular, user_linear, user_angular)
    reliability_threshold = 0.52 if params.adaptive_threshold else 0.50
    override = label != "USER"
    comfort = False
    if label != "USER" and alignment > 0.78 and projected_clearance > safety_buffer + 0.14 and projected_ttc > params.caution_ttc_s + 0.25 and state.reliability > reliability_threshold and (not same_lane_conflict) and (not cross_conflict):
        alpha = params.user_agreement_blend
        linear = (1.0 - alpha) * user_linear + alpha * linear
        angular = (1.0 - alpha) * user_angular + alpha * angular
        override = False
        comfort = True

    if cross_conflict and linear > params.caution_speed_mps:
        linear = min(linear, params.caution_speed_mps)
    linear = min(linear, params.speed_cap_mps)
    smooth_linear = 0.0 if risky else params.smooth_linear_gain
    smooth_angular = 0.0 if risky else params.smooth_angular_gain
    linear = (1.0 - smooth_linear) * linear + smooth_linear * state.last_exec_linear
    angular = (1.0 - smooth_angular) * angular + smooth_angular * state.last_exec_angular
    linear = max(-profile.max_linear_speed_mps, min(profile.max_linear_speed_mps, linear))
    angular = max(-profile.max_angular_speed_rad_s, min(profile.max_angular_speed_rad_s, angular))

    safe_final, final_clearance = _trajectory_safe(scenario.grid, pose, linear, angular, scenario.dynamic_obstacles, safety_buffer=safety_buffer, horizon=1.0)
    final_ttc = float("inf") if not params.use_dynamic_prediction else _time_to_dynamic_collision(pose, max(0.0, linear), angular, scenario.dynamic_obstacles, horizon=1.4)[0]
    if (not safe_final) or final_ttc < params.emergency_ttc_s:
        state.yield_steps = max(state.yield_steps, params.yield_hold_steps)
        if cross_conflict or same_lane_conflict:
            return StepDecision(linear=0.0, angular=0.0, safety_intervened=True, override_intervened=True, label="WAIT_SHIELD")
        linear = 0.0
        angular = (1.0 if left_clearance > right_clearance else -1.0) * 0.55 * profile.comfort_angular_speed_rad_s
        return StepDecision(linear=linear, angular=angular, safety_intervened=True, override_intervened=True, label="EMERGENCY")
    if final_clearance < safety_buffer + 0.10 or final_ttc < params.caution_ttc_s:
        linear = min(linear, params.caution_speed_mps)
        return StepDecision(linear=linear, angular=angular, safety_intervened=True, override_intervened=override or label != "USER", label=label)

    diff_norm = 1.0 - _alignment_score(profile, linear, angular, user_linear, user_angular)
    if diff_norm > 0.55:
        override = True
        comfort = False
    elif label != "USER" or diff_norm > 0.12:
        comfort = not override
    return StepDecision(linear=linear, angular=angular, comfort_intervened=comfort, override_intervened=override, label=label)

def _dispatch_controller(policy: str, profile: CapabilityProfile, state: UserState, pose: Pose2D, pilot_command: tuple[str, float, float], path_points: Sequence[tuple[float, float]], scenario: DynamicScenario, params: SharedControlParams, rng: random.Random) -> StepDecision:
    if policy == "baseline" or policy == "fixed_blend":
        return _fixed_blend_controller(profile, pose, pilot_command, path_points, scenario, params)
    if policy == "pure_autonomy":
        return _pure_autonomy_controller(profile, pose, pilot_command, path_points, scenario, params)
    if policy == "user_safety":
        return _user_plus_safety_controller(profile, pose, pilot_command, path_points, scenario, params)
    if policy == "improved":
        return _improved_controller(profile, state, pose, pilot_command, path_points, scenario, params, rng)
    raise ValueError(f"unknown policy: {policy}")


def run_episode(policy: str, scenario_seed: int, user_seed: int, *, params: SharedControlParams | None = None, max_steps: int = 160) -> EpisodeResult:
    params = params or SharedControlParams()
    scenario = generate_clinic_scenario(scenario_seed)
    profile = _default_profile(user_seed)
    rng = random.Random(user_seed * 1000 + scenario_seed)
    pose = Pose2D(scenario.start.x, scenario.start.y, scenario.start.yaw)
    path_points = _path_points(scenario)
    user_state = UserState(slip_est=profile.slip_probability)
    optimal_path_length = len(scenario.grid.a_star(scenario.grid.pose_to_cell(scenario.start), scenario.goal_cell)) * scenario.grid.resolution_m
    total_distance = 0.0
    intervention_count = 0
    safety_interventions = 0
    comfort_interventions = 0
    override_interventions = 0
    min_clearance = 10.0
    collision = False
    success = False
    steps_taken = 0

    for step in range(max_steps):
        steps_taken = step + 1
        pilot_command = user_command(profile, pose, path_points, rng)
        decision = _dispatch_controller(policy, profile, user_state, pose, pilot_command, path_points, scenario, params, rng)
        pose_before = pose
        pose = _step_robot(pose, decision.linear, decision.angular, 0.3)
        total_distance += math.dist((pose_before.x, pose_before.y), (pose.x, pose.y))
        for obstacle in scenario.dynamic_obstacles:
            obstacle.step(0.3)
        if _collision_static(scenario.grid, pose.x, pose.y) or _collision_dynamic(pose.x, pose.y, scenario.dynamic_obstacles):
            collision = True
            break
        min_clearance = min(min_clearance, _front_clearance_with_dynamic(scenario.grid, pose, scenario.dynamic_obstacles), _min_dynamic_clearance(pose.x, pose.y, scenario.dynamic_obstacles))
        if decision.safety_intervened or decision.comfort_intervened or decision.override_intervened:
            intervention_count += 1
        if decision.safety_intervened:
            safety_interventions += 1
        if decision.comfort_intervened:
            comfort_interventions += 1
        if decision.override_intervened:
            override_interventions += 1
        user_state.last_exec_linear = decision.linear
        user_state.last_exec_angular = decision.angular
        _update_user_state(user_state, pilot_command, (decision.linear, decision.angular), pose_before, pose, decision.safety_intervened or decision.comfort_intervened or decision.override_intervened, use_reliability_estimation=params.use_reliability_estimation)
        goal_xy = _cell_to_xy(scenario.grid, scenario.goal_cell)
        if math.dist((pose.x, pose.y), goal_xy) < 0.35:
            success = True
            break

    path_efficiency = min(1.0, (optimal_path_length / max(total_distance, 1e-6))) if success else 0.0
    return EpisodeResult(success=success, collision=collision, steps=steps_taken, path_efficiency=path_efficiency, distance_m=total_distance, intervention_rate=intervention_count / max(steps_taken, 1), min_clearance_m=min_clearance if min_clearance < 10.0 else 0.0, safety_intervention_rate=safety_interventions / max(steps_taken, 1), comfort_intervention_rate=comfort_interventions / max(steps_taken, 1), override_rate=override_interventions / max(steps_taken, 1))


def aggregate_results(results: Sequence[EpisodeResult]) -> AggregateResult:
    return AggregateResult(
        n=len(results),
        success_rate=sum(r.success for r in results) / max(len(results), 1),
        collision_rate=sum(r.collision for r in results) / max(len(results), 1),
        path_efficiency=sum(r.path_efficiency for r in results) / max(len(results), 1),
        intervention_rate=sum(r.intervention_rate for r in results) / max(len(results), 1),
        min_clearance_m=sum(r.min_clearance_m for r in results) / max(len(results), 1),
        mean_steps=sum(r.steps for r in results) / max(len(results), 1),
        safety_intervention_rate=sum(r.safety_intervention_rate for r in results) / max(len(results), 1),
        comfort_intervention_rate=sum(r.comfort_intervention_rate for r in results) / max(len(results), 1),
        override_rate=sum(r.override_rate for r in results) / max(len(results), 1),
    )


def evaluate_policy(policy: str, scenario_seeds: Sequence[int], user_seeds: Sequence[int], *, params: SharedControlParams | None = None, max_steps: int = 160) -> tuple[AggregateResult, list[EpisodeResult]]:
    results = [run_episode(policy, s, u, params=params, max_steps=max_steps) for s in scenario_seeds for u in user_seeds]
    return aggregate_results(results), results


def _utility(aggregate: AggregateResult) -> float:
    return 7.0 * aggregate.success_rate - 8.0 * aggregate.collision_rate + 2.0 * aggregate.path_efficiency - 0.10 * aggregate.safety_intervention_rate - 0.05 * aggregate.override_rate + 0.4 * aggregate.min_clearance_m


def optimize_and_benchmark(*, train_scenarios: Sequence[int] = tuple(range(8)), test_scenarios: Sequence[int] = tuple(range(100, 108)), user_seeds: Sequence[int] = (3, 7, 11), candidates: Sequence[SharedControlParams] | None = None) -> BenchmarkReport:
    if candidates is None:
        candidates = (
            SharedControlParams(),
            SharedControlParams(caution_ttc_s=1.00, emergency_ttc_s=0.65, wait_bonus=1.1, user_agreement_blend=0.30),
            SharedControlParams(caution_ttc_s=1.20, emergency_ttc_s=0.75, wait_bonus=1.5, speed_cap_mps=0.24, user_agreement_blend=0.40),
            SharedControlParams(caution_ttc_s=1.10, emergency_ttc_s=0.70, wait_bonus=1.3, smooth_linear_gain=0.10, smooth_angular_gain=0.15),
        )
    rows = []
    best_params = candidates[0]
    best_utility = float("-inf")
    for params in candidates:
        aggregate, _ = evaluate_policy("improved", train_scenarios, user_seeds[:2], params=params)
        utility = _utility(aggregate)
        row = asdict(params) | {"train_utility": utility} | asdict(aggregate)
        rows.append(row)
        if utility > best_utility:
            best_params = params
            best_utility = utility
    baseline, _ = evaluate_policy("fixed_blend", test_scenarios, user_seeds, max_steps=160)
    improved, _ = evaluate_policy("improved", test_scenarios, user_seeds, params=best_params, max_steps=160)
    return BenchmarkReport(baseline=baseline, improved=improved, selected_params=best_params, train_utility=best_utility, train_candidates=rows, train_scenarios=list(train_scenarios), test_scenarios=list(test_scenarios), user_seeds=list(user_seeds))


def save_report(report: BenchmarkReport, path: str | Path) -> None:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as handle:
        json.dump(asdict(report), handle, indent=2, sort_keys=True)
        handle.write("\n")
