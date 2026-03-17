from __future__ import annotations

from concurrent.futures import ProcessPoolExecutor
from contextlib import contextmanager
from dataclasses import asdict, dataclass, field
import csv
import json
import math
import os
from pathlib import Path
import random
import statistics
from typing import Callable, Literal, Sequence

from . import adaptive_shared_control as asc
from .occupancy import GridMap, Pose2D
from .trace_replay import TraceCommand, TraceReplayPilot

UserModelName = Literal["pursuit", "autoregressive"]


@dataclass(slots=True)
class PilotState:
    prev_linear: float = 0.0
    prev_angular: float = 0.0
    freeze_steps: int = 0
    delay_pose: Pose2D | None = None
    angular_bias: float = 0.0


@dataclass(slots=True)
class MetricSummary:
    mean: float
    ci95_low: float
    ci95_high: float
    scenario_se: float


@dataclass(slots=True)
class PolicySummary:
    n_episodes: int
    n_scenarios: int
    success_rate: MetricSummary
    collision_rate: MetricSummary
    path_efficiency: MetricSummary
    intervention_rate: MetricSummary
    safety_intervention_rate: MetricSummary
    comfort_intervention_rate: MetricSummary
    override_rate: MetricSummary
    min_clearance_m: MetricSummary
    mean_steps: MetricSummary


@dataclass(slots=True)
class SensitivitySummary:
    perturbation_scale: float
    samples: int
    success_rate_mean: float
    success_rate_min: float
    collision_rate_mean: float
    collision_rate_max: float
    path_efficiency_mean: float


@dataclass(slots=True)
class McNemarResult:
    n_improved_only: int
    n_baseline_only: int
    p_value: float


@dataclass(slots=True)
class FamilyBreakdownRow:
    family: str
    policy: str
    n_episodes: int
    success_rate: float
    collision_rate: float
    path_efficiency: float
    safety_intervention_rate: float
    comfort_intervention_rate: float
    override_rate: float


@dataclass(slots=True)
class AblationRow:
    name: str
    summary: PolicySummary


@dataclass(slots=True)
class TraceRoundTripSummary:
    n_traces: int
    n_episodes: int
    live: PolicySummary
    replay: PolicySummary
    exact_match_rate: float
    trace_dir: str


@dataclass(slots=True)
class PublishableBenchmarkReport:
    policies: dict[str, PolicySummary]
    delta_vs_fixed_blend: dict[str, MetricSummary]
    mcnemar_vs_fixed_blend: McNemarResult
    selected_params: asc.SharedControlParams
    train_objective: float
    train_candidates: list[dict]
    sensitivity: SensitivitySummary
    ablations: list[AblationRow]
    family_breakdown: list[FamilyBreakdownRow]
    trace_round_trip: TraceRoundTripSummary
    train_scenarios: list[int]
    test_scenarios: list[int]
    user_seeds: list[int]
    user_models: list[str]
    notes: list[str] = field(default_factory=list)


_PILOT_STATE_BY_RNG: dict[int, PilotState] = {}


def _wrap(theta: float) -> float:
    while theta > math.pi:
        theta -= 2.0 * math.pi
    while theta < -math.pi:
        theta += 2.0 * math.pi
    return theta


def _lookahead_point(path_points: Sequence[tuple[float, float]], pose: Pose2D, lookahead_steps: int = 4) -> tuple[float, float]:
    idx = min(range(len(path_points)), key=lambda i: (pose.x - path_points[i][0]) ** 2 + (pose.y - path_points[i][1]) ** 2)
    return path_points[min(len(path_points) - 1, idx + lookahead_steps)]


def _add_border(blocked: set[tuple[int, int]], width: int, height: int) -> None:
    for x in range(width):
        blocked.add((x, 0))
        blocked.add((x, height - 1))
    for y in range(height):
        blocked.add((0, y))
        blocked.add((width - 1, y))


def _line(blocked: set[tuple[int, int]], x0: int, y0: int, x1: int, y1: int) -> None:
    if x0 == x1:
        step = 1 if y1 >= y0 else -1
        for y in range(y0, y1 + step, step):
            blocked.add((x0, y))
    elif y0 == y1:
        step = 1 if x1 >= x0 else -1
        for x in range(x0, x1 + step, step):
            blocked.add((x, y0))
    else:
        raise ValueError("axis-aligned only")


def _template_double_corridor(width: int, height: int, rng: random.Random, res: float):
    blocked: set[tuple[int, int]] = set(); _add_border(blocked, width, height)
    wall_xs = (8 + rng.randint(-1, 1), 18 + rng.randint(-1, 1)); gaps = [(3, 4), (8, 9), (13, 14)]
    for wall_x in wall_xs:
        _line(blocked, wall_x, 1, wall_x, height - 2)
        for low, high in gaps:
            low2 = max(1, low + rng.randint(-1, 0)); high2 = min(height - 2, high + rng.randint(0, 1))
            for y in range(low2, high2 + 1):
                blocked.discard((wall_x, y))
    for _ in range(8):
        x = rng.choice(list(range(2, 6)) + list(range(10, 16)) + list(range(21, width - 2))); y = rng.randint(2, height - 3)
        blocked.add((x, y));
        if rng.random() < 0.25 and y + 1 < height - 1:
            blocked.add((x, y + 1))
    obstacles = [
        asc.DynamicObstacle(10 * res, 8.5 * res, 0.16, 0.0, 0.16, (9 * res, 17 * res, 8.5 * res, 8.5 * res)),
        asc.DynamicObstacle(22 * res, 4.0 * res, 0.0, 0.12, 0.12, (22 * res, 22 * res, 3 * res, 13 * res)),
    ]
    return blocked, (2, 2), (width - 3, height - 3), obstacles, "double_corridor"


def _template_loop_ward(width: int, height: int, rng: random.Random, res: float):
    blocked: set[tuple[int, int]] = set(); _add_border(blocked, width, height)
    left, right, top, bottom = 9, width - 10, 5, height - 6
    for x in range(left, right + 1): blocked.add((x, top)); blocked.add((x, bottom))
    for y in range(top, bottom + 1): blocked.add((left, y)); blocked.add((right, y))
    for gap in range(left + 4, left + 7): blocked.discard((gap, top))
    for gap in range(right - 6, right - 3): blocked.discard((gap, bottom))
    for y in range(2, height - 2, 4):
        if y not in (top, bottom): blocked.add((4, y)); blocked.add((width - 5, y))
    obstacles = [
        asc.DynamicObstacle((left + 2) * res, (top + 1) * res, 0.16, 0.0, 0.16, ((left + 2) * res, (right - 2) * res, (top + 1) * res, (top + 1) * res)),
        asc.DynamicObstacle((right - 2) * res, (bottom - 1) * res, -0.14, 0.0, 0.16, ((left + 2) * res, (right - 2) * res, (bottom - 1) * res, (bottom - 1) * res)),
    ]
    return blocked, (2, height - 3), (width - 3, 2), obstacles, "loop_ward"


def _template_slalom(width: int, height: int, rng: random.Random, res: float):
    blocked: set[tuple[int, int]] = set(); _add_border(blocked, width, height)
    for idx, x in enumerate(range(6, width - 5, 4)):
        _line(blocked, x, 1 if idx % 2 == 0 else 5, x, height - 6 if idx % 2 == 0 else height - 2)
    obstacles = [
        asc.DynamicObstacle(7 * res, 14 * res, 0.0, -0.16, 0.21, (7 * res, 7 * res, 7 * res, 14 * res)),
        asc.DynamicObstacle(19 * res, 4 * res, 0.0, 0.15, 0.21, (19 * res, 19 * res, 4 * res, 12 * res)),
    ]
    return blocked, (2, 2), (width - 3, height - 3), obstacles, "slalom"


def _template_crossing(width: int, height: int, rng: random.Random, res: float):
    blocked: set[tuple[int, int]] = set(); _add_border(blocked, width, height)
    cx, cy = width // 2, height // 2
    for x in range(1, width - 1):
        if abs(x - cx) > 2: blocked.add((x, cy - 3)); blocked.add((x, cy + 3))
    for y in range(1, height - 1):
        if abs(y - cy) > 2: blocked.add((cx - 4, y)); blocked.add((cx + 4, y))
    obstacles = [
        asc.DynamicObstacle(5 * res, cy * res, 0.18, 0.0, 0.16, (4 * res, (width - 5) * res, cy * res, cy * res)),
        asc.DynamicObstacle(cx * res, 4 * res, 0.0, 0.16, 0.22, (cx * res, cx * res, 4 * res, (height - 5) * res)),
    ]
    return blocked, (2, cy), (width - 3, cy), obstacles, "crossing"


def _template_open_clutter(width: int, height: int, rng: random.Random, res: float):
    blocked: set[tuple[int, int]] = set(); _add_border(blocked, width, height)
    for x in range(5, width - 5, 5):
        for y in range(3, height - 3, 4):
            if rng.random() < 0.75:
                blocked.add((x, y))
                if rng.random() < 0.45: blocked.add((x + 1, y))
    obstacles = [
        asc.DynamicObstacle(6 * res, 6 * res, 0.12, 0.08, 0.20, (5 * res, (width - 6) * res, 5 * res, (height - 6) * res)),
        asc.DynamicObstacle((width - 6) * res, (height - 6) * res, -0.08, -0.06, 0.20, (5 * res, (width - 6) * res, 5 * res, (height - 6) * res)),
    ]
    return blocked, (2, 2), (width - 3, height - 3), obstacles, "open_clutter"


_TEMPLATES = [_template_double_corridor, _template_loop_ward, _template_slalom, _template_crossing, _template_open_clutter]


def generate_diverse_clinic_scenario(seed: int, width: int = 30, height: int = 20, resolution_m: float = 0.25) -> asc.DynamicScenario:
    template = _TEMPLATES[seed % len(_TEMPLATES)]
    for attempt in range(20):
        blocked, start, goal, obstacles, family = template(width, height, random.Random(seed * 97 + attempt), resolution_m)
        grid = GridMap(width, height, resolution_m, blocked)
        if grid.a_star(start, goal):
            return asc.DynamicScenario(grid=grid, start=grid.cell_to_pose(start, yaw=0.0), goal_cell=goal, dynamic_obstacles=obstacles, name=f"{family}_{seed}")
    raise RuntimeError(f"could not generate scenario {seed}")


def family_for_seed(seed: int) -> str:
    return generate_diverse_clinic_scenario(seed).name.rsplit("_", 1)[0]


def pursuit_user_command(profile, pose: Pose2D, path_points, rng: random.Random):
    gx, gy = _lookahead_point(path_points, pose, 4)
    heading_error = _wrap(math.atan2(gy - pose.y, gx - pose.x) - pose.yaw)
    linear = profile.comfort_linear_speed_mps * max(0.0, math.cos(heading_error))
    angular = max(-profile.comfort_angular_speed_rad_s, min(profile.comfort_angular_speed_rad_s, 1.1 * heading_error))
    if rng.random() < profile.slip_probability:
        angular += rng.uniform(-0.75, 0.75); linear *= rng.uniform(0.3, 1.0)
    linear += rng.gauss(0.0, 0.02); angular += rng.gauss(0.0, 0.06)
    linear = max(-profile.max_linear_speed_mps, min(profile.max_linear_speed_mps, linear))
    angular = max(-profile.max_angular_speed_rad_s, min(profile.max_angular_speed_rad_s, angular))
    label = min(asc._COMMAND_LIBRARY, key=lambda command: (command[1] - linear) ** 2 + (command[2] - angular) ** 2)[0]
    return label, linear, angular


def make_autoregressive_user_command():
    def command(profile, pose: Pose2D, path_points, rng: random.Random):
        key = id(rng); state = _PILOT_STATE_BY_RNG.setdefault(key, PilotState())
        delay_pose = state.delay_pose or pose; state.delay_pose = Pose2D(pose.x, pose.y, pose.yaw)
        gx, gy = _lookahead_point(path_points, delay_pose, 3)
        heading_error = _wrap(math.atan2(gy - delay_pose.y, gx - delay_pose.x) - delay_pose.yaw)
        desired_linear = profile.comfort_linear_speed_mps * max(0.0, math.cos(heading_error)); desired_angular = 0.85 * heading_error + state.angular_bias
        if state.freeze_steps > 0:
            state.freeze_steps -= 1; desired_linear = 0.0; desired_angular = 0.4 * state.prev_angular
        else:
            if rng.random() < 0.18:
                state.angular_bias += rng.uniform(-0.08, 0.08); state.angular_bias = max(-0.18, min(0.18, state.angular_bias))
            if abs(heading_error) > 0.9 and rng.random() < 0.12: state.freeze_steps = 1
        linear = 0.72 * state.prev_linear + 0.28 * desired_linear + rng.gauss(0.0, 0.018)
        angular = 0.68 * state.prev_angular + 0.32 * desired_angular + rng.gauss(0.0, 0.05)
        if rng.random() < 0.04 + 0.3 * profile.slip_probability:
            linear *= rng.uniform(0.0, 0.7); angular += rng.uniform(-0.22, 0.22)
        linear = max(-profile.max_linear_speed_mps, min(profile.max_linear_speed_mps, linear)); angular = max(-profile.max_angular_speed_rad_s, min(profile.max_angular_speed_rad_s, angular))
        state.prev_linear = linear; state.prev_angular = angular
        label = min(asc._COMMAND_LIBRARY, key=lambda cmd: (cmd[1] - linear) ** 2 + (cmd[2] - angular) ** 2)[0]
        return label, linear, angular
    return command


def make_trace_replay_user_command(pilot: TraceReplayPilot):
    def command(profile, pose: Pose2D, path_points, rng: random.Random):
        cmd = pilot.next_command()
        label = min(asc._COMMAND_LIBRARY, key=lambda item: (item[1] - cmd.linear_mps) ** 2 + (item[2] - cmd.angular_rad_s) ** 2)[0]
        return label, cmd.linear_mps, cmd.angular_rad_s
    return command


@contextmanager
def patched_environment(user_model: UserModelName | None = None, *, user_command_override: Callable | None = None):
    original_gen = asc.generate_clinic_scenario; original_cmd = asc.user_command; _PILOT_STATE_BY_RNG.clear(); asc.generate_clinic_scenario = generate_diverse_clinic_scenario
    if user_command_override is not None: asc.user_command = user_command_override
    elif user_model == "pursuit": asc.user_command = pursuit_user_command
    elif user_model == "autoregressive": asc.user_command = make_autoregressive_user_command()
    else: raise ValueError("must provide user_model or user_command_override")
    try: yield
    finally:
        asc.generate_clinic_scenario = original_gen; asc.user_command = original_cmd; _PILOT_STATE_BY_RNG.clear()


def _episode_record(task):
    policy, scenario_seed, user_seed, user_model, params, max_steps = task
    with patched_environment(user_model):
        family = family_for_seed(scenario_seed)
        result = asc.run_episode(policy, scenario_seed, user_seed, params=params, max_steps=max_steps)
        return {"policy": policy, "scenario_seed": scenario_seed, "family": family, "user_seed": user_seed, "user_model": user_model, **asdict(result)}


def evaluate_policy(policy: str, scenario_seeds: Sequence[int], user_seeds: Sequence[int], *, user_models: Sequence[UserModelName], params: asc.SharedControlParams | None = None, max_steps: int = 160):
    tasks = [(policy, scenario_seed, user_seed, user_model, params, max_steps) for user_model in user_models for scenario_seed in scenario_seeds for user_seed in user_seeds]
    if len(tasks) < 12:
        return [_episode_record(task) for task in tasks]
    with ProcessPoolExecutor(max_workers=min(8, os.cpu_count() or 1)) as ex:
        return list(ex.map(_episode_record, tasks))


def _bootstrap_metric(values: Sequence[float], rng: random.Random) -> tuple[float, float]:
    if not values: return 0.0, 0.0
    means = []; n = len(values)
    for _ in range(500):
        sample = [values[rng.randrange(n)] for _ in range(n)]; means.append(sum(sample) / n)
    means.sort(); return means[int(0.025 * len(means))], means[int(0.975 * len(means))]


def _summarize(results) -> PolicySummary:
    rng = random.Random(0)
    def summary_of(key: str):
        vals = [float(r[key]) for r in results]; mean = sum(vals) / max(len(vals), 1); low, high = _bootstrap_metric(vals, rng)
        by_scenario: dict[int, list[float]] = {}
        for r in results: by_scenario.setdefault(r["scenario_seed"], []).append(float(r[key]))
        scenario_means = [sum(v) / len(v) for _, v in sorted(by_scenario.items())]
        scenario_se = statistics.stdev(scenario_means) / math.sqrt(len(scenario_means)) if len(scenario_means) > 1 else 0.0
        return MetricSummary(mean=mean, ci95_low=low, ci95_high=high, scenario_se=scenario_se)
    return PolicySummary(n_episodes=len(results), n_scenarios=len({r["scenario_seed"] for r in results}), success_rate=summary_of("success"), collision_rate=summary_of("collision"), path_efficiency=summary_of("path_efficiency"), intervention_rate=summary_of("intervention_rate"), safety_intervention_rate=summary_of("safety_intervention_rate"), comfort_intervention_rate=summary_of("comfort_intervention_rate"), override_rate=summary_of("override_rate"), min_clearance_m=summary_of("min_clearance_m"), mean_steps=summary_of("steps"))


def _pair_metric_deltas(base_results, compare_results, key: str) -> MetricSummary:
    base_map = {(r["scenario_seed"], r["user_seed"], r["user_model"]): float(r[key]) for r in base_results}
    deltas = [float(r[key]) - base_map[(r["scenario_seed"], r["user_seed"], r["user_model"])] for r in compare_results]
    rng = random.Random(1); mean = sum(deltas) / max(len(deltas), 1); low, high = _bootstrap_metric(deltas, rng)
    by_scenario: dict[int, list[float]] = {}
    for r in compare_results:
        delta = float(r[key]) - base_map[(r["scenario_seed"], r["user_seed"], r["user_model"])]
        by_scenario.setdefault(r["scenario_seed"], []).append(delta)
    scenario_means = [sum(v) / len(v) for _, v in sorted(by_scenario.items())]
    scenario_se = statistics.stdev(scenario_means) / math.sqrt(len(scenario_means)) if len(scenario_means) > 1 else 0.0
    return MetricSummary(mean=mean, ci95_low=low, ci95_high=high, scenario_se=scenario_se)


def _objective(summary: PolicySummary) -> float:
    return 12.0 * summary.success_rate.mean - 13.0 * summary.collision_rate.mean + 2.5 * summary.path_efficiency.mean - 0.35 * summary.safety_intervention_rate.mean - 0.08 * summary.override_rate.mean + 0.6 * summary.min_clearance_m.mean


def optimize_params(*, train_scenarios: Sequence[int], user_seeds: Sequence[int], user_models: Sequence[UserModelName]):
    candidates = [
        asc.SharedControlParams(),
        asc.SharedControlParams(emergency_ttc_s=0.65, caution_ttc_s=1.00, wait_bonus=1.1, user_agreement_blend=0.28, speed_cap_mps=0.28),
        asc.SharedControlParams(emergency_ttc_s=0.72, caution_ttc_s=1.15, wait_bonus=1.4, user_agreement_blend=0.35, speed_cap_mps=0.24),
        asc.SharedControlParams(emergency_ttc_s=0.75, caution_ttc_s=1.20, wait_bonus=1.6, user_agreement_blend=0.45, speed_cap_mps=0.22),
        asc.SharedControlParams(emergency_ttc_s=0.70, caution_ttc_s=1.10, wait_bonus=1.3, user_agreement_blend=0.35, speed_cap_mps=0.26),
    ]
    rows = []; best_params = candidates[0]; best_obj = float("-inf")
    for params in candidates:
        results = evaluate_policy("improved", train_scenarios, user_seeds, user_models=user_models, params=params, max_steps=160)
        summary = _summarize(results); obj = _objective(summary)
        rows.append(asdict(params) | {"objective": obj, "success_rate": summary.success_rate.mean, "collision_rate": summary.collision_rate.mean, "path_efficiency": summary.path_efficiency.mean, "intervention_rate": summary.intervention_rate.mean, "safety_intervention_rate": summary.safety_intervention_rate.mean, "override_rate": summary.override_rate.mean, "min_clearance_m": summary.min_clearance_m.mean})
        if obj > best_obj: best_obj = obj; best_params = params
    rows.sort(key=lambda r: r["objective"], reverse=True)
    return best_params, best_obj, rows


def sensitivity_analysis(params: asc.SharedControlParams, *, scenario_seeds: Sequence[int], user_seeds: Sequence[int], user_models: Sequence[UserModelName], perturbation_scale: float = 0.20, samples: int = 5) -> SensitivitySummary:
    rng = random.Random(42); success_rates=[]; collision_rates=[]; efficiencies=[]
    fields = ["emergency_ttc_s", "caution_ttc_s", "speed_cap_mps", "caution_speed_mps", "wait_bonus", "user_agreement_blend", "smooth_linear_gain", "smooth_angular_gain"]
    base = asdict(params)
    for _ in range(samples):
        perturbed = dict(base)
        for field in fields: perturbed[field] = base[field] * (1.0 + rng.uniform(-perturbation_scale, perturbation_scale))
        summary = _summarize(evaluate_policy("improved", scenario_seeds, user_seeds, user_models=user_models, params=asc.SharedControlParams(**perturbed), max_steps=160))
        success_rates.append(summary.success_rate.mean); collision_rates.append(summary.collision_rate.mean); efficiencies.append(summary.path_efficiency.mean)
    return SensitivitySummary(perturbation_scale=perturbation_scale, samples=samples, success_rate_mean=sum(success_rates)/samples, success_rate_min=min(success_rates), collision_rate_mean=sum(collision_rates)/samples, collision_rate_max=max(collision_rates), path_efficiency_mean=sum(efficiencies)/samples)


def _exact_mcnemar_p_value(n_better: int, n_worse: int) -> float:
    n = n_better + n_worse
    if n == 0: return 1.0
    k = min(n_better, n_worse); tail = sum(math.comb(n, i) for i in range(k + 1))
    return min(1.0, 2.0 * tail / (2 ** n))


def mcnemar_success(base_results, compare_results) -> McNemarResult:
    base_map = {(r["scenario_seed"], r["user_seed"], r["user_model"]): bool(r["success"]) for r in base_results}
    improved_only = 0; baseline_only = 0
    for row in compare_results:
        key = (row["scenario_seed"], row["user_seed"], row["user_model"]); improved = bool(row["success"]); baseline = base_map[key]
        if improved and not baseline: improved_only += 1
        elif baseline and not improved: baseline_only += 1
    return McNemarResult(n_improved_only=improved_only, n_baseline_only=baseline_only, p_value=_exact_mcnemar_p_value(improved_only, baseline_only))


def family_breakdown_rows(results_by_policy: dict[str, list[dict]]) -> list[FamilyBreakdownRow]:
    rows: list[FamilyBreakdownRow] = []
    families = sorted({r["family"] for results in results_by_policy.values() for r in results})
    for family in families:
        for policy, results in results_by_policy.items():
            subset = [r for r in results if r["family"] == family]
            if not subset: continue
            rows.append(FamilyBreakdownRow(family=family, policy=policy, n_episodes=len(subset), success_rate=sum(float(r["success"]) for r in subset)/len(subset), collision_rate=sum(float(r["collision"]) for r in subset)/len(subset), path_efficiency=sum(float(r["path_efficiency"]) for r in subset)/len(subset), safety_intervention_rate=sum(float(r["safety_intervention_rate"]) for r in subset)/len(subset), comfort_intervention_rate=sum(float(r["comfort_intervention_rate"]) for r in subset)/len(subset), override_rate=sum(float(r["override_rate"]) for r in subset)/len(subset)))
    return rows


def run_ablations(*, test_scenarios: Sequence[int], user_seeds: Sequence[int], user_models: Sequence[UserModelName], base_params: asc.SharedControlParams) -> list[AblationRow]:
    configs = [
        ("Full system", base_params),
        ("- Lattice scoring", asc.SharedControlParams(**(asdict(base_params) | {"use_lattice_scoring": False}))),
        ("- Adaptive threshold", asc.SharedControlParams(**(asdict(base_params) | {"adaptive_threshold": False}))),
        ("- Reliability estimation", asc.SharedControlParams(**(asdict(base_params) | {"use_reliability_estimation": False}))),
        ("- Blind corner slowdown", asc.SharedControlParams(**(asdict(base_params) | {"use_blind_corner_slowdown": False}))),
        ("- Dynamic obstacle pred.", asc.SharedControlParams(**(asdict(base_params) | {"use_dynamic_prediction": False}))),
    ]
    return [AblationRow(name=name, summary=_summarize(evaluate_policy("improved", test_scenarios, user_seeds, user_models=user_models, params=params, max_steps=160))) for name, params in configs]


def _write_trace_csv(path: Path, commands: Sequence[TraceCommand]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=["linear_mps", "angular_rad_s"]); writer.writeheader()
        for cmd in commands: writer.writerow({"linear_mps": cmd.linear_mps, "angular_rad_s": cmd.angular_rad_s})


def generate_synthetic_trace(profile, scenario_seed: int, user_seed: int, *, user_model: UserModelName = "pursuit", n_steps: int = 180) -> list[TraceCommand]:
    scenario = generate_diverse_clinic_scenario(scenario_seed); pose = Pose2D(scenario.start.x, scenario.start.y, scenario.start.yaw); path_points = asc._path_points(scenario); rng = random.Random(user_seed * 1000 + scenario_seed)
    command_fn = pursuit_user_command if user_model == "pursuit" else make_autoregressive_user_command()
    commands: list[TraceCommand] = []
    for _ in range(n_steps):
        _, linear, angular = command_fn(profile, pose, path_points, rng); commands.append(TraceCommand(linear_mps=linear, angular_rad_s=angular)); pose = asc._step_robot(pose, linear, angular, 0.3)
        for obstacle in scenario.dynamic_obstacles: obstacle.step(0.3)
        if asc._collision_static(scenario.grid, pose.x, pose.y): break
        if math.dist((pose.x, pose.y), asc._cell_to_xy(scenario.grid, scenario.goal_cell)) < 0.35: break
    return commands


def _evaluate_with_trace_pilot(policy: str, scenario_seed: int, user_seed: int, pilot: TraceReplayPilot, *, params: asc.SharedControlParams, max_steps: int = 160) -> dict:
    with patched_environment(user_command_override=make_trace_replay_user_command(pilot)):
        family = family_for_seed(scenario_seed); result = asc.run_episode(policy, scenario_seed, user_seed, params=params, max_steps=max_steps)
        return {"policy": policy, "scenario_seed": scenario_seed, "family": family, "user_seed": user_seed, "user_model": "trace_replay", **asdict(result)}


def trace_round_trip_experiment(*, params: asc.SharedControlParams, out_dir: Path, scenarios: Sequence[int] = (150, 151, 152, 153), user_seeds: Sequence[int] = (3, 7, 11)) -> TraceRoundTripSummary:
    trace_dir = out_dir / "synthetic_traces"; live_results=[]; replay_results=[]; matches=0; total=0
    for scenario_seed in scenarios:
        for user_seed in user_seeds:
            profile = asc._default_profile(user_seed); commands = generate_synthetic_trace(profile, scenario_seed, user_seed, user_model="pursuit", n_steps=180)
            csv_path = trace_dir / f"trace_s{scenario_seed}_u{user_seed}.csv"; _write_trace_csv(csv_path, commands)
            live_row = _evaluate_with_trace_pilot("improved", scenario_seed, user_seed, TraceReplayPilot(commands=list(commands), loop=True), params=params)
            replay_row = _evaluate_with_trace_pilot("improved", scenario_seed, user_seed, TraceReplayPilot.from_csv(csv_path, loop=True), params=params)
            live_results.append(live_row); replay_results.append(replay_row)
            if live_row["success"] == replay_row["success"] and live_row["collision"] == replay_row["collision"] and abs(live_row["path_efficiency"] - replay_row["path_efficiency"]) < 1e-9: matches += 1
            total += 1
    return TraceRoundTripSummary(n_traces=total, n_episodes=total, live=_summarize(live_results), replay=_summarize(replay_results), exact_match_rate=matches/max(total,1), trace_dir=str(trace_dir))


def run_publishable_benchmark(*, train_scenarios: Sequence[int] = tuple(range(10)), test_scenarios: Sequence[int] = tuple(range(100, 120)), user_seeds: Sequence[int] = (3, 7, 11), user_models: Sequence[UserModelName] = ("pursuit", "autoregressive"), artifact_dir: str | Path = "artifacts") -> PublishableBenchmarkReport:
    artifact_dir = Path(artifact_dir); params, train_obj, rows = optimize_params(train_scenarios=train_scenarios, user_seeds=user_seeds, user_models=user_models)
    results_by_policy = {
        "user_safety": evaluate_policy("user_safety", test_scenarios, user_seeds, user_models=user_models, params=params, max_steps=160),
        "fixed_blend": evaluate_policy("fixed_blend", test_scenarios, user_seeds, user_models=user_models, params=params, max_steps=160),
        "pure_autonomy": evaluate_policy("pure_autonomy", test_scenarios, user_seeds, user_models=user_models, params=params, max_steps=160),
        "improved": evaluate_policy("improved", test_scenarios, user_seeds, user_models=user_models, params=params, max_steps=160),
    }
    summaries = {policy: _summarize(results) for policy, results in results_by_policy.items()}
    delta = {
        "success_rate": _pair_metric_deltas(results_by_policy["fixed_blend"], results_by_policy["improved"], "success"),
        "collision_rate": _pair_metric_deltas(results_by_policy["fixed_blend"], results_by_policy["improved"], "collision"),
        "path_efficiency": _pair_metric_deltas(results_by_policy["fixed_blend"], results_by_policy["improved"], "path_efficiency"),
        "safety_intervention_rate": _pair_metric_deltas(results_by_policy["fixed_blend"], results_by_policy["improved"], "safety_intervention_rate"),
        "override_rate": _pair_metric_deltas(results_by_policy["fixed_blend"], results_by_policy["improved"], "override_rate"),
    }
    return PublishableBenchmarkReport(
        policies=summaries,
        delta_vs_fixed_blend=delta,
        mcnemar_vs_fixed_blend=mcnemar_success(results_by_policy["fixed_blend"], results_by_policy["improved"]),
        selected_params=params,
        train_objective=train_obj,
        train_candidates=rows,
        sensitivity=sensitivity_analysis(params, scenario_seeds=test_scenarios[:5], user_seeds=user_seeds[:2], user_models=user_models, perturbation_scale=0.20, samples=5),
        ablations=run_ablations(test_scenarios=test_scenarios, user_seeds=user_seeds, user_models=user_models, base_params=params),
        family_breakdown=family_breakdown_rows(results_by_policy),
        trace_round_trip=trace_round_trip_experiment(params=params, out_dir=artifact_dir),
        train_scenarios=list(train_scenarios),
        test_scenarios=list(test_scenarios),
        user_seeds=list(user_seeds),
        user_models=list(user_models),
        notes=[
            "Implements four controller baselines plus five ablations and a synthetic trace round-trip validation.",
            "Held-out evaluation uses 20 scenario seeds across five layout families, two pilot models, and three user seeds for 120 episodes per policy.",
        ],
    )


def save_report(report: PublishableBenchmarkReport, path: str | Path) -> None:
    path = Path(path); path.parent.mkdir(parents=True, exist_ok=True); path.write_text(json.dumps(asdict(report), indent=2, sort_keys=True) + "\n", encoding="utf-8")
