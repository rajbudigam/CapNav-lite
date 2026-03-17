from __future__ import annotations

from dataclasses import dataclass
import csv
from pathlib import Path
import statistics
from typing import Iterable, Sequence

from .profile import CapabilityProfile


@dataclass(slots=True)
class TrialPoint:
    t_s: float
    commanded_linear_mps: float
    commanded_angular_rad_s: float
    observed_linear_mps: float
    observed_angular_rad_s: float
    distance_to_reference_m: float


@dataclass(slots=True)
class CalibrationSummary:
    num_points: int
    inferred_slip_probability: float
    inferred_turning_cost: float
    inferred_backtracking_cost: float
    inferred_reaction_time_s: float
    inferred_clearance_margin_m: float


def load_trial_csv(path: str | Path) -> list[TrialPoint]:
    rows: list[TrialPoint] = []
    with Path(path).open("r", encoding="utf-8") as handle:
        for row in csv.DictReader(handle):
            rows.append(
                TrialPoint(
                    t_s=float(row["t_s"]),
                    commanded_linear_mps=float(row["commanded_linear_mps"]),
                    commanded_angular_rad_s=float(row["commanded_angular_rad_s"]),
                    observed_linear_mps=float(row["observed_linear_mps"]),
                    observed_angular_rad_s=float(row["observed_angular_rad_s"]),
                    distance_to_reference_m=float(row["distance_to_reference_m"]),
                )
            )
    return rows


def estimate_profile_from_trials(points: Sequence[TrialPoint], *, user_id: str = "calibrated-user") -> tuple[CapabilityProfile, CalibrationSummary]:
    if len(points) < 10:
        raise ValueError("need at least 10 trial points for calibration")

    linear_observed = [abs(p.observed_linear_mps) for p in points]
    angular_observed = [abs(p.observed_angular_rad_s) for p in points]
    max_linear = max(linear_observed)
    max_angular = max(angular_observed)
    comfort_linear = statistics.quantiles(linear_observed, n=10)[5]
    comfort_angular = statistics.quantiles(angular_observed, n=10)[5]

    mismatch = 0
    corrective_events = 0
    delays = []
    reference_distances = [p.distance_to_reference_m for p in points]
    for prev, cur in zip(points, points[1:]):
        if abs(prev.commanded_linear_mps) > 0.01:
            same_sign = prev.commanded_linear_mps * cur.observed_linear_mps >= 0
            if not same_sign or abs(cur.observed_linear_mps) < 0.15 * abs(prev.commanded_linear_mps):
                mismatch += 1
        if abs(cur.commanded_angular_rad_s) > 0.2 and abs(cur.observed_angular_rad_s) < 0.1:
            corrective_events += 1
        if abs(prev.commanded_linear_mps) < 1e-3 and abs(cur.commanded_linear_mps) > 0.05:
            delays.append(max(0.0, cur.t_s - prev.t_s))

    slip_probability = min(0.45, mismatch / max(1, len(points) - 1))
    turning_cost = 1.0 + corrective_events / max(1, len(points) // 8)
    backtracking_cost = 1.0 + sum(1 for p in points if p.commanded_linear_mps < -0.05) / max(1, len(points) // 6)
    reaction_time = statistics.fmean(delays) if delays else 0.35
    clearance_margin = min(0.6, max(0.2, statistics.fmean(reference_distances) * 0.65))

    profile = CapabilityProfile(
        user_id=user_id,
        max_linear_speed_mps=round(max_linear * 1.05, 3),
        max_angular_speed_rad_s=round(max_angular * 1.05, 3),
        comfort_linear_speed_mps=round(max(0.08, comfort_linear), 3),
        comfort_angular_speed_rad_s=round(max(0.12, comfort_angular), 3),
        turning_cost=round(turning_cost, 3),
        backtracking_cost=round(backtracking_cost, 3),
        slip_probability=round(slip_probability, 3),
        reaction_time_s=round(reaction_time, 3),
        clearance_margin_m=round(clearance_margin, 3),
        notes="Estimated from calibration traces.",
    )
    summary = CalibrationSummary(
        num_points=len(points),
        inferred_slip_probability=profile.slip_probability,
        inferred_turning_cost=profile.turning_cost,
        inferred_backtracking_cost=profile.backtracking_cost,
        inferred_reaction_time_s=profile.reaction_time_s,
        inferred_clearance_margin_m=profile.clearance_margin_m,
    )
    return profile, summary
