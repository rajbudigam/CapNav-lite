from __future__ import annotations

from dataclasses import dataclass, asdict
import json
from pathlib import Path
from typing import Any, Dict


def _clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


@dataclass(slots=True)
class CapabilityProfile:
    user_id: str = "unknown"
    max_linear_speed_mps: float = 0.45
    max_angular_speed_rad_s: float = 0.75
    comfort_linear_speed_mps: float = 0.28
    comfort_angular_speed_rad_s: float = 0.45
    turning_cost: float = 1.2
    backtracking_cost: float = 1.0
    slip_probability: float = 0.08
    reaction_time_s: float = 0.35
    clearance_margin_m: float = 0.30
    joystick_deadband: float = 0.05
    notes: str = ""

    def validate(self) -> None:
        if not self.user_id:
            raise ValueError("user_id must be non-empty")
        for field in (
            "max_linear_speed_mps",
            "max_angular_speed_rad_s",
            "comfort_linear_speed_mps",
            "comfort_angular_speed_rad_s",
            "turning_cost",
            "backtracking_cost",
            "reaction_time_s",
            "clearance_margin_m",
            "joystick_deadband",
        ):
            value = getattr(self, field)
            if value < 0:
                raise ValueError(f"{field} must be non-negative")
        self.slip_probability = _clamp(self.slip_probability, 0.0, 0.6)
        if self.comfort_linear_speed_mps > self.max_linear_speed_mps:
            raise ValueError("comfort_linear_speed_mps cannot exceed max_linear_speed_mps")
        if self.comfort_angular_speed_rad_s > self.max_angular_speed_rad_s:
            raise ValueError("comfort_angular_speed_rad_s cannot exceed max_angular_speed_rad_s")

    def normalized(self) -> "CapabilityProfile":
        self.validate()
        return CapabilityProfile(**asdict(self))

    def to_dict(self) -> Dict[str, Any]:
        self.validate()
        return asdict(self)

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> "CapabilityProfile":
        allowed = {f.name for f in cls.__dataclass_fields__.values()}  # type: ignore[attr-defined]
        payload = {k: v for k, v in data.items() if k in allowed}
        profile = cls(**payload)
        profile.validate()
        return profile


def load_profile(path: str | Path) -> CapabilityProfile:
    path = Path(path)
    with path.open("r", encoding="utf-8") as handle:
        return CapabilityProfile.from_dict(json.load(handle))


def save_profile(profile: CapabilityProfile, path: str | Path) -> None:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as handle:
        json.dump(profile.to_dict(), handle, indent=2, sort_keys=True)
        handle.write("\n")
