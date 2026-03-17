from __future__ import annotations

from dataclasses import dataclass

from .local_planner import VelocityCommand
from .profile import CapabilityProfile


@dataclass(slots=True)
class BlendDecision:
    autonomy_weight: float
    user_weight: float
    safe_command: VelocityCommand
    reason: str


class SafetyEnvelope:
    def __init__(self, profile: CapabilityProfile, hard_stop_ttc_s: float = 0.7, slow_ttc_s: float = 1.6):
        self.profile = profile.normalized()
        self.hard_stop_ttc_s = hard_stop_ttc_s
        self.slow_ttc_s = slow_ttc_s

    def enforce(self, command: VelocityCommand, *, front_clearance_m: float, estimated_ttc_s: float, risk_score: float) -> VelocityCommand:
        linear = command.linear_mps
        angular = command.angular_rad_s
        label = command.label
        score = command.score
        if estimated_ttc_s <= self.hard_stop_ttc_s or front_clearance_m <= self.profile.clearance_margin_m:
            return VelocityCommand(0.0, 0.0, score, "E_STOP")
        if estimated_ttc_s < self.slow_ttc_s or risk_score > 0.8:
            linear *= 0.35
            angular *= 0.55
            label = f"SAFE_{label}"
        linear = max(-self.profile.max_linear_speed_mps, min(self.profile.max_linear_speed_mps, linear))
        angular = max(-self.profile.max_angular_speed_rad_s, min(self.profile.max_angular_speed_rad_s, angular))
        return VelocityCommand(linear, angular, score, label)

    def blend(self, autonomy: VelocityCommand, user: VelocityCommand, *, risk_score: float, autonomy_confidence: float) -> BlendDecision:
        autonomy_weight = min(1.0, max(0.0, 0.2 + 0.6 * risk_score + 0.2 * autonomy_confidence))
        user_weight = 1.0 - autonomy_weight
        if risk_score < 0.25 and autonomy_confidence < 0.35:
            autonomy_weight = 0.25
            user_weight = 0.75
        linear = autonomy_weight * autonomy.linear_mps + user_weight * user.linear_mps
        angular = autonomy_weight * autonomy.angular_rad_s + user_weight * user.angular_rad_s
        return BlendDecision(
            autonomy_weight=autonomy_weight,
            user_weight=user_weight,
            safe_command=VelocityCommand(linear, angular, max(autonomy.score, user.score), "BLEND"),
            reason="risk-weighted shared control",
        )
