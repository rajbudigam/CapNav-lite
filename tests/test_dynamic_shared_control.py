from __future__ import annotations

from capnav_lite_core.adaptive_shared_control import SharedControlParams, evaluate_policy, optimize_and_benchmark


def test_dynamic_shared_control_smoke() -> None:
    params = SharedControlParams(horizon=4, samples=12, risk_penalty=4.0, margin=0.16, agreement=0.6)
    baseline, _ = evaluate_policy("baseline", [100, 101], [3], max_steps=120)
    improved, _ = evaluate_policy("improved", [100, 101], [3], params=params, max_steps=120)
    assert improved.success_rate >= baseline.success_rate
    assert improved.path_efficiency >= baseline.path_efficiency


def test_optimizer_returns_report() -> None:
    report = optimize_and_benchmark(train_scenarios=[0, 1], test_scenarios=[100, 101], user_seeds=[3])
    assert report.improved.success_rate >= report.baseline.success_rate
    assert report.selected_params.samples > 0
