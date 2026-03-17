from __future__ import annotations

import argparse
from dataclasses import asdict
import json
from pathlib import Path
import sys

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from capnav_lite_core.adaptive_shared_control import SharedControlParams, evaluate_policy, optimize_and_benchmark, save_report


def main() -> None:
    parser = argparse.ArgumentParser(description="Evaluate adaptive shared-control benchmark")
    parser.add_argument("--out", type=Path, default=Path("artifacts/dynamic_shared_control_report.json"))
    parser.add_argument("--optimize", action="store_true", help="search a small candidate grid before evaluation")
    parser.add_argument("--horizon", type=int, default=6)
    parser.add_argument("--samples", type=int, default=20)
    parser.add_argument("--risk-penalty", type=float, default=7.0)
    parser.add_argument("--margin", type=float, default=0.32)
    parser.add_argument("--agreement", type=float, default=0.4)
    parser.add_argument("--train-scenarios", nargs="*", type=int, default=[0, 1, 2])
    parser.add_argument("--test-scenarios", nargs="*", type=int, default=[100, 101, 102, 103])
    parser.add_argument("--user-seeds", nargs="*", type=int, default=[3, 7, 11])
    parser.add_argument("--max-steps", type=int, default=160)
    args = parser.parse_args()

    if args.optimize:
        candidates = (
            SharedControlParams(horizon=5, samples=24, risk_penalty=6.0, margin=0.25, agreement=0.5),
            SharedControlParams(horizon=6, samples=20, risk_penalty=7.0, margin=0.32, agreement=0.4),
            SharedControlParams(horizon=4, samples=20, risk_penalty=5.0, margin=0.22, agreement=0.55),
        )
        report = optimize_and_benchmark(
            train_scenarios=args.train_scenarios,
            test_scenarios=args.test_scenarios,
            user_seeds=args.user_seeds,
            candidates=candidates,
        )
        save_report(report, args.out)
        print(f"saved optimized report to {args.out}")
        print(f"baseline success={report.baseline.success_rate:.3f} collision={report.baseline.collision_rate:.3f} efficiency={report.baseline.path_efficiency:.3f}")
        print(f"improved success={report.improved.success_rate:.3f} collision={report.improved.collision_rate:.3f} efficiency={report.improved.path_efficiency:.3f}")
        print(f"selected params={report.selected_params}")
        return

    params = SharedControlParams(
        horizon=args.horizon,
        samples=args.samples,
        risk_penalty=args.risk_penalty,
        margin=args.margin,
        agreement=args.agreement,
    )
    train_improved, _ = evaluate_policy("improved", args.train_scenarios, args.user_seeds[:2], params=params, max_steps=args.max_steps)
    baseline, _ = evaluate_policy("baseline", args.test_scenarios, args.user_seeds, max_steps=args.max_steps)
    improved, _ = evaluate_policy("improved", args.test_scenarios, args.user_seeds, params=params, max_steps=args.max_steps)
    report = {
        "baseline": asdict(baseline),
        "improved": asdict(improved),
        "selected_params": asdict(params),
        "train_improved": asdict(train_improved),
        "train_scenarios": list(args.train_scenarios),
        "test_scenarios": list(args.test_scenarios),
        "user_seeds": list(args.user_seeds),
        "benchmark_notes": "Dynamic clinic benchmark with moving obstacles, blind-corner slowdown, chance-constrained risk scoring, selective intervention, and a final emergency safety filter.",
    }
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(f"saved report to {args.out}")
    print(f"baseline success={baseline.success_rate:.3f} collision={baseline.collision_rate:.3f} efficiency={baseline.path_efficiency:.3f}")
    print(f"improved success={improved.success_rate:.3f} collision={improved.collision_rate:.3f} efficiency={improved.path_efficiency:.3f}")
    print(f"selected params={params}")


if __name__ == "__main__":
    main()
