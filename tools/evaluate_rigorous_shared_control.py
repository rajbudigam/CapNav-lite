from __future__ import annotations

import argparse
from pathlib import Path
import sys

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from capnav_lite_core.rigorous_evaluation import run_robust_benchmark, save_report


def main() -> None:
    parser = argparse.ArgumentParser(description="Run rigorous held-out evaluation for CapNav-Lite shared control")
    parser.add_argument("--out", type=Path, default=Path("artifacts/rigorous_shared_control_report.json"))
    args = parser.parse_args()

    report = run_robust_benchmark()
    save_report(report, args.out)
    print(f"saved report to {args.out}")
    print(f"episodes={report.improved.n_episodes} scenarios={report.improved.n_scenarios}")
    print(f"baseline success={report.baseline.success_rate.mean:.3f} collision={report.baseline.collision_rate.mean:.3f} efficiency={report.baseline.path_efficiency.mean:.3f}")
    print(f"improved success={report.improved.success_rate.mean:.3f} collision={report.improved.collision_rate.mean:.3f} efficiency={report.improved.path_efficiency.mean:.3f}")
    print(f"selected params={report.selected_params}")


if __name__ == "__main__":
    main()
