#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
from dataclasses import asdict
from pathlib import Path

from capnav_lite_core.benchmark import run_benchmark, run_multiseed_benchmark, save_benchmark


def main() -> None:
    parser = argparse.ArgumentParser(description="Evaluate the paper-faithful hard benchmark.")
    parser.add_argument("--output", type=Path, default=Path("artifacts/paper_hard_summary.json"))
    parser.add_argument("--maps", type=int, default=24)
    parser.add_argument("--episodes", type=int, default=60)
    parser.add_argument("--meta-tasks", type=int, default=20)
    parser.add_argument("--seed", type=int, default=13)
    parser.add_argument("--multiseed", action="store_true")
    args = parser.parse_args()

    if args.multiseed:
        summary = run_multiseed_benchmark(eval_maps=args.maps, adaptation_episodes=args.episodes, meta_tasks=args.meta_tasks)
    else:
        summary = run_benchmark(eval_maps=args.maps, adaptation_episodes=args.episodes, meta_tasks=args.meta_tasks, seed=args.seed)
    save_benchmark(summary, args.output)
    print(json.dumps(asdict(summary), indent=2))


if __name__ == "__main__":
    main()
