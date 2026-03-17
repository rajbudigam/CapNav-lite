#!/usr/bin/env python3
from __future__ import annotations

import argparse
from pathlib import Path

from capnav_lite_core import train_prior_on_random_maps


def main() -> None:
    parser = argparse.ArgumentParser(description="Train a capability-aware prior on random maps.")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--tasks", type=int, default=18)
    parser.add_argument("--episodes", type=int, default=80)
    parser.add_argument("--seed", type=int, default=7)
    args = parser.parse_args()

    prior = train_prior_on_random_maps(num_tasks=args.tasks, episodes_per_task=args.episodes, seed=args.seed)
    prior.save(args.output)
    print(f"saved prior to {args.output}")


if __name__ == "__main__":
    main()
