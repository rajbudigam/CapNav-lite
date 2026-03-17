#!/usr/bin/env python3
from __future__ import annotations

import argparse
from pathlib import Path

from capnav_lite_core import GridMap


def main() -> None:
    parser = argparse.ArgumentParser(description="Generate solvable ASCII maps for examples.")
    parser.add_argument("--output-dir", type=Path, default=Path("sim/generated_maps"))
    parser.add_argument("--count", type=int, default=3)
    args = parser.parse_args()

    args.output_dir.mkdir(parents=True, exist_ok=True)
    for idx in range(args.count):
        grid, start, goal = GridMap.random_solvable(14, 14, 0.25, 100 + idx)
        text = grid.render_ascii(start=start, goal=goal)
        path = args.output_dir / f"map_{idx:02d}.txt"
        path.write_text(text + "\n", encoding="utf-8")
        print(path)


if __name__ == "__main__":
    main()
