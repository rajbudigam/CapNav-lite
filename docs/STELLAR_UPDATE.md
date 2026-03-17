# Stellar update

This update focuses on two practical weaknesses that were still blocking confidence in the repo:

1. the held-out evaluation was too small
2. the trace replay path was too thin for public simulator datasets

## What changed

- added flexible real-trace ingestion in `capnav_lite_core.trace_replay`
- added `TraceReplayPilot.from_wheelsim_csv()` as a best-effort parser for WheelSim-style Unity exports
- added `docs/REAL_TRACE_INTEGRATION.md`
- added a regression test for flexible joystick-axis CSV parsing
- added an extended held-out report over 10 scenario seeds, 5 layout families, 2 pilot models, and 3 user seeds

## Extended held-out result

Report file: `artifacts/extended_heldout_60ep_report.json`

Top-line numbers:
- baseline success: 28.3%
- tuned controller success: 45.0%
- baseline collision: 31.7%
- tuned controller collision: 23.3%
- baseline path efficiency: 0.283
- tuned controller path efficiency: 0.450

This is still a simulator result. It is stronger than the earlier 30-episode result because it covers 60 held-out episodes and a broader set of layouts.

## What is still not solved

- the held-out users are still synthetic, even though the repo can now ingest real traces
- the public WheelSim docs do not expose a stable CSV schema in the material available here
- no ROS compile validation or hardware validation has been run in this environment
- nothing here should be sold or deployed without real safety validation
