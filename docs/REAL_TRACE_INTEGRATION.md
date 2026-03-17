# Real trace integration

This repo now includes a more practical trace-ingestion path in `capnav_lite_core.trace_replay`.

What it can do now:
- load clean trace CSVs with `linear_mps` and `angular_rad_s`
- infer commands from flexible column names such as `speed`, `yaw_rate`, `joystick_x`, `joystick_y`, `axis_x`, `axis_y`
- attempt best-effort parsing of WheelSimPhysio-style Unity exports through `TraceReplayPilot.from_wheelsim_csv()`

What it cannot honestly guarantee:
- a stable public WheelSimPhysio column schema
- that the public dataset is commercially usable without checking the license yourself
- that replay alone is enough for clinical or product claims

## Why this path exists

Public descriptions of WheelSimPhysio-2023 report 58 participants and state that simulator performance data includes joystick events and collisions. The GitHub README also says the Unity-side `Performance.csv` is part of the dataset. The public descriptions do not expose a stable per-column schema in the docs available here, so the parser is heuristic rather than dataset-specific.

## Minimal example

```python
from capnav_lite_core.trace_replay import TraceReplayPilot

pilot = TraceReplayPilot.from_flexible_csv(
    "my_trace.csv",
    max_linear_mps=0.30,
    max_angular_rad_s=0.75,
)
```

## WheelSim-style example

```python
from capnav_lite_core.trace_replay import TraceReplayPilot

pilot = TraceReplayPilot.from_wheelsim_csv(
    "Performance.csv",
    max_linear_mps=0.30,
    max_angular_rad_s=0.75,
)
```

## Recommended next step

Download the dataset manually from its official host, inspect one real `Performance.csv`, then pin the exact column names in a local adapter. That is the right way to make the replay benchmark genuinely trace-grounded.
