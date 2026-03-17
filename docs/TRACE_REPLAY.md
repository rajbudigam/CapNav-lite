# Trace replay pilot interface

This repo includes a `TraceReplayPilot` helper in `capnav_lite_core.trace_replay` for evaluating the shared controller against recorded command traces.

Supported paths:
- strict CSV input with `linear_mps` and `angular_rad_s`
- flexible CSV input with inferred columns such as `speed`, `yaw_rate`, `joystick_x`, `joystick_y`
- best-effort WheelSim-style Unity exports through `TraceReplayPilot.from_wheelsim_csv()`

Expected strict CSV columns:
- `linear_mps`
- `angular_rad_s`

Minimal strict example:

```csv
linear_mps,angular_rad_s
0.12,0.00
0.10,0.20
0.00,0.55
```

Minimal flexible example:

```csv
joystick_x,joystick_y
0.30,0.85
-0.10,0.25
```

This still does not magically make the benchmark real-world validated. It is an interface for plugging in real joystick or alternative pilot traces once they are available.
