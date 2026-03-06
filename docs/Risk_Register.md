# Risk register (v0.1)

## Sim-to-real gap
Risk: calibration or policy works in sim but fails in corridor due to sensor noise and dynamics.
Mitigation: rosbag replay validation before live sessions, conservative limits, staged speed increase.

## User variability
Risk: three parameters do not capture all relevant motor differences.
Mitigation: keep parameters in conservative ranges, allow clinician override, log failure cases to refine model.

## Safety events
Risk: collision or near-miss during pilot.
Mitigation: physical E-stop, safety shell, clinician supervision, stop rules, route markers.

## Logistics and scope creep
Risk: trying multiple sites or too many tasks before v0.1.
Mitigation: single-site pilot, fixed corridor route, strict week-by-week gates.

## Integration burden
Risk: clinics cannot integrate Nav2 plugin easily.
Mitigation: drop-in plugin with clear README, prebuilt Docker image (optional), and a simple wizard.
