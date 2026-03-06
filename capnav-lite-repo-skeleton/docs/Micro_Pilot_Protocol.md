# Micro-Pilot Protocol (one-site, clinician supervised)

This protocol is designed for a small pilot in a rehabilitation hospital or assistive technology center. It prioritizes safety, clear stopping rules, and easy reproducibility.

## Scope
- Single corridor route with turns
- Two trials per participant
  - Trial A: default profile
  - Trial B: personalized profile
- Five to eight participants
- Five to eight sessions total, scheduled with clinician oversight

## Inclusion and exclusion
Include participants who:
- use a powered mobility device or are in a powered mobility training program
- can give informed consent (or have a guardian consent)
- can participate in a short supervised navigation task

Exclude participants who:
- have uncontrolled medical issues that make short mobility sessions unsafe
- have severe cognitive impairment that prevents following instructions
- are uncomfortable with the E-stop and supervision requirements

## Environment
- Mark a corridor route with tape markers.
- Minimum width: 1.4 m (or match the clinic setting).
- Add two 90 degree turns and one doorway-like narrowing if available.
- Ensure the floor is dry and clear.

## Hardware safety requirements
- Physical E-stop that cuts motion power, within reach of supervisor.
- Secondary stop button held by the supervisor if available.
- Bumper switches or a soft bumper.
- Speed limits configured in software and in the motor controller where possible.

## Software safety requirements
- Safety shell active and verified.
- Conservative limits in profile.
- Shared-control fallback configured, or stop-and-alert if mux is not available.
- A stop checklist and session termination rules, below.

## Session flow
1) Pre-check (2 minutes)
- Verify E-stop works.
- Verify safety shell is publishing and not stopping on stale scan.
- Verify route is clear.
- Confirm participant comfort.

2) Trial A (default)
- Load default profile.
- Start at marker A.
- Run the route once with supervisor present.
- Record outcomes and any interventions.

3) Calibration (3 minutes)
- Run calibration wizard or CLI routine.
- Save profile JSON.

4) Trial B (personalized)
- Load personalized profile.
- Run the same route once.
- Record outcomes and any interventions.

5) Post-check (2 minutes)
- Ask short comfort and trust rating (1 to 5).
- Save logs and profile.

## Stopping rules (terminate immediately)
- Any contact with a person.
- Any collision with a wall or object above light bumper touch.
- Any loss of control that requires rapid manual intervention.
- Any participant discomfort or request to stop.
- Any sensor failure that disables reliable clearance detection.

## Metrics
Primary:
- SPL (Success weighted by Path Length)
  - success is 1 if the robot reaches goal within time limit, else 0
  - SPL = success * (shortest_path_length / max(actual_path_length, shortest_path_length))

Secondary:
- path efficiency = shortest_path_length / actual_path_length on successful runs
- steps on success (time steps to reach goal)
- number of supervisor interventions
- participant rating (comfort and trust)

## Data capture
- Save profile JSON for each participant and session (opaque ID).
- Save Nav2 logs.
- Optional but recommended: rosbag for offline replay validation.

## Sim-to-real mitigation
Before any live session:
- Record a rosbag of the corridor.
- Replay it offline with the same profile and verify the safety shell triggers correctly on injected faults.

## Ethics and privacy
- Use opaque identifiers.
- Do not store names in logs.
- Store data on an encrypted drive accessible only to the pilot team.
- Obtain local ethics approval if required by the institution.
