# Indoor Micro-Pilot Protocol

## Goal
Demonstrate that a calibrated capability profile and safety shell reduce intervention rate compared with a generic controller in a controlled indoor route.

## Setup
- supervised indoor route with clear start and finish
- physical emergency stop
- operator behind the chair
- wheelchair speed cap under 0.4 m/s for pilot sessions
- logging of odometry, lidar, joystick, and controller outputs

## Route blocks
1. straight corridor
2. narrow doorway
3. 90-degree turn
4. cluttered obstacle slalom
5. return path

## Conditions
- baseline generic controller
- capability-aware controller without prior
- capability-aware controller with prior

## Primary metrics
- route completion
- interventions per minute
- minimum clearance
- comfort proxy from jerk and peak angular speed
- time to completion

## Stop rules
- any emergency stop event
- repeated oscillation near obstacle
- tracker loss
- operator concern

## Deliverables
- per-run logs
- video
- incident table
- paired results table across conditions
