# Validation Plan

This repository is stronger than a toy prototype, but no honest team should claim it is clinic-ready without staged validation.

## Stage 0: software checks
- unit tests for core logic
- schema validation for profile and prior files
- regression benchmarks on fixed random seeds

## Stage 1: synthetic benchmark
- compare prior vs scratch adaptation on random obstacle maps
- report reach, return, path efficiency, SPL, and steps
- lock benchmark seeds before paper submission

## Stage 2: high-fidelity simulator
- wheelchair or differential-drive base in Gazebo or Isaac Sim
- replay corridor, doorway, ramp, and clutter scenarios
- inject sensor latency and command noise

## Stage 3: bench hardware with safety driver
- indoor test lane with physical e-stop
- zero riders at first
- then able-bodied supervised driver
- then IRB or institutional process for intended users if applicable

## Stage 4: translational pilot
- predefined inclusion criteria
- clinician oversight
- stop rules, logging, adverse-event reporting
- conservative speed caps

## Hard truth

To sell something at scale, you will also need manufacturing quality systems, service processes, post-market monitoring, and device regulation strategy. This repo helps with the software and evaluation side. It does not replace that broader program.
