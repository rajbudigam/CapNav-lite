# Reproducibility checklist (v0.1)

A third party should be able to reproduce the CPU demo from the README.

## Repository
- [ ] `colcon build` works on a clean ROS 2 Humble install
- [ ] All dependencies declared via rosdep
- [ ] Example Nav2 params included
- [ ] Example profile JSON included

## Demo
- [ ] Simulation world and launch file included or referenced
- [ ] Safety shell runs and forwards safe commands
- [ ] Controller plugin loads via pluginlib
- [ ] Calibration writes a profile in under 3 minutes

## Results reporting
- [ ] SPL and path efficiency scripts included
- [ ] One example bag file or recorded run included (if licensing permits)
- [ ] Versioned metrics output (CSV)

## Safety documentation
- [ ] E-stop wiring diagram
- [ ] Session termination checklist
- [ ] Conservative default limits explained
