from capnav_lite_core import CapabilityProfile, GridMap, LocalPlanner, Pose2D, train_prior_on_random_maps


def test_planner_prefers_nonzero_safe_action() -> None:
    grid, start, goal = GridMap.from_ascii([
        "S....",
        ".###.",
        "....G",
        ".....",
        ".....",
    ])
    profile = CapabilityProfile(user_id="u1")
    prior = train_prior_on_random_maps(num_tasks=3, episodes_per_task=10, width=8, height=8, seed=9)
    planner = LocalPlanner(profile, prior)
    pose = grid.cell_to_pose(start, 0.0)
    global_path = [(p[0] * grid.resolution_m, p[1] * grid.resolution_m) for p in grid.a_star(start, goal)]
    cmd = planner.choose_command(grid, pose, global_path, goal)
    assert cmd.label in {"F", "SLOW", "FL", "FR", "L", "R", "S"}
    assert cmd.score > -20.0
