from capnav_lite_core import run_benchmark


def test_benchmark_runs_and_prior_helps_on_average() -> None:
    summary = run_benchmark(eval_maps=8, adaptation_episodes=60, meta_tasks=10, seed=11)
    assert summary.pretrained["reach"] > summary.scratch["reach"]
    assert summary.delta_reach > 0.25
