from capnav_lite_core import train_prior_on_random_maps


def test_prior_training_populates_q_table() -> None:
    prior = train_prior_on_random_maps(num_tasks=4, episodes_per_task=20, seed=3)
    assert len(prior.q) > 10
    some_state = next(iter(prior.q))
    assert set(prior.q[some_state]) >= {"F", "L", "R", "S"}
