from pathlib import Path

from capnav_lite_core import CapabilityProfile, load_profile, save_profile


def test_profile_roundtrip(tmp_path: Path) -> None:
    path = tmp_path / "profile.json"
    profile = CapabilityProfile(user_id="alice", turning_cost=1.7, slip_probability=0.14)
    save_profile(profile, path)
    loaded = load_profile(path)
    assert loaded.user_id == "alice"
    assert abs(loaded.turning_cost - 1.7) < 1e-9
    assert abs(loaded.slip_probability - 0.14) < 1e-9


def test_trace_replay_flexible_csv(tmp_path):
    from capnav_lite_core.trace_replay import TraceReplayPilot

    path = tmp_path / 'trace.csv'
    path.write_text('joystick_x,joystick_y\n0.5,1.0\n-0.25,0.0\n', encoding='utf-8')
    pilot = TraceReplayPilot.from_flexible_csv(path, max_linear_mps=0.3, max_angular_rad_s=0.8)
    first = pilot.next_command()
    second = pilot.next_command()
    assert abs(first.linear_mps - 0.3) < 1e-9
    assert abs(first.angular_rad_s - 0.4) < 1e-9
    assert abs(second.linear_mps - 0.0) < 1e-9
    assert abs(second.angular_rad_s + 0.2) < 1e-9
