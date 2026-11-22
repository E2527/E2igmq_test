import math

from field_map import sim_truth as st
import pytest


def close(a, b, tol=1e-6):
    return all(abs(x - y) < tol for x, y in zip(a, b))


def test_compose_inverse():
    a = (1.0, 2.0, 0.7)
    assert close(st.compose(a, st.inverse(a)), (0.0, 0.0, 0.0))
    b = (0.3, -0.4, -1.2)
    assert close(st.compose(a, st.relative(a, b)), b)


def test_truth_follows_odometry_without_faults():
    t = st.TruthTracker()
    t.on_odom((0.0, 0.0, 0.0))
    t.on_odom((1.0, 0.0, 0.0), dt=1.0)
    assert close(t.truth, (1.0, 0.0, 0.0)) and t.error() == (0.0, 0.0)


def test_slip_makes_truth_fall_behind():
    t = st.TruthTracker()
    t.on_odom((0.0, 0.0, 0.0))
    for i in range(1, 11):
        t.on_odom((0.1 * i, 0.0, 0.0), dt=0.1, slip=0.2)
    assert t.truth[0] == pytest.approx(0.8)              # 1 m 進んだつもりが 0.8 m
    assert t.error()[0] == pytest.approx(0.2)


def test_drift_turns_the_truth():
    t = st.TruthTracker()
    t.on_odom((0.0, 0.0, 0.0))
    for i in range(1, 11):
        t.on_odom((0.0, 0.0, 0.0), dt=0.1, drift_rad_s=math.radians(10))
    assert math.degrees(t.truth[2]) == pytest.approx(10.0)


def test_teleport_moves_truth_and_estimate():
    t = st.TruthTracker()
    t.on_odom((0.5, 0.2, 0.3))
    t.teleport((-2.0, 1.0, math.pi / 2))
    assert close(t.truth, (-2.0, 1.0, math.pi / 2))
    assert close(t.estimate(), (-2.0, 1.0, math.pi / 2))
    # 置き直したあと、ロボットの前(+x)に 1 m 進むと、地図では +y に 1 m
    t.on_odom(st.compose((0.5, 0.2, 0.3), (1.0, 0.0, 0.0)), dt=1.0)
    assert close(t.truth, (-2.0, 2.0, math.pi / 2))
    assert close(t.estimate(), t.truth)


def test_teleport_before_odometry_waits():
    t = st.TruthTracker()
    t.teleport((1.0, 1.0, 0.0))
    assert t.truth is None
    t.on_odom((0.0, 0.0, 0.0))
    assert close(t.truth, (1.0, 1.0, 0.0)) and close(t.estimate(), (1.0, 1.0, 0.0))


def test_odometry_restart_resyncs():
    t = st.TruthTracker()
    t.on_odom((5.0, 0.0, 0.0))
    t.on_odom((0.0, 0.0, 0.0), dt=0.02, slip=0.5)          # 5 m 飛んだ: やり直し
    assert close(t.truth, t.estimate())


def test_start_poses():
    info = {'start_poses': {'red': {'x': -2, 'y': -1.5, 'yaw_deg': 90}, 'bad': 'x',
                            'blue': {'x': 2.5}}}
    poses = st.start_poses(info)
    assert set(poses) == {'red', 'blue'}
    assert close(poses['red'], (-2.0, -1.5, math.pi / 2)) and close(poses['blue'], (2.5, 0, 0))
    assert st.start_poses({}) == {}
