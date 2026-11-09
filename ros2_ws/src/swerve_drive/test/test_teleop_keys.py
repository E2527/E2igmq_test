import math

from swerve_drive.teleop_keys import twist_from_keys


def approx(a, b):
    return all(abs(x - y) < 1e-9 for x, y in zip(a, b))


def test_no_keys_means_stop():
    assert twist_from_keys(set(), 0.5, 1.5) == (0.0, 0.0, 0.0)


def test_wasd_directions():
    assert approx(twist_from_keys({'w'}, 0.5, 1.5), (0.5, 0.0, 0.0))
    assert approx(twist_from_keys({'s'}, 0.5, 1.5), (-0.5, 0.0, 0.0))
    assert approx(twist_from_keys({'a'}, 0.5, 1.5), (0.0, 0.5, 0.0))     # 左 = +y
    assert approx(twist_from_keys({'d'}, 0.5, 1.5), (0.0, -0.5, 0.0))
    assert approx(twist_from_keys({'q'}, 0.5, 1.5), (0.0, 0.0, 1.5))     # 左回り = +ω
    assert approx(twist_from_keys({'e'}, 0.5, 1.5), (0.0, 0.0, -1.5))


def test_diagonal_keeps_speed():
    vx, vy, _ = twist_from_keys({'w', 'd'}, 0.5, 1.5)
    assert approx((math.hypot(vx, vy),), (0.5,)) and vx > 0 and vy < 0


def test_opposite_keys_cancel_and_uppercase_works():
    assert approx(twist_from_keys({'w', 's', 'q'}, 0.5, 1.5), (0.0, 0.0, 1.5))
    assert approx(twist_from_keys({'W'}, 0.5, 1.5), (0.5, 0.0, 0.0))


def test_turbo_and_move_while_turning():
    assert approx(twist_from_keys({'w', 'e'}, 0.5, 1.5, turbo=True, turbo_scale=2.0),
                  (1.0, 0.0, -3.0))
