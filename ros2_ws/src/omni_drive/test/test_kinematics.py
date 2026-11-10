import math

from omni_drive.kinematics import integrate_pose, OmniKinematics, Wheel
import pytest

R = 0.05
S = math.sqrt(0.5)

# config/omni.yaml と同じ 4 輪の X 型(全部「反時計回りに回る向き」がタイヤの正)
X4 = [Wheel('fl', 0.25, 0.25, math.radians(135)), Wheel('fr', 0.25, -0.25, math.radians(45)),
      Wheel('rl', -0.25, 0.25, math.radians(-135)), Wheel('rr', -0.25, -0.25, math.radians(-45))]
# omni.yaml のコメントにある 3 輪(120° おき)
T3 = [Wheel('front', 0.2, 0.0, math.radians(90)),
      Wheel('rear_left', -0.1, 0.1732, math.radians(-150)),
      Wheel('rear_right', -0.1, -0.1732, math.radians(-30))]


def make(wheels=X4, max_wheel_speed=100.0):
    return OmniKinematics(wheels, wheel_radius=R, max_wheel_speed=max_wheel_speed)


def approx(a, b, tol=1e-6):
    return all(abs(x - y) < tol for x, y in zip(a, b))


def test_x4_forward():
    # 前に進むときは、左の 2 輪が負、右の 2 輪が正(床を前に押す成分が cos 45°)
    assert approx(make().inverse(1.0, 0.0, 0.0), [-S / R, S / R, -S / R, S / R])


def test_x4_left():
    assert approx(make().inverse(0.0, 1.0, 0.0), [S / R, S / R, -S / R, -S / R])


def test_x4_spin_turns_all_wheels_the_same_way():
    speeds = make().inverse(0.0, 0.0, 1.0)
    radius = math.hypot(0.25, 0.25)
    assert approx(speeds, [radius / R] * 4)


@pytest.mark.parametrize('wheels', [X4, T3])
@pytest.mark.parametrize('vx,vy,omega', [
    (1.0, 0.0, 0.0), (0.0, -0.5, 0.0), (0.0, 0.0, 2.0), (0.4, 0.3, -1.1)])
def test_forward_inverts_inverse(wheels, vx, vy, omega):
    kin = make(wheels)
    assert approx(kin.forward(kin.inverse(vx, vy, omega)), (vx, vy, omega))


def test_three_wheels_spin_and_forward():
    kin = make(T3)
    radius = 0.2
    assert approx(kin.inverse(0.0, 0.0, 1.0), [radius / R] * 3, tol=1e-3)
    front, left, right = kin.inverse(1.0, 0.0, 0.0)
    assert abs(front) < 1e-9 and left < 0 < right    # 前の 1 輪は横向きなので回らない


def test_desaturates_keeping_the_ratio():
    kin = make(max_wheel_speed=10.0)
    speeds = kin.inverse(1.0, 0.5, 1.0)
    assert approx([max(abs(s) for s in speeds)], [10.0])
    vx, vy, omega = kin.forward(speeds)
    assert approx((vy / vx, omega / vx), (0.5, 1.0))


def test_rejects_bad_layouts():
    with pytest.raises(ValueError):
        make(X4[:2])                                      # 2 輪では全方向に動けない
    parallel = [Wheel(n, x, 0.0, 0.0) for n, x in (('a', 0.1), ('b', 0.0), ('c', -0.1))]
    with pytest.raises(ValueError):
        make(parallel)                                    # 全部同じ向きでは横に動けない
    with pytest.raises(ValueError):
        OmniKinematics(X4, wheel_radius=0.0, max_wheel_speed=1.0)


def test_integrate_pose():
    x, y, yaw = integrate_pose(0.0, 0.0, math.pi / 2, 1.0, 0.0, 0.0, 1.0)
    assert approx((x, y, yaw), (0.0, 1.0, math.pi / 2))
