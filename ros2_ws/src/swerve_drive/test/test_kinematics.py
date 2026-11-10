import math

import pytest
from swerve_drive.kinematics import (cosine_scale, integrate_pose, Module, SwerveKinematics,
                                     wrap_angle)

# 0.5 m 四方の四隅にモジュール(左前・右前・左後・右後)
MODULES = [Module('fl', 0.25, 0.25), Module('fr', 0.25, -0.25),
           Module('rl', -0.25, 0.25), Module('rr', -0.25, -0.25)]
R = 0.05


def make(max_wheel_speed=100.0):
    return SwerveKinematics(MODULES, wheel_radius=R, max_wheel_speed=max_wheel_speed)


def approx(a, b):
    return abs(a - b) < 1e-6


def test_forward_moves_all_modules_straight():
    cmds = make().inverse(1.0, 0.0, 0.0, [0.0] * 4)
    assert all(approx(c.angle, 0.0) and approx(c.wheel_speed, 1.0 / R) for c in cmds)


def test_left_moves_all_modules_sideways():
    cmds = make().inverse(0.0, 1.0, 0.0, [math.pi / 2] * 4)
    assert all(approx(c.angle, math.pi / 2) and approx(c.wheel_speed, 1.0 / R) for c in cmds)


def test_spin_in_place_points_modules_tangentially():
    cmds = make().inverse(0.0, 0.0, 1.0, [0.0] * 4)
    speed = math.hypot(0.25, 0.25) / R
    assert all(approx(abs(c.wheel_speed), speed) for c in cmds)
    # 左前のモジュールは、反時計回りに回るので左後ろ向き(135°)に進む → 近いほうの -45° で逆回転
    assert approx(cmds[0].angle, -math.pi / 4) and cmds[0].wheel_speed < 0


def test_flips_instead_of_turning_more_than_90_degrees():
    # 前を向いたまま後ろに進むときは、180° 回さずにタイヤを逆回転する
    cmds = make().inverse(-1.0, 0.0, 0.0, [0.0] * 4)
    assert all(approx(c.angle, 0.0) and approx(c.wheel_speed, -1.0 / R) for c in cmds)


def test_stop_keeps_current_angle():
    cmds = make().inverse(0.0, 0.0, 0.0, [0.3, -0.2, 1.0, 2.0])
    assert [round(c.angle, 6) for c in cmds] == [0.3, -0.2, 1.0, 2.0]
    assert all(c.wheel_speed == 0.0 for c in cmds)


def test_desaturates_keeping_the_ratio():
    kin = make(max_wheel_speed=10.0)
    cmds = kin.inverse(1.0, 0.0, 1.0, [0.0] * 4)    # 前進しながら回る
    fastest = max(abs(c.wheel_speed) for c in cmds)
    assert approx(fastest, 10.0)
    vx, vy, omega = kin.forward([c.angle for c in cmds], [c.wheel_speed for c in cmds])
    assert approx(omega / vx, 1.0) and approx(vy, 0.0)


@pytest.mark.parametrize('vx,vy,omega', [
    (1.0, 0.0, 0.0), (0.3, -0.7, 0.0), (0.0, 0.0, -2.0), (0.5, 0.4, 1.2)])
def test_forward_kinematics_inverts_inverse(vx, vy, omega):
    kin = make()
    cmds = kin.inverse(vx, vy, omega, [0.0] * 4)
    result = kin.forward([c.angle for c in cmds], [c.wheel_speed for c in cmds])
    assert all(approx(a, b) for a, b in zip(result, (vx, vy, omega)))


def test_forward_works_for_asymmetric_layout():
    kin = SwerveKinematics([Module('a', 0.4, 0.1), Module('b', -0.1, 0.3),
                            Module('c', 0.0, -0.35)], wheel_radius=R, max_wheel_speed=100.0)
    cmds = kin.inverse(0.2, -0.1, 0.7, [0.0] * 3)
    result = kin.forward([c.angle for c in cmds], [c.wheel_speed for c in cmds])
    assert all(approx(a, b) for a, b in zip(result, (0.2, -0.1, 0.7)))


def test_cosine_scale():
    assert approx(cosine_scale(0.5, 0.5), 1.0)
    assert approx(cosine_scale(math.pi / 3, 0.0), 0.5)
    assert cosine_scale(math.pi, 0.0) == 0.0
    assert approx(cosine_scale(math.pi - 0.1, -math.pi + 0.1), math.cos(0.2))   # ±π をまたぐ


def test_integrate_pose_turns_body_velocity_into_map_frame():
    # 左を向いている(yaw = 90°)ロボットが前に 1 m/s で 1 秒進むと、地図では +y に 1 m
    x, y, yaw = integrate_pose(0.0, 0.0, math.pi / 2, 1.0, 0.0, 0.0, 1.0)
    assert approx(x, 0.0) and approx(y, 1.0) and approx(yaw, math.pi / 2)


def test_wrap_angle():
    assert approx(wrap_angle(3 * math.pi / 2), -math.pi / 2)
    assert approx(wrap_angle(-3 * math.pi / 2), math.pi / 2)


def test_rejects_bad_settings():
    with pytest.raises(ValueError):
        SwerveKinematics([], wheel_radius=R, max_wheel_speed=1.0)
    with pytest.raises(ValueError):
        SwerveKinematics(MODULES, wheel_radius=0.0, max_wheel_speed=1.0)
