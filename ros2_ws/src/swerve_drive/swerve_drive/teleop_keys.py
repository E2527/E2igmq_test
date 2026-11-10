"""
キーボードの押しているキーから、ロボットの速度 (vx, vy, ω) を決める.

ROS にも画面にも依存しないので、PC だけでテストできる(test/test_teleop_keys.py)。

    W / S … 前 / 後ろ      A / D … 左 / 右(向きを変えずに横に動く)
    Q / E … 左回り / 右回り  Shift … 速くする
"""

import math

# キー → (前後, 左右, 回転) の向き
KEY_DIRECTIONS = {
    'w': (1, 0, 0),
    's': (-1, 0, 0),
    'a': (0, 1, 0),
    'd': (0, -1, 0),
    'q': (0, 0, 1),
    'e': (0, 0, -1),
}


def twist_from_keys(keys, linear_speed: float, angular_speed: float, turbo: bool = False,
                    turbo_scale: float = 2.0):
    """
    押しているキーの集合から (vx [m/s], vy [m/s], ω [rad/s]) を返す.

    斜め(W と D を同時など)でも、進む速さは linear_speed のままにする。
    反対のキー(W と S など)を同時に押すと、その向きは 0 になる。
    """
    fx = fy = fw = 0
    for key in keys:
        dx, dy, dw = KEY_DIRECTIONS.get(key.lower(), (0, 0, 0))
        fx += dx
        fy += dy
        fw += dw
    fx, fy, fw = (max(-1, min(1, v)) for v in (fx, fy, fw))
    scale = turbo_scale if turbo else 1.0
    norm = math.hypot(fx, fy)
    if norm > 0.0:
        fx, fy = fx / norm, fy / norm
    return (fx * linear_speed * scale, fy * linear_speed * scale, fw * angular_speed * scale)
