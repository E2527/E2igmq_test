"""relocalize.py の、AMCL に頼むかどうかの判断(ROS に依存しない。test/test_relocalize.py)."""


def should_request(now, last_request, period_s, speed, still_speed):
    """
    今、AMCL に直すように頼むかどうか.

    speed は今の速さ(並進 [m/s] と回転 [rad/s] の大きい方。分からなければ None)。
    """
    if period_s <= 0.0:
        return False
    if speed is not None and speed > still_speed:
        return False                # 動いている間は、AMCL が自分で直す
    return last_request is None or now - last_request >= period_s
