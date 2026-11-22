from robot_nav.relocalize_timing import should_request


def test_requests_periodically_while_still():
    assert should_request(0.0, None, 3.0, 0.0, 0.02)
    assert not should_request(1.0, 0.0, 3.0, 0.0, 0.02)
    assert should_request(3.0, 0.0, 3.0, 0.0, 0.02)


def test_not_while_moving_or_disabled():
    assert not should_request(10.0, 0.0, 3.0, 0.3, 0.02)
    assert not should_request(10.0, None, 0.0, 0.0, 0.02)


def test_unknown_speed_still_requests():
    assert should_request(10.0, 0.0, 3.0, None, 0.02)
