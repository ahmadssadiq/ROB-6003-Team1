import math

import pytest

from roomrunner_control.controller import at_goal, next_index, remaining_distance, velocity_command, wrap

GAINS = dict(max_speed=0.2, gain=1.5, max_turn=0.5, yaw_gain=1.5)


def test_drives_forward_towards_waypoint_ahead():
    forward, left, turn = velocity_command((0, 0, 0), (1, 0), 0.0, **GAINS)
    assert forward == pytest.approx(0.2) and left == pytest.approx(0) and turn == pytest.approx(0)


def test_drives_sideways_when_waypoint_is_to_the_left():
    forward, left, _ = velocity_command((0, 0, 0), (0, 1), 0.0, **GAINS)
    assert forward == pytest.approx(0, abs=1e-12) and left == pytest.approx(0.2)


def test_rotates_command_into_robot_frame():
    # Robot faces +y; a waypoint at +x is to its right (negative left).
    forward, left, _ = velocity_command((0, 0, math.pi / 2), (1, 0), math.pi / 2, **GAINS)
    assert forward == pytest.approx(0, abs=1e-12) and left == pytest.approx(-0.2)


def test_slows_down_near_waypoint_and_saturates_turn():
    forward, _, turn = velocity_command((0, 0, 0), (0.05, 0), 3.0, **GAINS)
    assert forward == pytest.approx(0.075) and turn == pytest.approx(0.5)


def test_next_index_skips_close_waypoints_but_not_the_last():
    points = [(0, 0), (0.1, 0), (1, 0)]
    assert next_index(points, 0, (0.05, 0), 0.15) == 2
    assert next_index(points, 2, (1, 0), 0.15) == 2


def test_at_goal():
    assert at_goal((1.0, 1.0, 0.01), (1.01, 1.0), 0.0, 0.03, 0.05)
    assert not at_goal((1.0, 1.0, 0.2), (1.0, 1.0), 0.0, 0.03, 0.05)


def test_remaining_distance():
    assert remaining_distance((0, 0), [(1, 0), (1, 1), (2, 1)], 0) == pytest.approx(3.0)


def test_wrap():
    assert wrap(3 * math.pi / 2) == pytest.approx(-math.pi / 2)
