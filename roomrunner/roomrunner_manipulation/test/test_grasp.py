import math

import pytest

from roomrunner_manipulation.grasp import in_reach, object_in_base, pick_sequence, place_sequence

S = dict(grasp_z=-0.01, hover_height=0.05, lift_height=0.06, place_z=0.02, gripper_open=40.0, gripper_closed=0.0)


def test_object_straight_ahead():
    assert object_in_base((1.0, 2.0, 0.0), (1.3, 2.0)) == pytest.approx((0.3, 0.0))


def test_object_with_robot_turned():
    # Robot faces +y; an object at +x of the robot is on its right.
    assert object_in_base((0.0, 0.0, math.pi / 2), (0.3, 0.0)) == pytest.approx((0.0, -0.3), abs=1e-12)


def test_in_reach():
    shoulder = (0.067, 0.0)
    assert in_reach((0.30, 0.0), shoulder, 0.12, 0.30)
    assert not in_reach((0.50, 0.0), shoulder, 0.12, 0.30)
    assert not in_reach((0.10, 0.0), shoulder, 0.12, 0.30)


def test_pick_sequence_order_and_gripper():
    phases = pick_sequence((0.28, 0.02), S)
    assert [p[0] for p in phases] == ["approach", "descend", "grasp", "lift"]
    assert phases[1][2] == S["gripper_open"] and phases[2][2] == S["gripper_closed"]
    heights = [p[1][2] for p in phases]
    assert heights[0] > heights[1] and heights[3] > heights[2]
    assert all(p[1][:2] == (0.28, 0.02) for p in phases)


def test_place_sequence_releases_once():
    phases = place_sequence((0.27, 0.0), S)
    assert [p[0] for p in phases] == ["carry", "lower", "release", "retreat"]
    assert [p[2] for p in phases] == [0.0, 0.0, 40.0, 40.0]
