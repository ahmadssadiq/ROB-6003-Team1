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


from roomrunner_manipulation.grasp import arm_reached, free_slot, holding, unreachable  # noqa: E402


def fake_ik(max_x):
    """IK stand-in: reaches any target with x <= max_x, returns angles that encode the target."""
    def ik(target, direction, start):
        return [target[0], target[1], target[2], 0.0, 0.0], (0.0 if target[0] <= max_x else 0.05)
    return ik


def test_unreachable_names_the_bad_phases():
    phases = pick_sequence((0.25, 0.0), S)
    assert unreachable(phases, fake_ik(0.30), [0] * 5) == []
    assert [p for p, _ in unreachable(phases, fake_ik(0.20), [0] * 5)] == ["approach", "descend", "grasp", "lift"]


def test_unreachable_with_the_real_assignment_03_ik():
    kinematics = pytest.importorskip("arm_control.kinematics")
    start = [0.0, 0.0, 0.0, 0.0, 0.0]
    near = pick_sequence((0.20, 0.0), S)
    far = pick_sequence((0.60, 0.0), S)                  # well beyond the SO-101's reach
    assert unreachable(far, kinematics.inverse_kinematics, start)
    assert len(unreachable(near, kinematics.inverse_kinematics, start)) < len(near)


def test_arm_reached():
    assert arm_reached([10, 20, 30, 40, 0, 5], [11, 19, 31, 40, 2, 40])
    assert not arm_reached([10, 20, 30, 40, 0, 5], [15, 20, 30, 40, 0, 0])


def test_holding():
    s = dict(S, holding_min_gripper=5.0)
    assert holding(12.0, s) and not holding(1.5, s)


def test_free_slot_skips_occupied_and_turns_with_the_tray():
    slots = [(-0.04, 0.0), (0.04, 0.0)]
    assert free_slot((1.0, 1.0, 0.0), slots, [], 0.03) == pytest.approx((0.96, 1.0))
    assert free_slot((1.0, 1.0, 0.0), slots, [(0.96, 1.0)], 0.03) == pytest.approx((1.04, 1.0))
    assert free_slot((1.0, 1.0, math.pi / 2), slots, [], 0.03) == pytest.approx((1.0, 0.96))
    assert free_slot((1.0, 1.0, 0.0), slots, [(0.96, 1.0), (1.04, 1.0)], 0.03) is None


from roomrunner_manipulation.grasp import with_grasp_offset  # noqa: E402


def test_grasp_offset_turns_with_the_arm():
    shoulder = (0.067, 0.0)
    assert with_grasp_offset((0.267, 0.0), shoulder, (0.01, 0.02)) == pytest.approx((0.277, 0.02))
    # object to the robot's left: radial is +y, "sideways left" is -x
    assert with_grasp_offset((0.067, 0.2), shoulder, (0.01, 0.02)) == pytest.approx((0.047, 0.21))
    assert with_grasp_offset((0.2, 0.1), shoulder, (0.0, 0.0)) == (0.2, 0.1)
