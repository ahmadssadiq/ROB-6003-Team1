import math

import pytest

from roomrunner_localization.pose_check import (PoseHistory, PoseMonitor, T_to_yaml, average_pose,
                                                fit_T_map_aruco)
from roomrunner_localization.transform2d import matrix, pose, transform_pose

TRUE_T = matrix(0.12, -0.30, math.radians(4))          # a made-up T_map_aruco
SPOTS = [(0.5, 0.5, 0.0), (2.5, 0.4, 1.5), (2.4, 2.0, 3.0), (0.6, 1.8, -1.2)]   # robot poses in aruco_map


def test_average_pose_wraps_the_heading():
    assert average_pose([(0, 0, math.pi - 0.1), (2, 0, -math.pi + 0.1)]) == pytest.approx((1, 0, math.pi))


def test_fit_recovers_the_transform():
    seen = [transform_pose(TRUE_T, p) for p in SPOTS]
    T, errors = fit_T_map_aruco(seen, SPOTS)
    assert pose(T) == pytest.approx(pose(TRUE_T), abs=1e-9)
    assert max(e[0] for e in errors) < 1e-9 and max(abs(e[1]) for e in errors) < 1e-9


def test_fit_shows_a_bad_spot():
    seen = [transform_pose(TRUE_T, p) for p in SPOTS]
    seen[2] = (seen[2][0] + 0.2, seen[2][1], seen[2][2])
    _, errors = fit_T_map_aruco(seen, SPOTS)
    assert max(e[0] for e in errors) > 0.1


def test_fit_needs_three_spots():
    with pytest.raises(ValueError):
        fit_T_map_aruco(SPOTS[:2], SPOTS[:2])


def test_yaml_output():
    seen = [transform_pose(TRUE_T, p) for p in SPOTS]
    out = T_to_yaml(*fit_T_map_aruco(seen, SPOTS))
    assert out["measured"] and out["spots"] == 4
    assert out["T_map_aruco"]["x"] == pytest.approx(0.12) and out["T_map_aruco"]["yaw"] == pytest.approx(0.0698, abs=1e-4)


def test_history_nearest_and_still():
    h = PoseHistory()
    for i in range(11):
        h.add(i * 0.1, (0.0, 0.0, 0.0))                  # standing still for 1 s
    for i in range(11, 21):
        h.add(i * 0.1, (0.2 * (i - 10) * 0.1, 0.0, 0.0))  # then 0.2 m/s
    assert h.still(1.0) and not h.still(2.0)
    assert h.nearest(1.52)[0] == pytest.approx(0.02 * 5)
    assert not PoseHistory().still(1.0)


def test_monitor_ok_skip_and_reset():
    m = PoseMonitor()
    good, bad = (1.0, 1.0, 0.0), (1.6, 1.0, 0.0)
    assert m.check(0, good, (1.05, 1.0, 0.05)) == "ok"
    assert m.check(10, bad, good, still=False) == "skip"
    assert m.check(20, bad, good) == "disagree"
    assert m.check(30, bad, good) == "reset"
    assert m.check(40, bad, good) == "disagree"          # strikes start again
    assert m.check(50, bad, good) == "disagree"          # second strike, but within the 30 s cooldown
    assert m.check(70, bad, good) == "reset"


def test_monitor_heading_and_recovery():
    m = PoseMonitor()
    assert m.check(0, (1, 1, 0.0), (1, 1, math.radians(40))) == "disagree"
    assert m.check(10, (1, 1, 0.0), (1, 1, 0.0)) == "ok"     # back in agreement: strikes cleared
    assert m.check(20, (1, 1, 0.0), (1, 1, math.radians(40))) == "disagree"
