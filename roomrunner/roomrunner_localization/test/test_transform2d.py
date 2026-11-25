import math

import numpy as np
import pytest

from roomrunner_localization.transform2d import compose, fit_rigid, invert, matrix, pose, transform_point, transform_pose


def test_matrix_pose_round_trip():
    assert pose(matrix(1.0, -2.0, 0.7)) == pytest.approx((1.0, -2.0, 0.7))


def test_invert():
    T = matrix(0.5, 1.5, -1.2)
    assert compose(T, invert(T)) == pytest.approx(np.eye(3))


def test_robot_pose_into_map():
    T_map_aruco = matrix(2.0, 1.0, math.pi / 2)        # aruco_map turned 90 deg, origin at (2, 1)
    x, y, yaw = transform_pose(T_map_aruco, (1.0, 0.0, 0.0))
    assert (x, y) == pytest.approx((2.0, 2.0)) and yaw == pytest.approx(math.pi / 2)


def test_point_into_map():
    assert transform_point(matrix(1.0, 0.0, math.pi), (1.0, 0.0)) == pytest.approx((0.0, 0.0))


def test_fit_rigid_recovers_transform():
    T = matrix(0.3, -0.8, 0.4)
    rng = np.random.default_rng(0)
    b = rng.uniform(-2, 2, (6, 2))
    a = [transform_point(T, p) for p in b]
    assert fit_rigid(a, b) == pytest.approx(T, abs=1e-9)


def test_fit_rigid_with_noise():
    T = matrix(1.0, 2.0, -0.3)
    rng = np.random.default_rng(1)
    b = rng.uniform(0, 3, (20, 2))
    a = np.array([transform_point(T, p) for p in b]) + rng.normal(0, 0.01, (20, 2))
    x, y, yaw = pose(fit_rigid(a, b))
    assert abs(x - 1.0) < 0.02 and abs(y - 2.0) < 0.02 and abs(yaw + 0.3) < 0.02
