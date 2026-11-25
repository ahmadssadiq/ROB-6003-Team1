"""Planar rigid transforms (x, y, yaw) as 3x3 homogeneous matrices (plain NumPy, no ROS).

A pose (x, y, yaw) of frame B in frame A is the matrix T_A_B; a point p_B in B is T_A_B @ [x, y, 1] in A.
Used for p_map = T_map_aruco p_aruco: the robot pose from the particle filter (aruco_map frame) into the
planning map (overhead camera frame).
"""
import math

import numpy as np


def matrix(x, y, yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, -s, x], [s, c, y], [0.0, 0.0, 1.0]])


def pose(T):
    """Matrix -> (x, y, yaw)."""
    return float(T[0, 2]), float(T[1, 2]), math.atan2(T[1, 0], T[0, 0])


def compose(T_a_b, T_b_c):
    return T_a_b @ T_b_c


def invert(T):
    R, t = T[:2, :2], T[:2, 2]
    out = np.eye(3)
    out[:2, :2] = R.T
    out[:2, 2] = -R.T @ t
    return out


def transform_pose(T_a_b, pose_b):
    """A pose (x, y, yaw) given in frame b, expressed in frame a."""
    return pose(T_a_b @ matrix(*pose_b))


def transform_point(T_a_b, point_b):
    x, y = point_b
    p = T_a_b @ np.array([x, y, 1.0])
    return float(p[0]), float(p[1])


def fit_rigid(points_a, points_b):
    """Least-squares rigid transform T_a_b with points_a ~ T_a_b points_b (2-D Kabsch / Umeyama, no scale).
    Use it to measure T_map_aruco from robot positions seen in both frames (overhead camera and PF)."""
    A, B = np.asarray(points_a, float), np.asarray(points_b, float)
    if len(A) < 2:
        raise ValueError("need at least two point pairs")
    ca, cb = A.mean(axis=0), B.mean(axis=0)
    H = (B - cb).T @ (A - ca)
    U, _, Vt = np.linalg.svd(H)
    D = np.diag([1.0, np.sign(np.linalg.det(Vt.T @ U.T))])
    R = Vt.T @ D @ U.T
    T = np.eye(3)
    T[:2, :2] = R
    T[:2, 2] = ca - R @ cb
    return T
