import math

import cv2
import numpy as np
import pytest

from roomrunner_perception.overhead_camera import OverheadCamera, camera_pose_from_anchors
from roomrunner_perception.overhead_geometry import homography_from_anchors
from roomrunner_perception.synthetic import looking_down, project

ANCHORS = [(0.5, -0.5), (2.5, -0.5), (2.5, 1.0), (0.5, 1.0)]
CENTRE = (1.4, 0.2, 2.25)


def camera_and_anchor_pixels(tilt=0.05):
    K, rvec, tvec = looking_down(CENTRE, tilt=tilt)
    pixels = project(K, rvec, tvec, [(x, y, 0.0) for x, y in ANCHORS])
    return K, rvec, tvec, pixels


def test_pose_from_anchors_finds_the_camera():
    K, _, _, pixels = camera_and_anchor_pixels()
    pose = camera_pose_from_anchors(pixels, ANCHORS, K)
    assert pose["centre"] == pytest.approx(CENTRE, abs=1e-6)
    assert pose["reprojection_px"] < 1e-6


def test_bad_anchor_measurement_shows_in_the_reprojection_error():
    K, _, _, pixels = camera_and_anchor_pixels()
    wrong = list(ANCHORS)
    wrong[2] = (2.6, 1.0)                    # one anchor tape-measured 10 cm off
    assert camera_pose_from_anchors(pixels, wrong, K)["reprojection_px"] > 2.0


def test_floor_and_raised_points_round_trip():
    K, rvec, tvec, _ = camera_and_anchor_pixels()
    camera = OverheadCamera(K=K, rvec=rvec, tvec=tvec)
    for z in (0.0, 0.05, 0.30):
        pixel = project(K, rvec, tvec, [(2.8, -1.2, z)])[0]
        assert camera.to_map(pixel, z) == pytest.approx((2.8, -1.2), abs=1e-6)
        assert camera.to_pixel((2.8, -1.2), z) == pytest.approx(tuple(pixel), abs=1e-6)


def test_floor_homography_misplaces_a_raised_tag():
    """The reason for the camera pose: the floor homography puts a 0.30 m high tag 1.5 m off-centre about
    0.2 m too far out; the pose gets it right."""
    K, rvec, tvec, pixels = camera_and_anchor_pixels(tilt=0.0)
    floor_only = OverheadCamera(H=homography_from_anchors(pixels, ANCHORS))
    with_pose = OverheadCamera(K=K, rvec=rvec, tvec=tvec)
    point = (CENTRE[0] + 1.5, CENTRE[1])
    pixel = project(K, rvec, tvec, [(*point, 0.30)])[0]
    x_floor, _ = floor_only.to_map(pixel)
    assert x_floor - point[0] == pytest.approx(1.5 * 0.30 / (2.25 - 0.30), abs=1e-3)   # 0.23 m too far
    assert with_pose.to_map(pixel, 0.30) == pytest.approx(point, abs=1e-6)


def test_pose_and_homography_agree_on_the_floor():
    K, rvec, tvec, pixels = camera_and_anchor_pixels()
    a = OverheadCamera(K=K, rvec=rvec, tvec=tvec)
    b = OverheadCamera(H=homography_from_anchors(pixels, ANCHORS))
    assert a.to_map((700, 300)) == pytest.approx(b.to_map((700, 300)), abs=1e-6)


def test_tag_pose_at_height():
    K, rvec, tvec, _ = camera_and_anchor_pixels()
    camera = OverheadCamera(K=K, rvec=rvec, tvec=tvec)
    x, y, z, yaw, h = 2.0, 0.6, 0.05, -0.8, 0.025
    corners = [(x + dx * math.cos(yaw) - dy * math.sin(yaw), y + dx * math.sin(yaw) + dy * math.cos(yaw), z)
               for dx, dy in [(-h, h), (h, h), (h, -h), (-h, -h)]]
    pose = camera.tag_pose(project(K, rvec, tvec, corners), z)
    assert pose == pytest.approx((x, y, yaw), abs=1e-6)


def test_needs_a_model():
    with pytest.raises(ValueError):
        OverheadCamera()
    with pytest.raises(ValueError):
        camera_pose_from_anchors(np.zeros((3, 2)), ANCHORS[:3], np.eye(3))


def test_rodrigues_of_a_level_camera_points_down():
    _, rvec, _ = looking_down(CENTRE)
    R, _ = cv2.Rodrigues(np.asarray(rvec))
    assert R.T @ np.array([0, 0, 1.0]) == pytest.approx([0, 0, -1.0])   # optical axis = map -z
