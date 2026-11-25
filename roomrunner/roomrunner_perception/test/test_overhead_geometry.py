import math

import numpy as np
import pytest

from roomrunner_perception.overhead_geometry import (floor_to_pixel, homography_from_anchors, obstacle_mask,
                                                     occupancy_from_mask, pixel_to_floor, tag_pose_on_floor)

# A made-up camera: 200 px per metre, floor point (0, 0) at pixel (100, 500), image y pointing to map -y.
def true_pixel(x, y):
    return 100 + 200 * x, 500 - 200 * y


ANCHORS_FLOOR = [(0.0, 0.0), (2.0, 0.0), (2.0, 1.5), (0.0, 1.5)]
ANCHORS_PIXELS = [true_pixel(*p) for p in ANCHORS_FLOOR]


def test_homography_maps_anchors_and_other_points():
    H = homography_from_anchors(ANCHORS_PIXELS, ANCHORS_FLOOR)
    assert pixel_to_floor(H, true_pixel(1.23, 0.45)) == pytest.approx((1.23, 0.45), abs=1e-6)
    assert floor_to_pixel(H, (1.0, 1.0)) == pytest.approx(true_pixel(1.0, 1.0), abs=1e-6)


def test_needs_four_anchors():
    with pytest.raises(ValueError):
        homography_from_anchors(ANCHORS_PIXELS[:3], ANCHORS_FLOOR[:3])


def test_tag_pose():
    H = homography_from_anchors(ANCHORS_PIXELS, ANCHORS_FLOOR)
    yaw, half = 0.6, 0.025
    cx, cy = 1.0, 0.8
    corners_floor = []
    for dx, dy in [(-half, half), (half, half), (half, -half), (-half, -half)]:   # TL, TR, BR, BL in the tag frame
        corners_floor.append((cx + dx * math.cos(yaw) - dy * math.sin(yaw), cy + dx * math.sin(yaw) + dy * math.cos(yaw)))
    x, y, theta = tag_pose_on_floor(H, [true_pixel(*c) for c in corners_floor])
    assert (x, y) == pytest.approx((cx, cy), abs=1e-6) and theta == pytest.approx(yaw, abs=1e-6)


def test_occupancy_grid_from_mask():
    H = homography_from_anchors(ANCHORS_PIXELS, ANCHORS_FLOOR)
    mask = np.zeros((600, 600), np.uint8)
    u0, v1 = true_pixel(0.5, 0.5)
    u1, v0 = true_pixel(1.0, 1.0)
    mask[int(v0):int(v1), int(u0):int(u1)] = 255          # a 0.5 m box from (0.5, 0.5) to (1.0, 1.0)
    grid = occupancy_from_mask(mask, H, origin=(0.0, 0.0), resolution=0.1, width=2.0, height=1.5)
    assert grid.shape == (15, 20)
    assert grid[7, 7] == 100          # cell centre (0.75, 0.75): in the box
    assert grid[2, 2] == 0            # (0.25, 0.25): free floor
    assert (grid == 100).sum() == 25  # 5 x 5 cells


def test_cells_outside_the_image_are_unknown():
    H = homography_from_anchors(ANCHORS_PIXELS, ANCHORS_FLOOR)
    grid = occupancy_from_mask(np.zeros((100, 100), np.uint8), H, (0.0, 0.0), 0.1, 2.0, 1.5)
    assert grid[14, 19] == -1


def test_obstacle_mask_from_background():
    empty = np.full((100, 100, 3), 120, np.uint8)
    scene = empty.copy()
    scene[20:60, 30:70] = 20          # a dark box, 40 x 40 px
    scene[90, 90] = 255               # one noisy pixel, dropped
    mask = obstacle_mask(scene, empty)
    assert mask[40, 50] == 255 and mask[90, 90] == 0
