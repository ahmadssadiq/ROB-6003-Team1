"""Overhead camera geometry: image pixels <-> floor (map frame), object poses and the occupancy grid.
Plain NumPy and OpenCV, no ROS, so every step can be tested on made-up data.

The camera looks down at a flat floor, so a single 3x3 homography H maps (undistorted) pixels to floor
coordinates: [x, y, 1] ~ H [u, v, 1]. It is found once from four ArUco anchor tags lying on the floor whose
centres were tape-measured from wall marker 2, so the floor coordinates are the wall-marker map frame.

    H = homography_from_anchors(anchor_pixels, anchor_floor)      # calibration, once
    x, y = pixel_to_floor(H, (u, v))
    x, y, yaw = tag_pose_on_floor(H, corners)                     # a tag lying flat (blocks, tray)
    grid = occupancy_from_mask(obstacle_mask, H, origin, resolution, width, height)
"""
import math

import cv2
import numpy as np


def homography_from_anchors(anchor_pixels, anchor_floor):
    """H mapping pixels to floor metres, from at least 4 pixel <-> floor point pairs (anchor tag centres)."""
    src = np.asarray(anchor_pixels, dtype=np.float64).reshape(-1, 2)
    dst = np.asarray(anchor_floor, dtype=np.float64).reshape(-1, 2)
    if len(src) < 4:
        raise ValueError("need at least 4 anchors")
    H, _ = cv2.findHomography(src, dst, 0)
    return H


def pixel_to_floor(H, pixel):
    u, v = pixel
    p = H @ np.array([u, v, 1.0])
    return float(p[0] / p[2]), float(p[1] / p[2])


def floor_to_pixel(H, point):
    x, y = point
    p = np.linalg.inv(H) @ np.array([x, y, 1.0])
    return float(p[0] / p[2]), float(p[1] / p[2])


def tag_pose_on_floor(H, corners):
    """Pose (x, y, yaw) on the floor of a tag lying flat, from its 4 corner pixels in ArUco order
    (top-left, top-right, bottom-right, bottom-left of the printed tag). yaw is the direction of the tag's
    x axis (top-left -> top-right) in the map frame."""
    floor = np.array([pixel_to_floor(H, c) for c in np.asarray(corners).reshape(4, 2)])
    x, y = floor.mean(axis=0)
    direction = (floor[1] - floor[0]) + (floor[2] - floor[3])   # both horizontal edges, averaged
    return float(x), float(y), math.atan2(direction[1], direction[0])


def occupancy_from_mask(obstacle_mask, H, origin, resolution, width, height):
    """Occupancy grid (row = y, col = x; 0 free, 100 occupied, -1 not seen by the camera) from a binary image
    mask of obstacles (non-zero = obstacle) in camera pixels.

    Each grid cell's centre is projected into the image; the cell takes the mask value there."""
    rows, cols = round(height / resolution), round(width / resolution)
    xs = origin[0] + (np.arange(cols) + 0.5) * resolution
    ys = origin[1] + (np.arange(rows) + 0.5) * resolution
    gx, gy = np.meshgrid(xs, ys)
    floor = np.stack([gx.ravel(), gy.ravel(), np.ones(gx.size)])
    pix = np.linalg.inv(H) @ floor
    u, v = pix[0] / pix[2], pix[1] / pix[2]
    h, w = obstacle_mask.shape[:2]
    inside = (u >= 0) & (u < w) & (v >= 0) & (v < h)
    grid = np.full(gx.size, -1, dtype=np.int8)
    ui, vi = u[inside].astype(int), v[inside].astype(int)
    grid[inside] = np.where(obstacle_mask[vi, ui] > 0, 100, 0)
    return grid.reshape(rows, cols)


def obstacle_mask(image, empty_floor, threshold=40, min_area=200):
    """Obstacles as the pixels that differ from a picture of the empty floor (taken once at setup).
    Small blobs (noise, tags) under min_area pixels are dropped. Returns a uint8 mask, 255 = obstacle."""
    diff = cv2.absdiff(cv2.cvtColor(image, cv2.COLOR_BGR2GRAY), cv2.cvtColor(empty_floor, cv2.COLOR_BGR2GRAY))
    _, mask = cv2.threshold(cv2.GaussianBlur(diff, (5, 5), 0), threshold, 255, cv2.THRESH_BINARY)
    count, labels, stats, _ = cv2.connectedComponentsWithStats(mask)
    keep = np.zeros_like(mask)
    for i in range(1, count):
        if stats[i, cv2.CC_STAT_AREA] >= min_area:
            keep[labels == i] = 255
    return keep
