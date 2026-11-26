"""Overhead camera model: image pixels -> map frame, at the floor or at any height above it.

Frames (docs/frames.md): map = the wall-marker frame, x and y on the floor, z up, metres. camera = OpenCV
camera frame (x right, y down, z along the optical axis). The camera pose is (rvec, tvec):
    X_camera = R X_map + t,   R = Rodrigues(rvec),   camera centre in the map frame C = -R^T t

Why a camera pose and not only the floor homography: a tag on top of a block or on the robot is above the
floor, so it appears shifted away from the point under the camera. The floor homography's error is
    tag height / (camera height - tag height) x (distance from the point under the camera)
which for a 0.30 m robot tag, a 2.25 m camera and 1.5 m off-centre is 0.23 m. With the pose, each pixel is
turned into a ray and cut with the plane z = tag height, which removes that error.

    pose = camera_pose_from_anchors(anchor_pixels, anchor_floor, K)    # calibration, once
    camera = OverheadCamera(K=K, rvec=pose["rvec"], tvec=pose["tvec"])  # or OverheadCamera(H=H) without intrinsics
    x, y = camera.to_map((u, v), z=0.05)
    x, y, yaw = camera.tag_pose(corners, z=0.05)

Pixels are undistorted pixels (the node undistorts each frame), so no distortion appears here.
"""
import math

import cv2
import numpy as np
import yaml


def camera_pose_from_anchors(anchor_pixels, anchor_floor, K):
    """Camera pose from at least 4 anchor tag centres lying on the floor (z = 0).

    Returns {"rvec", "tvec", "centre", "reprojection_px"}: centre is the camera position in the map frame
    (its z is the camera height), reprojection_px the RMS distance in pixels between the seen anchors and
    where the pose puts them. With 4 anchors there are 8 numbers for 6 unknowns, so it is a real check."""
    floor = np.asarray(anchor_floor, dtype=np.float64).reshape(-1, 2)
    pixels = np.asarray(anchor_pixels, dtype=np.float64).reshape(-1, 2)
    if len(floor) < 4:
        raise ValueError("need at least 4 anchors")
    points = np.hstack([floor, np.zeros((len(floor), 1))])
    ok, rvec, tvec = cv2.solvePnP(points, pixels, np.asarray(K, float), None, flags=cv2.SOLVEPNP_IPPE)
    if not ok:
        raise RuntimeError("solvePnP failed: check the anchor ids and positions")
    projected, _ = cv2.projectPoints(points, rvec, tvec, np.asarray(K, float), None)
    error = math.sqrt(np.mean(np.sum((projected.reshape(-1, 2) - pixels) ** 2, axis=1)))
    R, _ = cv2.Rodrigues(rvec)
    centre = (-R.T @ tvec).ravel()
    return {"rvec": rvec.ravel().tolist(), "tvec": tvec.ravel().tolist(), "centre": centre.tolist(),
            "reprojection_px": error}


class OverheadCamera:
    """Pixels -> map frame. Give K, rvec and tvec (exact at any height) or only H, the floor homography
    (pixels -> floor metres; heights are then ignored, which is fine for things lying on the floor)."""

    def __init__(self, H=None, K=None, rvec=None, tvec=None):
        self.has_pose = K is not None and rvec is not None and len(rvec) == 3
        if self.has_pose:
            self.K = np.asarray(K, float)
            self.R, _ = cv2.Rodrigues(np.asarray(rvec, float))
            self.t = np.asarray(tvec, float).reshape(3)
            self.centre = -self.R.T @ self.t
            # floor homography from the pose: pixel ~ K [r1 r2 t] [x y 1]
            self.H = np.linalg.inv(self.K @ np.column_stack([self.R[:, 0], self.R[:, 1], self.t]))
        elif H is not None and len(H):
            self.H = np.asarray(H, float)
        else:
            raise ValueError("need either the floor homography H or K with a camera pose")

    def to_map(self, pixel, z=0.0):
        """Map-frame (x, y) of the point at height z (m above the floor) seen at this pixel."""
        u, v = pixel
        if not self.has_pose:
            p = self.H @ np.array([u, v, 1.0])
            return float(p[0] / p[2]), float(p[1] / p[2])
        ray = self.R.T @ np.linalg.solve(self.K, np.array([u, v, 1.0]))   # viewing ray in the map frame
        s = (z - self.centre[2]) / ray[2]
        point = self.centre + s * ray
        return float(point[0]), float(point[1])

    def to_pixel(self, point, z=0.0):
        """Pixel where the map-frame point (x, y) at height z appears."""
        x, y = point
        if not self.has_pose:
            p = np.linalg.inv(self.H) @ np.array([x, y, 1.0])
        else:
            p = self.K @ (self.R @ np.array([x, y, z]) + self.t)
        return float(p[0] / p[2]), float(p[1] / p[2])

    def tag_pose(self, corners, z=0.0):
        """Pose (x, y, yaw) of a flat tag at height z from its 4 corner pixels in ArUco order (top-left,
        top-right, bottom-right, bottom-left of the printed tag). yaw is the direction of the tag's x axis
        (top-left -> top-right) in the map frame."""
        points = np.array([self.to_map(c, z) for c in np.asarray(corners).reshape(4, 2)])
        x, y = points.mean(axis=0)
        direction = (points[1] - points[0]) + (points[2] - points[3])   # both horizontal edges, averaged
        return float(x), float(y), math.atan2(direction[1], direction[0])


def load_intrinsics(path):
    """camera_matrix and distortion from an Assignment 04 style calibration file, or (None, None)."""
    if not path:
        return None, None
    with open(path) as f:
        data = yaml.safe_load(f)
    K = np.array(data["camera_matrix"]["data"], float).reshape(3, 3)
    return K, np.array(data["distortion_coefficients"]["data"], float)


def camera_from_config(cfg, K):
    """OverheadCamera from overhead_camera.yaml: the camera pose when calibration found one (needs the
    intrinsics), otherwise the floor homography."""
    pose = cfg.get("camera_pose") or {}
    if K is not None and pose.get("rvec"):
        return OverheadCamera(K=K, rvec=pose["rvec"], tvec=pose["tvec"])
    return OverheadCamera(H=np.array(cfg["homography"], float))
