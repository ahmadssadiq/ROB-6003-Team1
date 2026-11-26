"""One overhead frame -> object poses, the robot pose and the occupancy grid. Plain Python (no ROS), so the
whole pipeline can be tested on synthetic pictures (synthetic.py) and tried on saved photos (overhead_debug).

    camera = OverheadCamera(...)                       # overhead_camera.py
    result = process_frame(frame, camera, detector, objects_cfg, grid_cfg, empty_floor)
    result["objects"]   [{"name", "kind", "tag_id", "x", "y", "yaw"}, ...]   map frame, m and rad
    result["robot"]     (x, y, yaw) of the robot-top tag, or None
    result["grid"]      int8 array, row = y, col = x: 0 free, 100 occupied, -1 not seen
    result["seen"]      sorted ids of every tag in the picture

objects_cfg is objects.yaml; grid_cfg is the grid section of overhead_camera.yaml.
"""
import math

import cv2
import numpy as np

from roomrunner_perception.overhead_geometry import obstacle_mask, occupancy_from_mask


class Detector:
    """ArUco detector that works with OpenCV 4.6 (Ubuntu 24.04's python3-opencv) and with 4.7 and newer,
    whose ArUco API changed. Sub-pixel corners, as in the Assignment 04 detector."""

    def __init__(self, dictionary="DICT_4X4_50"):
        aruco = cv2.aruco
        self.dictionary = aruco.getPredefinedDictionary(getattr(aruco, dictionary))
        new_api = hasattr(aruco, "ArucoDetector")
        self.parameters = aruco.DetectorParameters() if new_api else aruco.DetectorParameters_create()
        self.parameters.cornerRefinementMethod = aruco.CORNER_REFINE_SUBPIX
        self.detector = aruco.ArucoDetector(self.dictionary, self.parameters) if new_api else None

    def detect(self, frame):
        """{tag id: 4x2 corner pixels} of every tag in the frame."""
        if self.detector is not None:
            corners, ids, _ = self.detector.detectMarkers(frame)
        else:
            corners, ids, _ = cv2.aruco.detectMarkers(frame, self.dictionary, parameters=self.parameters)
        return {} if ids is None else {int(i): c.reshape(4, 2) for c, i in zip(corners, ids.ravel())}


def make_detector(dictionary="DICT_4X4_50"):
    return Detector(dictionary)


def marker_image(dictionary, tag_id, side):
    """Printable tag image (side x side pixels) with either OpenCV API."""
    aruco = cv2.aruco
    if hasattr(aruco, "generateImageMarker"):
        return aruco.generateImageMarker(dictionary, tag_id, side)
    return aruco.drawMarker(dictionary, tag_id, side)


def clear_disc(grid, origin, resolution, x, y, radius):
    """Mark the cells within radius (m) of map point (x, y) as free. Used for the robot's own footprint: the
    robot is not in the empty-floor picture, so without this it would be an obstacle on its own start."""
    rows, cols = grid.shape
    ys = origin[1] + (np.arange(rows) + 0.5) * resolution
    xs = origin[0] + (np.arange(cols) + 0.5) * resolution
    gx, gy = np.meshgrid(xs, ys)
    grid[(gx - x) ** 2 + (gy - y) ** 2 <= radius ** 2] = 0
    return grid


def process_frame(frame, camera, detector, objects_cfg, grid_cfg, empty_floor=None):
    tags = {int(k): v for k, v in objects_cfg["tags"].items()}
    robot_tag = int(objects_cfg["robot_tag"])
    seen = detector.detect(frame)

    objects, robot = [], None
    for tag_id, corners in sorted(seen.items()):
        if tag_id == robot_tag:
            robot = camera.tag_pose(corners, float(objects_cfg.get("robot_tag_height", 0.0)))
        elif tag_id in tags:
            info = tags[tag_id]
            x, y, yaw = camera.tag_pose(corners, float(info.get("height", 0.0)))
            objects.append({"name": info["name"], "kind": info["kind"], "tag_id": tag_id, "x": x, "y": y,
                            "yaw": yaw})

    if empty_floor is not None:
        mask = obstacle_mask(frame, empty_floor)
    else:
        mask = np.zeros(frame.shape[:2], np.uint8)
    origin, resolution = grid_cfg["origin"], grid_cfg["resolution"]
    grid = occupancy_from_mask(mask, camera.H, origin, resolution, grid_cfg["width"], grid_cfg["height"])
    if robot is not None:
        clear_disc(grid, origin, resolution, robot[0], robot[1], float(objects_cfg.get("robot_clear_radius", 0.0)))
    return {"objects": objects, "robot": robot, "grid": grid, "seen": sorted(seen)}


def yaw_to_quaternion_zw(yaw):
    """(z, w) of the quaternion for a rotation of yaw about z."""
    return math.sin(yaw / 2), math.cos(yaw / 2)
