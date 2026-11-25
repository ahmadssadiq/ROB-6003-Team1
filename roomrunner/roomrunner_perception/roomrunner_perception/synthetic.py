"""Made-up overhead pictures with ArUco tags and boxes, from a known camera. Used by the tests and by
`overhead_debug --synthetic` to try the whole pipeline without a camera.

    K, rvec, tvec = looking_down(centre=(1.5, 0.0, 2.25))       # camera 2.25 m above (1.5, 0) in the map frame
    image = render(K, rvec, tvec, tags=[Tag(10, 1.0, 0.5, z=0.05, yaw=0.3)], boxes=[(2.0, -1.0, 2.4, -0.6)])

Map frame: x, y on the floor, z up (metres, radians). The tag's x axis points from its printed top-left to
top-right corner and its y axis towards its top edge, the same convention as OverheadCamera.tag_pose.
"""
from dataclasses import dataclass

import cv2
import numpy as np

from roomrunner_perception.overhead_pipeline import marker_image

WIDTH, HEIGHT = 1920, 1080
FLOOR_GREY = 170
BOX_GREY = 60


@dataclass
class Tag:
    id: int
    x: float
    y: float
    z: float = 0.0          # height of the tag above the floor (m)
    yaw: float = 0.0        # rad
    size: float = 0.05      # printed black square side (m)


def looking_down(centre=(1.5, 0.0, 2.25), tilt=0.0, focal=1000.0, width=WIDTH, height=HEIGHT):
    """K, rvec, tvec of a camera at centre (map frame) looking straight down, image right = map +x and image
    up = map +y, optionally tilted by `tilt` rad about the image x axis (a camera that is not quite level)."""
    K = np.array([[focal, 0, width / 2], [0, focal, height / 2], [0, 0, 1]], float)
    down = np.diag([1.0, -1.0, -1.0])                            # map -> camera for a level, downward camera
    c, s = np.cos(tilt), np.sin(tilt)
    R = np.array([[1, 0, 0], [0, c, -s], [0, s, c]]) @ down
    t = -R @ np.asarray(centre, float)
    rvec, _ = cv2.Rodrigues(R)
    return K, rvec.ravel(), t


def project(K, rvec, tvec, points):
    pixels, _ = cv2.projectPoints(np.asarray(points, float).reshape(-1, 3), np.asarray(rvec, float),
                                  np.asarray(tvec, float), K, None)
    return pixels.reshape(-1, 2)


def tag_corners(tag, scale=1.0):
    """Map-frame 3D corners (TL, TR, BR, BL) of the tag's square, scaled about its centre."""
    h = tag.size / 2 * scale
    c, s = np.cos(tag.yaw), np.sin(tag.yaw)
    return [(tag.x + dx * c - dy * s, tag.y + dx * s + dy * c, tag.z)
            for dx, dy in [(-h, h), (h, h), (h, -h), (-h, -h)]]


def render(K, rvec, tvec, tags=(), boxes=(), width=WIDTH, height=HEIGHT, dictionary="DICT_4X4_50"):
    """Picture of a grey floor with dark boxes (floor rectangles [x_min, y_min, x_max, y_max]) and tags
    (each printed with a white border half its size wide)."""
    image = np.full((height, width, 3), FLOOR_GREY, np.uint8)
    for x0, y0, x1, y1 in boxes:
        corners = project(K, rvec, tvec, [(x0, y0, 0), (x1, y0, 0), (x1, y1, 0), (x0, y1, 0)])
        cv2.fillConvexPoly(image, np.round(corners).astype(np.int32), (BOX_GREY,) * 3)
    aruco = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, dictionary))
    side, border = 200, 50
    for tag in tags:
        printed = cv2.copyMakeBorder(marker_image(aruco, tag.id, side), border, border, border,
                                     border, cv2.BORDER_CONSTANT, value=255)
        n = side + 2 * border
        source = np.float32([(0, 0), (n, 0), (n, n), (0, n)])
        target = np.float32(project(K, rvec, tvec, tag_corners(tag, scale=n / side)))
        warp = cv2.getPerspectiveTransform(source, target)
        paper = cv2.warpPerspective(cv2.cvtColor(printed, cv2.COLOR_GRAY2BGR), warp, (width, height),
                                    flags=cv2.INTER_LINEAR)
        mask = cv2.warpPerspective(np.full((n, n), 255, np.uint8), warp, (width, height), flags=cv2.INTER_NEAREST)
        image[mask > 0] = paper[mask > 0]
    return image
