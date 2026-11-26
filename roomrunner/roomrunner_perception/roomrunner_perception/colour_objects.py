"""Objects without tags, found by colour in the overhead picture (plain NumPy and OpenCV, no ROS).

For each object of objects.yaml's colour_objects:
    1. pixels inside its HSV range (and, with an empty-floor picture, different from the empty floor),
    2. blobs (contours); each one is projected onto the plane of the object's top face (z = height) with
       OverheadCamera, so its area and centre are in metres in the map frame wherever it is in the picture,
    3. away from the point under the camera it also sees the side of the cuboid facing away, so the blob is
       bigger than the top face and its centre lies a little outward. From a first estimate, the outline the
       camera should see (top face plus visible sides, `silhouette`) is predicted; its centre shift is taken
       off and the blob's area is compared with the predicted one,
    4. the blob whose area is closest to the prediction (within min_area..max_area of it) wins,
    5. yaw = direction of the blob's long side (cv2.minAreaRect), in [0, 180) deg: a cuboid looks the same
       turned by half a turn (a square one every 90 deg).

    found = find_colour_objects(frame_bgr, camera, objects_cfg["colour_objects"], foreground)
    # [{"name", "kind", "tag_id": -1, "x", "y", "yaw", "height", "confidence"}]

Units: metres and radians in the map frame; hue 0..179, saturation and value 0..255 (OpenCV HSV).
"""
import math

import cv2
import numpy as np


def footprint_area(spec):
    """Area (m^2) of the cuboid seen from above; spec["size"] = [length, width] in metres."""
    length, width = spec["size"]
    return length * width


def symmetry(spec):
    """Angle (rad) after which the cuboid looks the same from above: half a turn, a quarter for a square one."""
    length, width = spec["size"]
    return math.pi / 2 if abs(length - width) < 0.1 * max(length, width) else math.pi


def colour_mask(hsv, low, high):
    """Binary mask of pixels within [low, high] (hue, saturation, value). A hue range with low > high wraps
    around 0 (for reds, e.g. low hue 170, high hue 10)."""
    low, high = np.asarray(low, np.uint8), np.asarray(high, np.uint8)
    if low[0] <= high[0]:
        return cv2.inRange(hsv, low, high)
    first = cv2.inRange(hsv, np.array([0, low[1], low[2]], np.uint8), high)
    second = cv2.inRange(hsv, low, np.array([179, high[1], high[2]], np.uint8))
    return first | second


def foreground_mask(image, empty_floor, threshold=30):
    """Pixels that differ from the empty-floor picture (255), without dropping small blobs."""
    diff = cv2.absdiff(cv2.cvtColor(image, cv2.COLOR_BGR2GRAY), cv2.cvtColor(empty_floor, cv2.COLOR_BGR2GRAY))
    colour_diff = cv2.absdiff(image, empty_floor).max(axis=2)
    return np.where((diff > threshold) | (colour_diff > threshold), 255, 0).astype(np.uint8)


def polygon_area_centroid(points):
    """Area and centroid of a polygon (shoelace formula); points as an N x 2 array."""
    x, y = points[:, 0], points[:, 1]
    x1, y1 = np.roll(x, -1), np.roll(y, -1)
    cross = x * y1 - x1 * y
    area = cross.sum() / 2
    if abs(area) < 1e-12:
        return 0.0, (float(x.mean()), float(y.mean()))
    return abs(area), (float(((x + x1) * cross).sum() / (6 * area)), float(((y + y1) * cross).sum() / (6 * area)))


def long_side_yaw(points, period):
    """Direction of the long side of the smallest rectangle around the points, reduced to [0, period)."""
    (_, _), (w, h), angle = cv2.minAreaRect(points.astype(np.float32))
    yaw = math.radians(angle) if w >= h else math.radians(angle) + math.pi / 2
    return yaw % period


def cuboid_corners(x, y, yaw, spec, z):
    length, width = spec["size"]
    c, s, a, b = math.cos(yaw), math.sin(yaw), length / 2, width / 2
    return [(x + dx * c - dy * s, y + dx * s + dy * c, z) for dx, dy in [(-a, -b), (a, -b), (a, b), (-a, b)]]


def silhouette(camera, x, y, yaw, spec):
    """What the camera sees of a cuboid at (x, y, yaw), drawn on the plane of its top face: the outline of its
    top and bottom corners together. Returns (area m^2, centre shift (dx, dy) from (x, y))."""
    height = float(spec["height"])
    corners = cuboid_corners(x, y, yaw, spec, height) + cuboid_corners(x, y, yaw, spec, 0.0)
    on_top = np.array([camera.to_map(camera.to_pixel(p[:2], p[2]), height) for p in corners], np.float32)
    hull = cv2.convexHull(on_top).reshape(-1, 2).astype(float)
    area, (cx, cy) = polygon_area_centroid(hull)
    return area, (cx - x, cy - y)


def find_colour_objects(frame_bgr, camera, specs, foreground=None, min_area=0.5, max_area=1.8):
    """Every colour object found in the picture (at most one per name). specs: objects.yaml colour_objects."""
    hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
    kernel = np.ones((3, 3), np.uint8)
    found = []
    for name, spec in specs.items():
        mask = colour_mask(hsv, spec["hsv_low"], spec["hsv_high"])
        if foreground is not None:
            mask &= foreground
        mask = cv2.morphologyEx(cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel), cv2.MORPH_CLOSE, kernel)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        expected, height = footprint_area(spec), float(spec["height"])
        best = None
        for contour in contours:
            if len(contour) < 3:
                continue
            points = np.array([camera.to_map(p, height) for p in contour.reshape(-1, 2)])
            area, (x, y) = polygon_area_centroid(points)
            if not min_area <= area / expected <= 2 * max_area:       # far off before the correction: skip
                continue
            yaw = long_side_yaw(points, symmetry(spec))
            seen_area, (dx, dy) = silhouette(camera, x, y, yaw, spec)
            ratio = area / seen_area
            if not min_area <= ratio <= max_area:
                continue
            score = abs(math.log(ratio))
            if best is None or score < best[0]:
                best = (score, x - dx, y - dy, yaw, ratio)
        if best is not None:
            score, x, y, yaw, ratio = best
            found.append({"name": name, "kind": spec.get("kind", "block"), "tag_id": -1, "x": x, "y": y,
                          "yaw": yaw, "height": height, "confidence": round(math.exp(-2 * score), 3)})
    return found


def suggest_hsv_range(hsv_pixels, hue_margin=8, min_saturation=60, min_value=50):
    """An HSV range around sampled pixels of one object (N x 3 array), for objects.yaml: the median hue
    plus or minus a margin (wider if the samples spread), saturation and value from their low percentiles."""
    p = np.asarray(hsv_pixels, float).reshape(-1, 3)
    hue = p[:, 0]
    # hue is circular (0..179): centre the samples on their circular mean before taking the spread
    angle = hue * 2 * math.pi / 180
    centre = (math.degrees(math.atan2(np.sin(angle).mean(), np.cos(angle).mean())) / 2) % 180
    offset = (hue - centre + 90) % 180 - 90
    margin = max(hue_margin, float(np.percentile(np.abs(offset), 95)) + 3)
    low_s = max(min_saturation, int(np.percentile(p[:, 1], 5) * 0.7))
    low_v = max(min_value, int(np.percentile(p[:, 2], 5) * 0.6))
    low_h, high_h = int(round(centre - margin)) % 180, int(round(centre + margin)) % 180
    return [low_h, low_s, low_v], [high_h, 255, 255]
