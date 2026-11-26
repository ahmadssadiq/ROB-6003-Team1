"""The simulated room's configuration and the exact overhead camera model (plain Python, no MuJoCo or ROS).

The simulator knows its overhead camera exactly, so instead of calibrating it, sim.launch.py writes the files
the real system gets from calibrate_overhead: camera intrinsics, overhead_camera.yaml (camera pose, homography,
grid) and an objects.yaml with the simulated robot's tag.

    room = load_room()                                   # config/sim_room.yaml
    K, rvec, tvec = overhead_model(room)                 # OpenCV camera (X_camera = R X_map + t)
    paths = write_configs(room, objects_file, places_file, out_dir)   # {"intrinsics", "overhead", "objects", "places"}

Frames: map = the simulator's world (z up, metres). The MuJoCo overhead camera has no rotation, so it looks
straight down with the picture's right along map +x and the picture's top along map +y; in OpenCV terms
(x right, y down, z forward) that is R = diag(1, -1, -1).
"""
import math
from pathlib import Path

import numpy as np
import yaml

PACKAGE = Path(__file__).resolve().parents[1]          # the source tree (tests); the launch file uses the share dir
ROOM = PACKAGE / "config" / "sim_room.yaml"
PLACES = PACKAGE / "config" / "places_sim.yaml"


def load_room(path=ROOM):
    return yaml.safe_load(Path(path).read_text())


def overhead_model(room):
    """K, rvec, tvec of the simulated overhead camera (a perfect pinhole camera, as the course sim_node
    reports for its own cameras: focal length from the vertical field of view)."""
    c = room["overhead_camera"]
    f = (c["height"] / 2) / math.tan(math.radians(c["fovy_deg"]) / 2)
    K = np.array([[f, 0, c["width"] / 2], [0, f, c["height"] / 2], [0, 0, 1]])
    R = np.diag([1.0, -1.0, -1.0])
    t = -R @ np.asarray(c["position"], float)
    rvec = np.array([math.pi, 0.0, 0.0])                # Rodrigues of diag(1, -1, -1): half a turn about x
    return K, rvec, t


def floor_seen(room):
    """Floor size (m) the overhead camera sees: (along x, along y)."""
    c = room["overhead_camera"]
    half_y = c["position"][2] * math.tan(math.radians(c["fovy_deg"]) / 2)
    return 2 * half_y * c["width"] / c["height"], 2 * half_y


def floor_homography(K, rvec, tvec):
    """Pixels -> floor (z = 0) homography of a camera pose: the inverse of K [r1 r2 t]."""
    import cv2
    R, _ = cv2.Rodrigues(np.asarray(rvec, float))
    return np.linalg.inv(K @ np.column_stack([R[:, 0], R[:, 1], np.asarray(tvec, float)]))


def write_configs(room, objects_file, places_file, out_dir):
    """Write the sim's intrinsics, overhead_camera.yaml, objects.yaml and places.yaml into out_dir. The
    overhead config names out_dir/overhead_empty_floor.png, which room_sim writes when it starts."""
    out = Path(out_dir).expanduser()
    out.mkdir(parents=True, exist_ok=True)
    K, rvec, tvec = overhead_model(room)
    c = room["overhead_camera"]
    intrinsics = {"image_width": c["width"], "image_height": c["height"],
                  "camera_matrix": {"rows": 3, "cols": 3, "data": K.ravel().tolist()},
                  "distortion_coefficients": {"rows": 1, "cols": 5, "data": [0.0] * 5}}
    paths = {name: out / f"{name}.yaml" for name in ("intrinsics", "overhead", "objects", "places")}
    paths["intrinsics"].write_text(yaml.safe_dump(intrinsics, sort_keys=False))
    overhead = {"device": "sim", "width": c["width"], "height": c["height"],
                "intrinsics_file": str(paths["intrinsics"]), "dictionary": "DICT_4X4_50", "anchors": {},
                "homography": floor_homography(K, rvec, tvec).tolist(),
                "camera_pose": {"rvec": rvec.tolist(), "tvec": tvec.tolist(), "centre": list(c["position"]),
                                "reprojection_px": 0.0},
                "empty_floor_image": str(out / "overhead_empty_floor.png"), "grid": room["grid"], "capture_period": 2.0, "calibrated": True}
    paths["overhead"].write_text("# Written by roomrunner_sim (exact simulated camera); do not edit.\n"
                                 + yaml.safe_dump(overhead, sort_keys=False))
    objects = yaml.safe_load(Path(objects_file).read_text())
    tag = room["robot_tag"]
    objects.update(robot_tag=tag["id"], robot_tag_height=tag["height"], robot_tag_offset=tag["offset"],
                   robot_clear_radius=room["robot_clear_radius"])
    objects["tags"] = {room["tray"]["tag_id"]: {"name": "tray", "kind": "tray",
                                               "height": room["tray"]["size"][2] + 0.002}}
    paths["objects"].write_text("# Written by roomrunner_sim from objects.yaml and sim_room.yaml.\n"
                                + yaml.safe_dump(objects, sort_keys=False))
    paths["places"].write_text(Path(places_file).read_text())
    return paths
