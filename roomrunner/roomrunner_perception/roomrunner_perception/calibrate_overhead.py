"""One-time overhead camera calibration: floor homography from the anchor tags, and an empty-floor picture.

Before running: calibrate the intrinsics with the Assignment 04 checkerboard script and set intrinsics_file;
lay the 4 anchor tags (ids 30 to 33) flat on the floor and tape-measure their centres from below wall
marker 2 into `anchors` of overhead_camera.yaml.

    ros2 run roomrunner_perception calibrate_overhead --config <path to overhead_camera.yaml>

1. Clear the floor (no obstacles), press Enter: saves the empty-floor picture.
2. Make sure all 4 anchors are visible, press Enter: computes the homography and writes it back into the
   config with calibrated: true. With exactly 4 anchors the fit is exact on them, so the printout only
   confirms the ids; the real check is the robot-top tag against the particle filter (decisions.md, 11).
"""
import argparse
from pathlib import Path

import cv2
import numpy as np
import yaml

from roomrunner_perception.overhead_geometry import homography_from_anchors, pixel_to_floor
from roomrunner_perception.overhead_node import load_intrinsics


def grab(cfg, K, dist):
    camera = cv2.VideoCapture(int(cfg["device"]))
    camera.set(cv2.CAP_PROP_FRAME_WIDTH, cfg["width"])
    camera.set(cv2.CAP_PROP_FRAME_HEIGHT, cfg["height"])
    for _ in range(10):
        ok, frame = camera.read()
    camera.release()
    if not ok:
        raise SystemExit("No frame from the overhead camera: check device in the config.")
    return cv2.undistort(frame, K, dist) if K is not None else frame


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--config", required=True)
    args = parser.parse_args()
    path = Path(args.config)
    cfg = yaml.safe_load(path.read_text())
    K, dist = load_intrinsics(cfg.get("intrinsics_file", ""))

    input("Clear the floor of obstacles (anchors may stay), then press Enter...")
    empty_path = path.with_name("overhead_empty_floor.png")
    cv2.imwrite(str(empty_path), grab(cfg, K, dist))
    print(f"Saved {empty_path}")

    input("All 4 anchor tags visible? Press Enter...")
    frame = grab(cfg, K, dist)
    detector = cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, cfg["dictionary"])),
                                       cv2.aruco.DetectorParameters())
    corners, ids, _ = detector.detectMarkers(frame)
    seen = {} if ids is None else {int(i): c.reshape(4, 2).mean(axis=0) for c, i in zip(corners, ids.ravel())}
    anchors = {int(k): v for k, v in cfg["anchors"].items()}
    missing = sorted(set(anchors) - set(seen))
    if missing:
        raise SystemExit(f"Anchor tags not found: {missing}. Seen: {sorted(seen)}")
    ids_sorted = sorted(anchors)
    H = homography_from_anchors([seen[i] for i in ids_sorted], [anchors[i] for i in ids_sorted])
    for i in ids_sorted:
        x, y = pixel_to_floor(H, seen[i])
        print(f"anchor {i}: measured {anchors[i]}, from the image {x:.3f}, {y:.3f}")

    cfg.update(homography=np.asarray(H).tolist(), empty_floor_image=str(empty_path), calibrated=True)
    path.write_text(yaml.safe_dump(cfg, sort_keys=False))
    print(f"Wrote the homography to {path}")


if __name__ == "__main__":
    main()
