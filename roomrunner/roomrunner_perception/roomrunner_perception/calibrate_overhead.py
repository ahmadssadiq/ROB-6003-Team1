"""One-time overhead camera calibration: floor homography from the anchor tags, and an empty-floor picture.

Before running: calibrate the intrinsics with the Assignment 04 checkerboard script and set intrinsics_file;
lay the 4 anchor tags (ids 30 to 33) flat on the floor and tape-measure their centres from below wall
marker 2 into `anchors` of overhead_camera.yaml.

    ros2 run roomrunner_perception calibrate_overhead --config <path to overhead_camera.yaml>

1. Clear the floor (no obstacles), press Enter: saves the empty-floor picture.
2. Make sure all 4 anchors are visible, press Enter: computes the floor homography and, when the intrinsics
   are set, the camera pose (overhead_camera.py), and writes them back into the config with calibrated: true.

The homography fits 4 anchors exactly, so it cannot show a bad tape measurement. The camera pose can: it has
6 unknowns for 8 measured numbers, so a wrong anchor shows as a reprojection error of more than a few pixels
and as a camera height that does not match the tape. The final check is the robot-top tag against the
particle filter (decisions.md, 11).
"""
import argparse
from pathlib import Path

import cv2
import numpy as np
import yaml

from roomrunner_perception.overhead_camera import OverheadCamera, camera_pose_from_anchors
from roomrunner_perception.overhead_geometry import homography_from_anchors
from roomrunner_perception.overhead_camera import load_intrinsics
from roomrunner_perception.overhead_pipeline import make_detector


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
    seen = {i: c.mean(axis=0) for i, c in make_detector(cfg["dictionary"]).detect(frame).items()}
    anchors = {int(k): v for k, v in cfg["anchors"].items()}
    missing = sorted(set(anchors) - set(seen))
    if missing:
        raise SystemExit(f"Anchor tags not found: {missing}. Seen: {sorted(seen)}")
    ids_sorted = sorted(anchors)
    pixels, floor = [seen[i] for i in ids_sorted], [anchors[i] for i in ids_sorted]
    H = homography_from_anchors(pixels, floor)
    cfg.update(homography=np.asarray(H).tolist(), empty_floor_image=str(empty_path), calibrated=True)

    if K is None:
        print("No intrinsics_file: saved the floor homography only. Tag heights will be ignored.")
        cfg["camera_pose"] = {}
    else:
        pose = camera_pose_from_anchors(pixels, floor, K)
        camera = OverheadCamera(K=K, rvec=pose["rvec"], tvec=pose["tvec"])
        cx, cy, cz = pose["centre"]
        print(f"Camera at x {cx:.3f}, y {cy:.3f} m, height {cz:.3f} m (check the height with a tape)")
        print(f"Reprojection error {pose['reprojection_px']:.2f} px")
        for i, p, f in zip(ids_sorted, pixels, floor):
            x, y = camera.to_map(p)
            print(f"anchor {i}: measured {f}, from the image {x:.3f}, {y:.3f} "
                  f"({100 * np.hypot(x - f[0], y - f[1]):.1f} cm)")
        if pose["reprojection_px"] > 3.0:
            print("WARNING: more than 3 px. Re-measure the anchors (and check their ids) before trusting this.")
        cfg["camera_pose"] = {k: [float(v) for v in pose[k]] for k in ("rvec", "tvec", "centre")}
        cfg["camera_pose"]["reprojection_px"] = float(pose["reprojection_px"])
    path.write_text(yaml.safe_dump(cfg, sort_keys=False))
    print(f"Wrote the homography to {path}")


if __name__ == "__main__":
    main()
