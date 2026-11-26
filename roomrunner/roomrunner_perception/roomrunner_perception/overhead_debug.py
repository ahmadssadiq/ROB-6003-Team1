"""Run the overhead pipeline once and save an annotated picture: every tag with its map pose, the robot's
heading, and the occupied grid cells. No ROS needed; use it while mounting and calibrating the camera.

    ros2 run roomrunner_perception overhead_debug --synthetic                 # made-up room, no camera
    ros2 run roomrunner_perception overhead_debug --image photo.png --config <overhead_camera.yaml>
    ros2 run roomrunner_perception overhead_debug --camera --config <overhead_camera.yaml>

--config and --objects default to the files in roomrunner_bringup/config of the source tree. --image and
--camera need a calibrated config (calibrate_overhead). Writes overhead_debug.png (--out to change).
Look for: tags in the right place (tape-measure one), obstacles red, the robot not red.
"""
import argparse
import math
from pathlib import Path

import cv2
import numpy as np
import yaml

from roomrunner_perception.overhead_camera import OverheadCamera
from roomrunner_perception.overhead_pipeline import make_detector, process_frame
from roomrunner_perception.synthetic import Cuboid, Tag, looking_down, render

CONFIG = Path(__file__).resolve().parents[2] / "roomrunner_bringup" / "config"


def middle_colour(spec):
    """BGR colour in the middle of a colour object's HSV range (hue ranges may wrap around 0)."""
    (h0, s0, v0), (h1, s1, v1) = spec["hsv_low"], spec["hsv_high"]
    hue = ((h0 + ((h1 - h0) % 180) / 2) % 180)
    hsv = np.uint8([[[hue, (s0 + s1) / 2, (v0 + v1) / 2]]])
    return tuple(int(c) for c in cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)[0, 0])


def synthetic_scene(objects_cfg):
    """A made-up room seen by a level camera 2.25 m up: every object of objects.yaml (tagged and colour
    cuboids, each cuboid drawn in the middle of its colour range), the robot and a box."""
    K, rvec, tvec = looking_down((1.5, 0.0, 2.25), tilt=0.03)
    tags = [Tag(int(tag_id), 2.6 - 0.6 * i, 0.9, z=float(info.get("height", 0.0)), yaw=0.3 * i, size=0.08)
            for i, (tag_id, info) in enumerate(objects_cfg["tags"].items())]
    cuboids = [Cuboid(0.5 + 0.5 * i, 0.45 - 0.3 * (i % 2), yaw=0.5 * i, length=spec["size"][0], width=spec["size"][1],
                      height=spec["height"], colour=middle_colour(spec))
               for i, spec in enumerate(objects_cfg.get("colour_objects", {}).values())]
    tags.append(Tag(int(objects_cfg["robot_tag"]), 0.6, -0.7, z=float(objects_cfg.get("robot_tag_height", 0.0)),
                    yaw=1.0, size=0.10))
    boxes = [(2.0, -1.0, 2.4, -0.7), (0.45, -0.85, 0.75, -0.55)]      # an obstacle; the robot's base
    camera = OverheadCamera(K=K, rvec=rvec, tvec=tvec)
    return render(K, rvec, tvec, tags=tags, boxes=boxes, cuboids=cuboids), render(K, rvec, tvec), camera


def draw(frame, result, camera, objects_cfg, grid_cfg):
    image = frame.copy()
    origin, res = grid_cfg["origin"], grid_cfg["resolution"]
    for row, col in zip(*np.nonzero(result["grid"] == 100)):
        u, v = camera.to_pixel((origin[0] + (col + 0.5) * res, origin[1] + (row + 0.5) * res))
        cv2.circle(image, (int(u), int(v)), 3, (0, 0, 255), -1)
    for o in result["objects"]:
        u, v = camera.to_pixel((o["x"], o["y"]), o["height"])
        label = f'{o["name"]} ({o["x"]:.2f}, {o["y"]:.2f}) {math.degrees(o["yaw"]):.0f} deg'
        cv2.circle(image, (int(u), int(v)), 6, (0, 200, 0), 2)
        cv2.putText(image, label, (int(u) + 10, int(v) - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 120, 0), 2)
    if result["robot"] is not None:
        x, y, yaw = result["robot"]
        z = float(objects_cfg.get("robot_tag_height", 0.0))
        start = camera.to_pixel((x, y), z)
        end = camera.to_pixel((x + 0.25 * math.cos(yaw), y + 0.25 * math.sin(yaw)), z)
        cv2.arrowedLine(image, tuple(map(int, start)), tuple(map(int, end)), (255, 80, 0), 3, tipLength=0.25)
        cv2.putText(image, f"robot ({x:.2f}, {y:.2f}) {math.degrees(yaw):.0f} deg",
                    (int(start[0]) + 10, int(start[1]) + 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 60, 0), 2)
    return image


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument("--synthetic", action="store_true", help="made-up room, no camera or calibration")
    source.add_argument("--image", help="a saved overhead photo")
    source.add_argument("--camera", action="store_true", help="grab one frame from the camera in --config")
    parser.add_argument("--config", default=str(CONFIG / "overhead_camera.yaml"))
    parser.add_argument("--objects", default=str(CONFIG / "objects.yaml"))
    parser.add_argument("--out", default="overhead_debug.png")
    args = parser.parse_args()
    cfg = yaml.safe_load(Path(args.config).read_text())
    objects_cfg = yaml.safe_load(Path(args.objects).read_text())

    if args.synthetic:
        frame, empty, camera = synthetic_scene(objects_cfg)
        grid_cfg = {"origin": [-0.5, -2.0], "width": 4.0, "height": 4.0, "resolution": 0.05}
    else:
        from roomrunner_perception.calibrate_overhead import grab          # needs a calibrated config
        from roomrunner_perception.overhead_node import camera_from_config, load_intrinsics
        if not cfg.get("calibrated"):
            raise SystemExit("The config is not calibrated yet: run calibrate_overhead first (or use --synthetic).")
        K, dist = load_intrinsics(cfg.get("intrinsics_file", ""))
        camera = camera_from_config(cfg, K)
        if args.camera:
            frame = grab(cfg, K, dist)
        else:
            frame = cv2.imread(args.image)
            if frame is None:
                raise SystemExit(f"Cannot read {args.image}")
            if K is not None:
                frame = cv2.undistort(frame, K, dist)
        empty = cv2.imread(cfg["empty_floor_image"]) if cfg.get("empty_floor_image") else None
        grid_cfg = cfg["grid"]

    result = process_frame(frame, camera, make_detector(cfg.get("dictionary", "DICT_4X4_50")), objects_cfg,
                           grid_cfg, empty)
    print(f"tags seen: {result['seen']}")
    for o in result["objects"]:
        print(f'  {o["name"]:12s} x {o["x"]:.3f}  y {o["y"]:.3f}  yaw {math.degrees(o["yaw"]):6.1f} deg')
    if result["robot"] is not None:
        x, y, yaw = result["robot"]
        print(f"  {'robot':12s} x {x:.3f}  y {y:.3f}  yaw {math.degrees(yaw):6.1f} deg")
    grid = result["grid"]
    print(f"grid {grid.shape[1]} x {grid.shape[0]} cells: {np.mean(grid == 100):.1%} occupied, "
          f"{np.mean(grid == -1):.1%} not seen")
    cv2.imwrite(args.out, draw(frame, result, camera, objects_cfg, grid_cfg))
    print(f"wrote {args.out}")


if __name__ == "__main__":
    main()
