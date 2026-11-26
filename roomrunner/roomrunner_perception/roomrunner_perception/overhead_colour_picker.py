"""Set the colour cuboids' HSV ranges under the real lighting: drag a box inside each cuboid's top face.

    ros2 run roomrunner_perception overhead_colour_picker --camera          # one frame from the overhead camera
    ros2 run roomrunner_perception overhead_colour_picker --image photo.png
    add --write to put the ranges into objects.yaml (only the hsv_low / hsv_high of those lines change)

For each cuboid of objects.yaml (colour_objects) a window asks for a box: drag it inside the top face (not the
floor around it), press Enter; press c to skip a cuboid that is not in the picture. It prints the suggested
ranges, warns when two ranges overlap in hue (those colours would be confused), and saves a check picture
(overhead_colours.png) with what the detector finds with the new ranges. Needs a display (system python3 has
OpenCV with windows; the conda one may not).
"""
import argparse
import re
from pathlib import Path

import cv2
import yaml

from roomrunner_perception.colour_objects import find_colour_objects, suggest_hsv_range

CONFIG = Path(__file__).resolve().parents[2] / "roomrunner_bringup" / "config"


def hue_overlap(a, b):
    """True if two hue ranges [low, high] (each may wrap around 0) share any hue."""
    def hues(r):
        low, high = r
        return set(range(low, high + 1)) if low <= high else set(range(low, 180)) | set(range(0, high + 1))
    return bool(hues(a) & hues(b))


def write_ranges(path, ranges):
    """Replace hsv_low / hsv_high on each cuboid's line of objects.yaml, keeping everything else."""
    text = Path(path).read_text()
    for name, (low, high) in ranges.items():
        pattern = rf"(^\s*{name}:.*?hsv_low: )\[[^\]]*\](, hsv_high: )\[[^\]]*\]"
        text, n = re.subn(pattern, rf"\g<1>{low}\g<2>{high}", text, flags=re.M)
        if n != 1:
            raise SystemExit(f"could not find the line of {name} in {path}")
    Path(path).write_text(text)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument("--image")
    source.add_argument("--camera", action="store_true")
    parser.add_argument("--config", default=str(CONFIG / "overhead_camera.yaml"))
    parser.add_argument("--objects", default=str(CONFIG / "objects.yaml"))
    parser.add_argument("--write", action="store_true", help="write the ranges into --objects")
    args = parser.parse_args()
    objects_cfg = yaml.safe_load(Path(args.objects).read_text())
    cfg = yaml.safe_load(Path(args.config).read_text())

    from roomrunner_perception.overhead_node import camera_from_config, load_intrinsics
    K, dist = load_intrinsics(cfg.get("intrinsics_file", ""))
    if args.camera:
        from roomrunner_perception.calibrate_overhead import grab
        frame = grab(cfg, K, dist)
    else:
        frame = cv2.imread(args.image)
        if frame is None:
            raise SystemExit(f"Cannot read {args.image}")
        if K is not None:
            frame = cv2.undistort(frame, K, dist)
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    specs, ranges = objects_cfg["colour_objects"], {}
    for name in specs:
        x, y, w, h = cv2.selectROI(f"{name}: box inside its top face, Enter (c = not here)", frame, False)
        cv2.destroyAllWindows()
        if w < 2 or h < 2:
            print(f"{name}: skipped")
            continue
        low, high = suggest_hsv_range(hsv[y:y + h, x:x + w].reshape(-1, 3))
        ranges[name] = (low, high)
        specs[name].update(hsv_low=low, hsv_high=high)
        print(f"{name}: hsv_low: {low}, hsv_high: {high}")
    names = list(ranges)
    for i, a in enumerate(names):
        for b in names[i + 1:]:
            if hue_overlap((ranges[a][0][0], ranges[a][1][0]), (ranges[b][0][0], ranges[b][1][0])):
                print(f"WARNING: {a} and {b} overlap in hue: they may be confused; use colours further apart")

    if cfg.get("calibrated"):
        camera = camera_from_config(cfg, K)
        found = find_colour_objects(frame, camera, specs)
        check = frame.copy()
        for o in found:
            u, v = camera.to_pixel((o["x"], o["y"]), o["height"])
            cv2.circle(check, (int(u), int(v)), 8, (255, 255, 255), 2)
            cv2.putText(check, f'{o["name"]} {o["confidence"]:.2f}', (int(u) + 10, int(v)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        cv2.imwrite("overhead_colours.png", check)
        print(f"found with these ranges: {[o['name'] for o in found]} (overhead_colours.png)")
    else:
        print("The overhead camera is not calibrated yet: ranges only, no detection check.")
    if args.write and ranges:
        write_ranges(args.objects, ranges)
        print(f"wrote the ranges into {args.objects}")


if __name__ == "__main__":
    main()
