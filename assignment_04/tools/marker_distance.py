"""D2: detected marker distance against tape-measured distance, on the real robot.

Run it while the localization launch file runs (localization:=false is enough):
    python ~/FOR26/ros_ws/tools/marker_distance.py d2_distance

For each spot it asks for the distance you measured with the tape measure along the floor, from
below the lens to below the marker's centre, then averages 2 s of /detected_markers. The detected
distance is the HORIZONTAL distance from the lens, so a marker higher or lower than the lens does
not make it longer (the camera's tilt, --pitch, is taken into account). Type q to finish. It writes
data/<run>/distance.csv and data/<run>/distance_accuracy.png. To plot again without the robot:
    python ~/FOR26/ros_ws/tools/marker_distance.py d2_distance --plot
"""
import argparse
import csv
import time
from pathlib import Path

import numpy as np

DATA = Path(__file__).resolve().parent.parent / "data"
BLUE, ORANGE, GREY = "#2a78d6", "#eb6834", "#8a8984"
AVERAGE_TIME = 2.0   # seconds of detections averaged at each spot


def horizontal_and_height(position, pitch_deg):
    """Marker centre relative to the lens: horizontal distance and height above the lens (m).

    position is the marker in the camera's optical frame (x right, y down, z forward). The camera
    is tilted up by pitch_deg, as passed to the launch file."""
    x, y, z = position
    forward, left, up = z, -x, -y                      # optical -> camera_link axes
    pitch = np.radians(pitch_deg)
    level_forward = np.cos(pitch) * forward - np.sin(pitch) * up
    level_up = np.sin(pitch) * forward + np.cos(pitch) * up
    return float(np.hypot(level_forward, left)), float(level_up)


def record(path, pitch_deg, marker_id, append):
    import rclpy
    from aruco_interfaces.msg import ArucoMarkerArray

    rclpy.init()
    node = rclpy.create_node("marker_distance")
    seen = []   # (time, id, horizontal distance, height above lens) of every detection of marker_id
    node.create_subscription(ArucoMarkerArray, "detected_markers", lambda msg: seen.extend(
        (time.monotonic(), m.id, *horizontal_and_height(
            [m.pose.position.x, m.pose.position.y, m.pose.position.z], pitch_deg))
        for m in msg.markers if marker_id is None or m.id == marker_id), 10)

    rows = []
    if append and path.exists():
        with open(path) as f:
            rows = list(csv.reader(f))[1:]
        print(f"Adding to the {len(rows)} readings already in {path}")
    while True:
        answer = input("\nTape-measured floor distance in metres (below the lens to below the marker), or q to finish: ").strip()
        if answer.lower() == "q":
            break
        try:
            true_distance = float(answer)
        except ValueError:
            print("Type a number such as 1.5, or q.")
            continue
        start = time.monotonic()
        while time.monotonic() - start < AVERAGE_TIME:
            rclpy.spin_once(node, timeout_sec=0.05)
        now = [d for t, i, d, h in seen if t >= start]
        heights = [h for t, i, d, h in seen if t >= start]
        ids = sorted({i for t, i, d, h in seen if t >= start})
        if not now:
            print(f"  No marker detected at {true_distance} m. Saved as 'not detected'.")
            rows.append([true_distance, "", "", 0, ""])
            continue
        mean, std = float(np.mean(now)), float(np.std(now))
        print(f"  detected {mean:.3f} m (std {std * 1000:.1f} mm, {len(now)} detections, id {ids}), "
              f"error {(mean - true_distance) * 1000:+.0f} mm; marker centre {np.mean(heights) * 100:+.1f} cm above the lens")
        rows.append([true_distance, f"{mean:.4f}", f"{std:.4f}", len(now), " ".join(map(str, ids))])

    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "w", newline="") as f:
        csv.writer(f).writerows([["true_m", "detected_m", "std_m", "count", "ids"]] + rows)
    print(f"\nWrote {path}")
    node.destroy_node()
    rclpy.shutdown()


def plot(path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    with open(path) as f:
        rows = list(csv.DictReader(f))
    found = [r for r in rows if r["detected_m"]]
    detected_at = {float(r["true_m"]) for r in rows if r["detected_m"]}
    missed = sorted({float(r["true_m"]) for r in rows if not r["detected_m"]} - detected_at)
    true = np.array([float(r["true_m"]) for r in found])
    detected = np.array([float(r["detected_m"]) for r in found])
    std = np.array([float(r["std_m"]) for r in found])
    error = (detected - true) * 1000

    for t, d, e in zip(true, detected, error):
        print(f"true {t:.2f} m  detected {d:.3f} m  error {e:+.0f} mm ({100 * e / (1000 * t):+.1f} %)")
    if len(true) >= 2:
        slope, offset = np.polyfit(true, detected - true, 1)
        print(f"error fit: {offset * 1000:+.1f} mm + {slope * 100:+.2f} % of the distance")
    if missed:
        print(f"not detected at: {missed} m")

    fig, (left, right) = plt.subplots(1, 2, figsize=(9, 3.6))
    top = max(true.max(), detected.max()) * 1.08
    left.plot([0, top], [0, top], color=GREY, linewidth=1, linestyle=(0, (4, 3)), label="perfect")
    left.errorbar(true, detected, yerr=std, fmt="o", color=BLUE, capsize=3, label="detected")
    left.set_xlabel("tape-measured distance (m)")
    left.set_ylabel("detected distance (m)")
    left.set_xlim(0, top)
    left.set_ylim(0, top)
    left.set_title("Detected against measured distance")
    left.legend(loc="upper left", frameon=False)
    right.axhline(0, color=GREY, linewidth=1)
    right.plot(true, error, "o", color=ORANGE, label="measured")
    if len(true) >= 2:
        fit_x = np.linspace(0, true.max(), 50)
        right.plot(fit_x, (offset + slope * fit_x) * 1000, color=ORANGE, linewidth=1, linestyle=(0, (4, 3)),
                   label=f"fit: {offset * 1000:+.0f} mm {slope * 100:+.1f} % of distance")
        right.legend(loc="upper left", frameon=False)
    right.set_xlabel("tape-measured distance (m)")
    right.set_ylabel("detected - measured (mm)")
    right.set_title("Distance error")
    for ax in (left, right):
        ax.grid(True, color="#ecebe7")
        ax.spines[["top", "right"]].set_visible(False)
    fig.tight_layout()
    out = path.with_name("distance_accuracy.png")
    fig.savefig(out, dpi=300)
    print("wrote", out)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("run", help="folder name under ros_ws/data/")
    parser.add_argument("--plot", action="store_true", help="only plot an existing recording")
    parser.add_argument("--pitch", type=float, default=12.0, help="camera tilt up (deg), as in the launch file")
    parser.add_argument("--id", type=int, help="only use this marker id (others in view are ignored)")
    parser.add_argument("--append", action="store_true", help="add readings to the existing file instead of starting over")
    args = parser.parse_args()
    csv_path = DATA / args.run / "distance.csv"
    if not args.plot:
        record(csv_path, args.pitch, args.id, args.append)
    plot(csv_path)
