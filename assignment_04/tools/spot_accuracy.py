"""L3: the real robot's estimated position against tape-measured spots on the classroom floor.

Run it while the localization runs on the real robot:
    python ~/FOR26/ros_ws/tools/spot_accuracy.py l3_real

Drive the robot onto a taped spot (its base centre over the tape) and let it stand still. Then type the
spot's map coordinates, measured with a tape measure from marker 2: x out of marker 2's wall into the
room, y along that wall (to the right when you face marker 2 from the room). The tool averages 10 s of
/pose_estimate. Type q to finish. It writes data/<run>/spots.csv and data/<run>/spot_accuracy.png.
To plot again: python ~/FOR26/ros_ws/tools/spot_accuracy.py l3_real --plot
"""
import argparse
import csv
import math
import time
from pathlib import Path

import numpy as np

DATA = Path(__file__).resolve().parent.parent / "data"
MAP = Path(__file__).resolve().parent.parent / "src/aruco_localization/config/classroom_map.yaml"
BLUE, ORANGE, GREY, INK = "#2a78d6", "#eb6834", "#8a8984", "#0b0b0b"
AVERAGE_TIME = 10.0


def record(path, append):
    import rclpy
    from geometry_msgs.msg import PoseWithCovarianceStamped

    rclpy.init()
    node = rclpy.create_node("spot_accuracy")
    poses = []   # (time, x, y, yaw)

    def on_pose(msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        poses.append((time.monotonic(), p.x, p.y, yaw))

    node.create_subscription(PoseWithCovarianceStamped, "pose_estimate", on_pose, 10)
    rows = []
    if append and path.exists():
        with open(path) as f:
            rows = list(csv.reader(f))[1:]
        print(f"Adding to the {len(rows)} spots already in {path}")
    while True:
        answer = input(f"\nSpot {len(rows) + 1}: tape-measured map x y in metres (e.g. 1.20 -0.50), or q: ").strip()
        if answer.lower() == "q":
            break
        try:
            x, y = (float(v) for v in answer.replace(",", " ").split())
        except ValueError:
            print("Type two numbers, x then y, such as 1.20 -0.50")
            continue
        print(f"  Averaging {AVERAGE_TIME:.0f} s of /pose_estimate. Keep the robot still...")
        start = time.monotonic()
        while time.monotonic() - start < AVERAGE_TIME:
            rclpy.spin_once(node, timeout_sec=0.1)
        now = np.array([(px, py, pyaw) for t, px, py, pyaw in poses if t >= start])
        if len(now) == 0:
            print("  No /pose_estimate received. Is the localization running? Not saved.")
            continue
        ex, ey = now[:, 0].mean(), now[:, 1].mean()
        eyaw = math.degrees(math.atan2(np.sin(now[:, 2]).mean(), np.cos(now[:, 2]).mean()))
        sx, sy = now[:, 0].std(), now[:, 1].std()
        error = math.hypot(ex - x, ey - y)
        print(f"  estimate ({ex:.3f}, {ey:.3f}), heading {eyaw:.0f} deg, wobble {100 * max(sx, sy):.1f} cm; "
              f"error {100 * error:.1f} cm (x {100 * (ex - x):+.1f}, y {100 * (ey - y):+.1f})")
        rows.append([f"{x:.3f}", f"{y:.3f}", f"{ex:.4f}", f"{ey:.4f}", f"{eyaw:.1f}", f"{sx:.4f}", f"{sy:.4f}",
                     f"{error:.4f}", len(now)])
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "w", newline="") as f:
        csv.writer(f).writerows([["true_x", "true_y", "est_x", "est_y", "est_yaw_deg", "std_x", "std_y", "error_m",
                                  "count"]] + rows)
    print(f"\nWrote {path}")
    node.destroy_node()
    rclpy.shutdown()


def plot(path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    import yaml

    with open(path) as f:
        rows = list(csv.DictReader(f))
    true = np.array([[float(r["true_x"]), float(r["true_y"])] for r in rows])
    est = np.array([[float(r["est_x"]), float(r["est_y"])] for r in rows])
    error = np.linalg.norm(est - true, axis=1) * 100
    print("spot  measured (m)        estimated (m)       error (cm)")
    for i, (t, e, err) in enumerate(zip(true, est, error), 1):
        print(f"{i:4d}  ({t[0]:6.2f}, {t[1]:6.2f})    ({e[0]:6.3f}, {e[1]:6.3f})    {err:5.1f}")
    print(f"mean error {error.mean():.1f} cm, largest {error.max():.1f} cm")

    markers = yaml.safe_load(open(MAP))["markers"]
    fig, ax = plt.subplots(figsize=(6.5, 5))
    ax.scatter([m["position"][0] for m in markers], [m["position"][1] for m in markers], marker="s", s=60,
               color=GREY, label="map markers")
    for m in markers:
        ax.annotate(f"marker {m['id']}", m["position"][:2], textcoords="offset points", xytext=(6, 4),
                    fontsize=8, color=GREY)
    ax.scatter(true[:, 0], true[:, 1], s=70, facecolors="none", edgecolors=BLUE, linewidths=1.8,
               label="tape-measured spot")
    ax.scatter(est[:, 0], est[:, 1], marker="x", s=60, color=ORANGE, linewidths=2, label="estimated position")
    for i, (t, e, err) in enumerate(zip(true, est, error), 1):
        ax.plot([t[0], e[0]], [t[1], e[1]], color=ORANGE, linewidth=1)
        ax.annotate(f"spot {i}: {err:.0f} cm", t, textcoords="offset points", xytext=(8, 8 if i % 2 else -14),
                    fontsize=9, color=INK)
    # Zoom on the spots and the markers near them; far markers (marker 8) would squeeze the plot.
    near = np.vstack([true, est] + [[m["position"][:2]] for m in markers
                                    if np.min(np.linalg.norm(true - m["position"][:2], axis=1)) < 3.0])
    ax.set_xlim(near[:, 0].min() - 0.4, near[:, 0].max() + 0.6)
    ax.set_ylim(near[:, 1].min() - 0.4, near[:, 1].max() + 0.4)
    ax.set_aspect("equal")
    ax.set_xlabel("map x (m)")
    ax.set_ylabel("map y (m)")
    ax.set_title("Real robot: estimated against tape-measured position")
    ax.grid(True, color="#ecebe7")
    ax.spines[["top", "right"]].set_visible(False)
    ax.legend(loc="best", frameon=False, fontsize=8)
    fig.tight_layout()
    out = path.with_name("spot_accuracy.png")
    fig.savefig(out, dpi=300)
    print("wrote", out)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("run", help="folder name under ros_ws/data/")
    parser.add_argument("--plot", action="store_true", help="only plot an existing recording")
    parser.add_argument("--append", action="store_true", help="add spots to the existing file")
    args = parser.parse_args()
    csv_path = DATA / args.run / "spots.csv"
    if not args.plot:
        record(csv_path, args.append)
    plot(csv_path)
