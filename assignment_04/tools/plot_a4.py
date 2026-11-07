"""Plots for the Assignment 4 report, from the CSV files of the course's recording scripts.

Usage (conda environment ros_mujoco):
    python ~/FOR26/ros_ws/tools/plot_a4.py l1 a4_l1_sim     localization error over time, PF and odometry
    python ~/FOR26/ros_ws/tools/plot_a4.py l2 a4_l2_sim     marker distance from its map position, against distance

The PNG files are written next to the CSV files, in ~/FOR26/ros_ws/data/<run>/.
"""
import csv
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

DATA = Path(__file__).resolve().parent.parent / "data"
BLUE, ORANGE, GREY, INK_2 = "#2a78d6", "#eb6834", "#8a8984", "#52514e"

plt.rcParams.update({
    "figure.dpi": 150, "savefig.dpi": 300, "font.size": 10, "axes.titlesize": 10.5,
    "axes.edgecolor": "#c9c8c2", "axes.labelcolor": INK_2, "xtick.color": INK_2, "ytick.color": INK_2,
    "axes.grid": True, "axes.axisbelow": True, "grid.color": "#ecebe7",
    "axes.spines.top": False, "axes.spines.right": False, "legend.frameon": False,
})


def wrap(angle):
    return (angle + np.pi) % (2.0 * np.pi) - np.pi


def load_localization(path):
    """A record_localization.py CSV: time and the errors of the estimate and of odometry against the truth."""
    with open(path) as f:
        rows = [r for r in csv.DictReader(f) if r["true_x"] and r["odom_x"]]
    col = lambda name: np.array([float(r[name]) for r in rows])
    t = col("time") - col("time")[0]
    true_x, true_y, true_theta = col("true_x"), col("true_y"), col("true_theta")
    return {
        "t": t,
        "pf_pos": np.hypot(col("est_x") - true_x, col("est_y") - true_y),
        "pf_yaw": np.degrees(np.abs(wrap(col("est_theta") - true_theta))),
        # The odom frame starts where the robot started, which is the map origin in these runs.
        "odom_pos": np.hypot(col("odom_x") - true_x, col("odom_y") - true_y),
        "odom_yaw": np.degrees(np.abs(wrap(col("odom_theta") - true_theta))),
    }


def summary(name, values, unit):
    return f"{name}: mean {values.mean():.1f} {unit}, max {values.max():.1f} {unit}, final {values[-1]:.1f} {unit}"


def plot_l1(run):
    """Simulation, square path: position and heading error over time, PF (unknown and known start) and odometry."""
    unknown = load_localization(DATA / run / "unknown_start.csv")
    known = load_localization(DATA / run / "known_start.csv")
    for label, d in (("unknown start", unknown), ("known start", known)):
        print(f"PF, {label}:   " + summary("position", 100 * d["pf_pos"], "cm") + ";  " + summary("yaw", d["pf_yaw"], "deg"))
        above = np.flatnonzero(d["pf_pos"] > 0.2)
        if len(above) == 0:
            print("   error below 20 cm from the start")
        elif above[-1] + 1 < len(d["t"]):
            print(f"   error stays below 20 cm after {d['t'][above[-1] + 1]:.1f} s")
        else:
            print("   error never settles below 20 cm")
    print("Odometry:          " + summary("position", 100 * known["odom_pos"], "cm") + ";  "
          + summary("yaw", known["odom_yaw"], "deg"))

    fig, (top, bottom) = plt.subplots(2, 1, figsize=(6.5, 5), sharex=True)
    for ax, key, scale, unit in ((top, "pos", 100, "position error (cm)"), (bottom, "yaw", 1, "yaw error (deg)")):
        ax.plot(unknown["t"], unknown["pf_" + key] * scale, color=BLUE, linewidth=1.4, label="PF, unknown start")
        ax.plot(known["t"], known["pf_" + key] * scale, color=ORANGE, linewidth=1.4, label="PF, known start")
        ax.plot(known["t"], known["odom_" + key] * scale, color=GREY, linewidth=1.4, linestyle=(0, (4, 2)),
                label="odometry only")
        ax.set_ylabel(unit)
        ax.set_ylim(bottom=0)
    top.set_title("Simulation, 1.5 m square driven twice: localization error over time")
    top.legend(loc="upper right")
    bottom.set_xlabel("time (s)")
    fig.tight_layout()
    out = DATA / run / "l1_error_over_time.png"
    fig.savefig(out)
    print("wrote", out)


def plot_l2(run):
    """Marker consistency: how far each detected marker lands from its map position, against distance."""
    label = "Real robot" if "real" in run else "Simulation"
    with open(DATA / run / "marker_errors.csv") as f:
        rows = list(csv.DictReader(f))
    distance = np.array([float(r["range"]) for r in rows])
    error = np.array([float(r["error_xy"]) for r in rows]) * 100
    ids = np.array([int(r["id"]) for r in rows])
    print(f"{len(rows)} sightings of {len(set(ids))} markers; error median {np.median(error):.1f} cm, "
          f"90th percentile {np.percentile(error, 90):.1f} cm")
    for marker in sorted(set(ids)):
        e = error[ids == marker]
        print(f"  marker {marker}: {len(e)} sightings, median {np.median(e):.1f} cm, "
              f"at {distance[ids == marker].min():.1f}-{distance[ids == marker].max():.1f} m")

    edges = np.arange(np.floor(distance.min() * 2) / 2, distance.max() + 0.5, 0.5)
    centres, medians = [], []
    for low, high in zip(edges[:-1], edges[1:]):
        inside = (distance >= low) & (distance < high)
        if inside.sum() >= 5:
            centres.append((low + high) / 2)
            medians.append(np.median(error[inside]))
            print(f"  {low:.1f}-{high:.1f} m: median {medians[-1]:.1f} cm ({inside.sum()} sightings)")

    fig, ax = plt.subplots(figsize=(6, 3.6))
    ax.scatter(distance, error, s=6, color=BLUE, alpha=0.25, linewidths=0, label="each sighting")
    ax.plot(centres, medians, "o-", color=ORANGE, linewidth=1.8, label="median per 0.5 m")
    ax.set_xlabel("distance from robot to marker (m)")
    ax.set_ylabel("detected - map position (cm)")
    ax.set_ylim(bottom=0)
    ax.set_title(f"{label}: detected markers against their map positions")
    ax.legend(loc="upper left")
    fig.tight_layout()
    out = DATA / run / "l2_marker_consistency.png"
    fig.savefig(out)
    print("wrote", out)


if __name__ == "__main__":
    commands = {"l1": plot_l1, "l2": plot_l2}
    if len(sys.argv) != 3 or sys.argv[1] not in commands:
        sys.exit(__doc__)
    commands[sys.argv[1]](sys.argv[2])
