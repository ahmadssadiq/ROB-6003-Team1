"""Make the report's plots from the CSV files that record.py writes.

These follow the "Measure how well it works" section of Kinematics/03_keyboard_typing.md.

Usage (with the system Python, which has matplotlib):
    /usr/bin/python3 ~/FOR26/ros_ws/tools/plot.py keys  sim_typing      position, model and tracking error
    /usr/bin/python3 ~/FOR26/ros_ws/tools/plot.py fk    sim_fk_validation  FK against the true fingertip
    /usr/bin/python3 ~/FOR26/ros_ws/tools/plot.py speed sim_speed       keys pressed against move time
    /usr/bin/python3 ~/FOR26/ros_ws/tools/plot.py real  real_pool_1     fingertip height while typing

The PNG files are written next to the CSV files, in ~/FOR26/ros_ws/data/<run>/.
"""
import csv
import re
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

DATA = Path(__file__).resolve().parent.parent / "data"
BLUE, ORANGE = "#2a78d6", "#eb6834"
GREY, INK, INK_2 = "#8a8984", "#0b0b0b", "#52514e"

# The simulated keyboard, as in keyboard_typer.py: top of each key, and how deep the typer aims.
SIM_KEYS = {"a": [0.300, 0.070, -0.012], "s": [0.300, 0.000, -0.012], "d": [0.300, -0.070, -0.012]}
PRESS_DEPTH = 0.008

plt.rcParams.update({
    "figure.dpi": 150, "savefig.dpi": 300, "font.size": 10, "axes.titlesize": 10.5,
    "axes.edgecolor": "#c9c8c2", "axes.labelcolor": INK_2, "xtick.color": INK_2,
    "ytick.color": INK_2, "axes.grid": True, "axes.axisbelow": True, "grid.color": "#ecebe7",
    "axes.spines.top": False, "axes.spines.right": False, "legend.frameon": False,
})


def load(run, topic):
    """One recorded topic as a dict of columns, or None if nothing was recorded."""
    path = DATA / run / f"{topic}.csv"
    if not path.exists():
        return None
    with open(path) as f:
        rows = list(csv.reader(f))
    if len(rows) < 2:
        return None
    columns = {}
    for i, name in enumerate(rows[0]):
        values = [r[i] for r in rows[1:]]
        try:
            columns[name] = np.array(values, dtype=float)
        except ValueError:
            columns[name] = values
    return columns


def position_at(d, t):
    """The x, y, z in recording `d` closest in time to `t`."""
    i = int(np.argmin(np.abs(d["t"] - t)))
    return np.array([d["x"][i], d["y"][i], d["z"][i]])


def label_bars(ax, bars, unit="", decimals=1):
    for bar in bars:
        ax.text(bar.get_x() + bar.get_width() / 2, bar.get_height(), f"{bar.get_height():.{decimals}f}{unit}",
                ha="center", va="bottom", color=INK, fontsize=9)


def save(fig, run, name):
    path = DATA / run / name
    fig.savefig(path, bbox_inches="tight")
    plt.close(fig)
    print("wrote", path)


def plot_keys(run):
    """Simulation: for each key, the errors at the moment the finger is deepest on the key."""
    truth, fk, presses = load(run, "true_fingertip"), load(run, "fk_pose"), load(run, "key_pressed")
    keys, position_err, model_err, tracking_err = [], [], [], []
    for t_press, name in zip(presses["t"], presses["key"]):
        key = name.replace("key_", "")
        # The deepest point of the finger within 1.5 s of the press event.
        near = np.abs(truth["t"] - t_press) < 1.5
        t_deep = truth["t"][near][np.argmin(truth["z"][near])]
        target = np.array(SIM_KEYS[key]) - [0.0, 0.0, PRESS_DEPTH]
        true_tip, fk_tip = position_at(truth, t_deep), position_at(fk, t_deep)
        keys.append(key)
        position_err.append(np.linalg.norm(true_tip - target) * 1000)
        model_err.append(np.linalg.norm(true_tip - fk_tip) * 1000)
        tracking_err.append(np.linalg.norm(fk_tip - target) * 1000)
        print(f"key {key}: target {np.round(target, 3)}, real fingertip {np.round(true_tip, 3)}, "
              f"FK {np.round(fk_tip, 3)} -> position error {position_err[-1]:.1f} mm "
              f"(x {1000 * (true_tip - target)[0]:+.1f}, y {1000 * (true_tip - target)[1]:+.1f}, "
              f"z {1000 * (true_tip - target)[2]:+.1f}), model {model_err[-1]:.2f} mm, "
              f"tracking {tracking_err[-1]:.1f} mm")

    fig, ax = plt.subplots(figsize=(5, 3.2))
    label_bars(ax, ax.bar(keys, position_err, width=0.5, color=BLUE), " mm")
    ax.set_xlabel("key")
    ax.set_ylabel("position error (mm)")
    ax.set_title("Simulation: fingertip distance from the press target")
    ax.set_ylim(0, max(position_err) * 1.25)
    save(fig, run, "position_error.png")

    fig, ax = plt.subplots(figsize=(5.5, 3.2))
    x = np.arange(len(keys))
    label_bars(ax, ax.bar(x - 0.18, model_err, width=0.34, color=BLUE, label="model error  |true - FK|"), decimals=2)
    label_bars(ax, ax.bar(x + 0.18, tracking_err, width=0.34, color=ORANGE, label="tracking error  |FK - target|"))
    ax.set_xticks(x, keys)
    ax.set_xlabel("key")
    ax.set_ylabel("error (mm)")
    ax.set_title("Simulation: model error and tracking error")
    ax.set_ylim(0, max(tracking_err) * 1.4)
    ax.legend(loc="upper left")
    save(fig, run, "model_vs_tracking.png")


def plot_fk(run):
    """Simulation: forward kinematics against the simulator's true fingertip while the arm moves."""
    truth, fk = load(run, "true_fingertip"), load(run, "fk_pose")
    # The two topics arrive at slightly different times: compare FK at the true fingertip's times.
    fk_at = np.stack([np.interp(truth["t"], fk["t"], fk[k]) for k in "xyz"], axis=1)
    true_at = np.stack([truth[k] for k in "xyz"], axis=1)
    error = np.linalg.norm(fk_at - true_at, axis=1) * 1000
    t = truth["t"] - truth["t"][0]
    print(f"model error over {len(t)} samples: median {np.median(error):.3f} mm, mean {error.mean():.3f} mm, "
          f"max {error.max():.2f} mm; fingertip range (mm) {np.round((true_at.max(0) - true_at.min(0)) * 1000)}")

    fig, (top, bottom) = plt.subplots(2, 1, figsize=(6, 4.2), sharex=True, height_ratios=[1.6, 1])
    for i, (name, colour) in enumerate(zip("xyz", (BLUE, ORANGE, GREY))):
        top.plot(t, true_at[:, i] * 100, color=colour, linewidth=2.2, label=f"{name} true")
        top.plot(t, fk_at[:, i] * 100, color=INK, linewidth=0.8, linestyle=(0, (3, 2)))
    top.plot([], [], color=INK, linewidth=0.8, linestyle=(0, (3, 2)), label="FK")
    top.set_ylabel("fingertip (cm)")
    top.legend(loc="lower left", ncol=4, fontsize=8)
    top.set_title("Simulation: forward kinematics against the true fingertip")
    bottom.plot(t, error, color=BLUE, linewidth=1)
    bottom.set_ylabel("|true - FK| (mm)")
    bottom.set_xlabel("time (s)")
    save(fig, run, "fk_validation.png")


def plot_speed(run):
    """Simulation: how many of the three keys registered, for each move time."""
    with open(DATA / run / "results.csv") as f:
        rows = list(csv.DictReader(f))
    times = sorted({float(r["move_time"]) for r in rows}, reverse=True)
    pressed = [sum(int(r["pressed"]) for r in rows if float(r["move_time"]) == t) for t in times]
    for t, p in zip(times, pressed):
        print(f"move time {t} s: {p} of 3 keys pressed")

    fig, ax = plt.subplots(figsize=(5.5, 3.2))
    bars = ax.bar([f"{t:g}" for t in times], pressed, width=0.55,
                  color=[BLUE if p == 3 else ORANGE for p in pressed])
    for bar, p in zip(bars, pressed):
        ax.text(bar.get_x() + bar.get_width() / 2, p, f"{p}/3", ha="center", va="bottom", color=INK, fontsize=9)
    ax.set_xlabel("time for each move (s), faster to the right")
    ax.set_ylabel("keys pressed (of 3)")
    ax.set_yticks([0, 1, 2, 3])
    ax.set_ylim(0, 3.6)
    ax.set_title("Simulation: keys registered against typing speed")
    save(fig, run, "speed_vs_success.png")


def ee_positions():
    """The named positions in ros_ws/ee, for example {"p": [x, y, z], "home": [...]}."""
    names = {}
    for name, numbers in re.findall(r'(\w+)\)\s+xyz="([^"]+)"', (DATA.parent / "ee").read_text()):
        names[name] = np.array([float(v) for v in numbers.split()])
    return names


def plot_real(run):
    """Real robot: fingertip height (from FK) while typing, with each command marked."""
    fk, targets = load(run, "fk_pose"), load(run, "ik_target")
    names = ee_positions()
    t0 = targets["t"][0] - 2.0
    keep = (fk["t"] >= t0) & (fk["t"] <= targets["t"][-1] + 6.0)

    fig, ax = plt.subplots(figsize=(7.5, 3.4))
    ax.plot(fk["t"][keep] - t0, fk["z"][keep] * 100, color=BLUE, linewidth=1.8, label="fingertip height (FK)")
    key_tops = [names[k][2] * 100 for k in ("p", "o", "l") if k in names]
    if key_tops:
        ax.axhline(np.mean(key_tops), color=GREY, linewidth=1, linestyle=(0, (4, 3)))
        ax.text(0.3, np.mean(key_tops) + 0.2, "keys (pressed)", color=INK_2, fontsize=9, ha="left", va="bottom")
    if "home" in names:
        ax.axhline(names["home"][2] * 100, color=GREY, linewidth=1, linestyle=(0, (4, 3)))
        ax.text(0.3, names["home"][2] * 100 - 0.2, "home (commanded)", color=INK_2, fontsize=9, ha="left", va="top")
    top = ax.get_ylim()[1]
    for t, x, y, z in zip(targets["t"], targets["x"], targets["y"], targets["z"]):
        match = [n for n, p in names.items() if np.abs(p - [x, y, z]).max() < 0.0005]
        name = match[0] if match else "?"
        if name != "home":
            ax.axvline(t - t0, color=ORANGE, linewidth=1)
            ax.text(t - t0, top, name, color=INK, ha="center", va="bottom", fontsize=11)
        settled = position_at(fk, (targets["t"][targets["t"] > t][0] if np.any(targets["t"] > t)
                                   else fk["t"][-1]) - 0.1)
        print(f"{name:5s} sent at {t - t0:5.1f} s: target ({x:.3f}, {y:.3f}, {z:.3f}), "
              f"fingertip settled at {np.round(settled, 3)}, error {np.linalg.norm(settled - [x, y, z]) * 1000:.1f} mm")
    ax.set_xlabel("time (s)")
    ax.set_ylabel("fingertip height z (cm)")
    ax.set_title("Real robot: fingertip height while typing; orange lines = key commands", pad=18)
    save(fig, run, "real_typing_height.png")


if __name__ == "__main__":
    commands = {"keys": plot_keys, "fk": plot_fk, "speed": plot_speed, "real": plot_real}
    if len(sys.argv) != 3 or sys.argv[1] not in commands:
        sys.exit(__doc__)
    commands[sys.argv[1]](sys.argv[2])
