"""Print the position mat: an A4 page, 1:1 scale, with the 10 training (T) and 5 held-out (H) object positions.

    python3 make_mat.py                  # writes position_mat.pdf next to this script
Print at 100 % ("actual size", no "fit to page"), check the 10 cm scale bar with a ruler, then tape the page
to the floor with its top edge against the robot's front edge and its centre line on the robot's axis.

Each cross is an object centre in the base frame (il_pick.yaml, positions); the page's top edge is at
base x = front_edge_x. If front_edge_x was measured wrong, every cross is off by the same amount: check one
with the arm before recording (README, step 2).
"""
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from il_tools import load_config, mat_marks

WIDTH, HEIGHT = 29.7, 21.0     # A4 landscape, cm
CM = 1 / 2.54                  # inches per cm


def main():
    cfg = load_config()
    fig = plt.figure(figsize=(WIDTH * CM, HEIGHT * CM))
    ax = fig.add_axes([0, 0, 1, 1])
    ax.set_xlim(-WIDTH / 2, WIDTH / 2)
    ax.set_ylim(HEIGHT, 0)                 # v grows down the page, away from the robot
    ax.set_aspect("equal")
    ax.axis("off")

    ax.plot([-WIDTH / 2, WIDTH / 2], [0.15, 0.15], color="black", lw=2)
    ax.text(0, 0.8, f"ROBOT FRONT EDGE (base x = {100 * cfg['front_edge_x']:.0f} cm): lay this edge against the robot",
            ha="center", va="top", fontsize=8)
    ax.plot([0, 0], [0, HEIGHT], color="0.6", lw=0.8, ls="--")
    ax.text(0.3, HEIGHT - 0.6, "robot axis (base y = 0)", fontsize=7, color="0.4")
    ax.text(WIDTH / 2 - 0.5, 2.0, "robot's left  ->", ha="right", fontsize=7, color="0.4")

    for label, u, v in mat_marks(cfg):
        colour = "tab:blue" if label.startswith("T") else "tab:red"
        ax.plot([u - 0.6, u + 0.6], [v, v], color=colour, lw=1.2)
        ax.plot([u, u], [v - 0.6, v + 0.6], color=colour, lw=1.2)
        x, y = cfg["positions"][label]
        ax.text(u + 0.35, v - 0.35, label, color=colour, fontsize=9, fontweight="bold")
        ax.text(u + 0.35, v + 0.9, f"x {100 * x:.1f}  y {100 * y:+.1f}", color=colour, fontsize=5.5)

    ax.plot([-WIDTH / 2 + 1, -WIDTH / 2 + 11], [HEIGHT - 1.2] * 2, color="black", lw=2)
    ax.text(-WIDTH / 2 + 6, HEIGHT - 1.5, "10 cm: check with a ruler", ha="center", fontsize=7)
    ax.text(WIDTH / 2 - 1, HEIGHT - 1.2, "T = training (record)   H = held-out (evaluation only)\n"
            "yaw 0 = the object's long side pointing away from the robot", ha="right", fontsize=7)
    out = Path(__file__).resolve().parent / "position_mat.pdf"
    fig.savefig(out)
    print(f"wrote {out}: print at 100 % (actual size)")


if __name__ == "__main__":
    main()
