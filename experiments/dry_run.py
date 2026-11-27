"""Dry run of the planning stack without ROS: request -> steps -> mission state machine -> A* routes -> PNG.

    python3 experiments/dry_run.py
    python3 experiments/dry_run.py --request "bring the blue block to the tray then go home"
    python3 experiments/dry_run.py --room roomrunner/roomrunner_bringup/config/fake_room.yaml --radius 0.20

It runs the same code as the robot: the mock parser, the validator, mission.run_mission with the state machine
of fsm.py, and A* (astar.py) on the room's occupancy grid inflated by the robot radius. Only the skills are fake:
navigation teleports the robot to the end of the planned path, and every pick and place succeeds. Each leg's
route is drawn on the grid and saved to experiments/plots/dry_run.png. Units: metres and radians, map frame.
"""
import argparse
import math
import sys
from pathlib import Path

import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "roomrunner" / "roomrunner_planning"))

from roomrunner_planning.astar import (astar, cell_to_world, inflate, nearest_free,  # noqa: E402
                                       path_length, world_to_cell)
from roomrunner_planning.mission import run_mission  # noqa: E402
from roomrunner_planning.mock_parser import parse  # noqa: E402
from roomrunner_planning.validator import validate  # noqa: E402

ROOM_FILE = ROOT / "roomrunner" / "roomrunner_bringup" / "config" / "fake_room.yaml"
PLOTS = Path(__file__).resolve().parent / "plots"


def room_grid(room):
    """0 free / 100 occupied grid of the room (row = y, col = x). Same layout as fake_overhead.room_grid,
    copied here because that module needs ROS to import."""
    res = room["resolution"]
    rows, cols = round(room["height"] / res), round(room["width"] / res)
    grid = np.zeros((rows, cols), dtype=np.int8)
    wall = max(1, round(room["wall_thickness"] / res))
    grid[:wall, :] = grid[-wall:, :] = grid[:, :wall] = grid[:, -wall:] = 100
    for x_min, y_min, x_max, y_max in room["obstacles"]:
        grid[int(y_min / res):math.ceil(y_max / res), int(x_min / res):math.ceil(x_max / res)] = 100
    return grid


def approach_pose(robot_xy, target_xy, standoff):
    """Where the base stops next to an object: `standoff` m short of it, facing it (as in mission_fsm)."""
    yaw = math.atan2(target_xy[1] - robot_xy[1], target_xy[0] - robot_xy[0])
    return target_xy[0] - standoff * math.cos(yaw), target_xy[1] - standoff * math.sin(yaw), yaw


class OfflineSkills:
    """The skills mission.run_mission needs, on the fake room: real A*, fake driving and grasping."""

    def __init__(self, room_cfg, radius, standoff):
        room = room_cfg["room"]
        self.res, self.origin = room["resolution"], (0.0, 0.0)
        self.occupied = room_grid(room) != 0
        self.blocked = inflate(self.occupied, radius / self.res)
        self.objects = {o["name"]: o for o in room_cfg["objects"]}
        self.places = room_cfg.get("places", {})
        start = room_cfg["robot_start"]
        self.robot = (start["x"], start["y"], start["yaw"])
        self.standoff = standoff
        self.routes = []            # (target, list of (x, y), length m)
        self.path = None

    def refresh_map(self):
        pass

    def check(self, steps):
        return validate(steps, {n: o["kind"] for n, o in self.objects.items()}, set(self.places))

    def plan(self, target):
        if target in self.objects:
            obj = self.objects[target]
            goal = approach_pose(self.robot[:2], (obj["x"], obj["y"]), self.standoff)
        else:
            goal = (self.places[target]["x"], self.places[target]["y"], self.places[target]["yaw"])
        start = nearest_free(self.blocked, world_to_cell(*self.robot[:2], self.origin, self.res))
        end = nearest_free(self.blocked, world_to_cell(*goal[:2], self.origin, self.res))
        cells = astar(self.blocked, start, end) if start and end else None
        if cells is None:
            return False, "no path"
        points = [cell_to_world(r, c, self.origin, self.res) for r, c in cells]
        length = path_length(cells, self.res)
        self.path = (target, points, length, goal[2])
        return True, f"{length:.2f} m"

    def navigate(self, target, on_arrived):
        _, points, length, yaw = self.path
        self.routes.append(self.path)
        self.robot = (points[-1][0], points[-1][1], yaw)
        on_arrived()
        return "aligned"

    def manipulate(self, skill, target):
        if skill == "pick":           # the block travels with the robot
            self.objects[target]["held"] = True
        return True


def draw(skills, request, out):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    h, w = skills.occupied.shape
    extent = (0, w * skills.res, 0, h * skills.res)
    image = np.ones((h, w, 3))
    image[skills.blocked] = (0.85, 0.85, 0.85)      # grown by the robot radius
    image[skills.occupied] = (0.25, 0.25, 0.25)     # real obstacles and walls
    fig, ax = plt.subplots(figsize=(8, 6))
    ax.imshow(image, origin="lower", extent=extent)
    colours = ["#2a78d6", "#eb6834", "#3a9a5b", "#9b59b6", "#c0392b"]
    for i, (target, points, length, _) in enumerate(skills.routes):
        xs, ys = zip(*points)
        ax.plot(xs, ys, "-", lw=2.5, color=colours[i % len(colours)], label=f"leg {i + 1}: to {target} ({length:.2f} m)")
        ax.plot(xs[-1], ys[-1], "o", color=colours[i % len(colours)])
    for name, obj in skills.objects.items():
        ax.plot(obj["x"], obj["y"], "s" if obj["kind"] == "block" else "D", color="k", ms=7)
        ax.annotate(name, (obj["x"], obj["y"]), xytext=(5, 5), textcoords="offset points", fontsize=8)
    for name, place in skills.places.items():
        ax.plot(place["x"], place["y"], "*", color="k", ms=10)
        ax.annotate(name, (place["x"], place["y"]), xytext=(5, -10), textcoords="offset points", fontsize=8)
    ax.set_xlabel("map x (m)")
    ax.set_ylabel("map y (m)")
    ax.set_title(f'"{request}"\nA* 8-connected, dark = obstacles, grey = grown by robot radius', fontsize=10)
    ax.legend(loc="upper left", fontsize=8)
    fig.tight_layout()
    out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out, dpi=150)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--request", default="bring the red block to the tray")
    parser.add_argument("--room", default=str(ROOM_FILE))
    parser.add_argument("--radius", type=float, default=0.20, help="robot radius for inflation (m)")
    parser.add_argument("--standoff", type=float, default=0.30, help="base centre to object at a stop (m)")
    parser.add_argument("--out", default=str(PLOTS / "dry_run.png"))
    parser.add_argument("--no-plot", action="store_true")
    args = parser.parse_args()

    room_cfg = yaml.safe_load(Path(args.room).read_text())
    skills = OfflineSkills(room_cfg, args.radius, args.standoff)
    try:
        steps = parse(args.request)
    except ValueError as error:
        print(f"mock parser: {error}")
        steps = []
    print("steps:", ", ".join(f"{s}({t})" for s, t in steps) or "none")

    def on_state(state, detail, leg):
        print(f"  [{state:15s}] {detail}")

    final, detail = run_mission(steps, skills, on_state=on_state, request=args.request)
    print(f"result: {final} ({detail}); total route {sum(r[2] for r in skills.routes):.2f} m")
    if not args.no_plot and skills.routes:
        draw(skills, args.request, Path(args.out))
        print(f"wrote {args.out}")
    return 0 if final == "DONE" else 1


if __name__ == "__main__":
    sys.exit(main())
