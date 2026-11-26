"""Plain-Python helpers for the imitation-learning pickup (no ROS, no LeRobot), so they can be tested anywhere.

    cfg = load_config()                         # roomrunner_bringup/config/il_pick.yaml
    plan = episode_plan(cfg, "blue_cube")       # which position and yaw for each episode, in order
    trials = evaluation_plan(cfg)               # held-out positions x objects x trials
    log_trial(path, row)                        # append one evaluation result to the CSV
    problems = check_episode(actions, fps)      # sanity checks on one recorded episode
"""
import csv
import math
import random
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[3]
CONFIG = ROOT / "roomrunner" / "roomrunner_bringup" / "config" / "il_pick.yaml"
TRIAL_COLUMNS = ["method", "object", "position", "trial", "yaw_deg", "success", "seconds", "note"]


def load_config(path=CONFIG):
    cfg = yaml.safe_load(Path(path).read_text())
    for key in ("root",):
        cfg["dataset"][key] = str(Path(cfg["dataset"][key]).expanduser())
    cfg["evaluation"]["policy_path"] = str(Path(cfg["evaluation"]["policy_path"]).expanduser())
    return cfg


def training_positions(cfg):
    return {k: tuple(v) for k, v in cfg["positions"].items() if k.startswith("T")}


def held_out_positions(cfg):
    return {k: tuple(v) for k, v in cfg["positions"].items() if k.startswith("H")}


def episode_plan(cfg, object_name):
    """Episodes to record for one object: every training position `repetitions` times, each time with a
    different yaw, in a shuffled order (fixed by the seed). Returns [(position id, (x, y), yaw_deg)]."""
    if object_name not in cfg["objects"]:
        raise ValueError(f"unknown object {object_name!r}; one of {sorted(cfg['objects'])}")
    rng = random.Random(f"{cfg['seed']}-{object_name}")
    yaws = cfg["yaws_deg"]
    plan = []
    for rep in range(cfg["repetitions"]):
        for i, (pid, xy) in enumerate(training_positions(cfg).items()):
            plan.append((pid, xy, yaws[(i + rep * 2 + hash_name(object_name)) % len(yaws)]))
    rng.shuffle(plan)
    return plan


def hash_name(name):
    """A small stable number from a name (Python's hash() changes between runs)."""
    return sum(ord(c) for c in name)


def evaluation_plan(cfg):
    """Evaluation trials, the same for every method: each object at each held-out position
    trials_per_position times, yaw varied. Returns [(object, position id, (x, y), trial, yaw_deg)]."""
    rng = random.Random(f"{cfg['seed']}-evaluation")
    trials = []
    for obj in cfg["objects"]:
        for pid, xy in held_out_positions(cfg).items():
            for t in range(1, cfg["evaluation"]["trials_per_position"] + 1):
                trials.append((obj, pid, xy, t, rng.choice(cfg["yaws_deg"])))
    rng.shuffle(trials)
    return trials


def done_trials(path, method):
    """(object, position, trial) already logged for this method, so an interrupted evaluation can go on."""
    if not Path(path).exists():
        return set()
    with open(path) as f:
        return {(r["object"], r["position"], int(r["trial"])) for r in csv.DictReader(f) if r["method"] == method}


def log_trial(path, row):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    new = not path.exists()
    with open(path, "a", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=TRIAL_COLUMNS)
        if new:
            writer.writeheader()
        writer.writerow({k: row.get(k, "") for k in TRIAL_COLUMNS})


def check_episode(actions, fps, gripper_index=5, min_seconds=3.0, min_gripper_change=10.0):
    """Problems with one recorded episode, as a list of strings (empty = looks fine).

    actions: list of per-frame action vectors (6 values: 5 joints in degrees, gripper in %).
    A good pick episode is long enough, the arm actually moves, and the gripper opens and closes."""
    problems = []
    n = len(actions)
    if n < min_seconds * fps:
        problems.append(f"only {n / fps:.1f} s long")
        if n == 0:
            return problems
    joints = list(zip(*actions))
    travel = max(max(j) - min(j) for j in joints[:gripper_index])
    if travel < 5.0:
        problems.append(f"the arm barely moves ({travel:.1f} deg)")
    gripper = joints[gripper_index]
    if max(gripper) - min(gripper) < min_gripper_change:
        problems.append(f"the gripper hardly changes ({max(gripper) - min(gripper):.1f} %): no grasp?")
    jumps = [abs(b - a) for j in joints[:gripper_index] for a, b in zip(j, j[1:])]   # joints only: the gripper may snap
    if jumps and max(jumps) > 30.0:
        problems.append(f"a jump of {max(jumps):.0f} in one frame (dropped frames or a servo glitch)")
    return problems


def mat_marks(cfg):
    """Marks for the printed position mat, in mat coordinates (cm): the mat's top edge lies against the
    robot's front edge, its centre line on the robot's axis. Returns [(label, u_cm, v_cm)] with u to the
    robot's left (= base y) and v away from the robot (= base x - front_edge_x)."""
    marks = []
    for pid, (x, y) in cfg["positions"].items():
        marks.append((pid, 100 * y, 100 * (x - cfg["front_edge_x"])))
    return marks


def yaw_arrow(yaw_deg, length=1.0):
    """Direction (du, dv) on the mat of an object's long side at this yaw (yaw 0 = along base x).
    Base (x, y) = (cos, sin) of the yaw; on the mat u is base y and v is base x."""
    a = math.radians(yaw_deg)
    return math.sin(a) * length, math.cos(a) * length
