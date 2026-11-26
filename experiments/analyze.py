"""Plots and numbers for the report and website, from CSV logs in experiments/data/ (not in git).

    python3 experiments/analyze.py e1      e1_checkpoints.csv: route,run,checkpoint,true_x,true_y,odom_x,odom_y,pf_x,pf_y
                                           e1_timeseries.csv (optional): t,true_x,true_y,odom_x,odom_y,pf_x,pf_y
    python3 experiments/analyze.py e2      e2_trials.csv (from run_e2.py): id,steps,planner,valid,replans,success,...
    python3 experiments/analyze.py pickup  pickup_trials.csv (from il/evaluate.py): method,object,position,trial,success,...

PNG files are written to experiments/plots/.
"""
import csv
import math
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from metrics import rmse, success_rate, task_summary

HERE = Path(__file__).resolve().parent
DATA, PLOTS = HERE / "data", HERE / "plots"
BLUE, ORANGE, GREY = "#2a78d6", "#eb6834", "#8a8984"


def rows(name):
    with open(DATA / name) as f:
        return list(csv.DictReader(f))


def save(fig, name):
    PLOTS.mkdir(exist_ok=True)
    fig.tight_layout()
    fig.savefig(PLOTS / name, dpi=200)
    print("wrote", PLOTS / name)


def e1():
    r = rows("e1_checkpoints.csv")
    truth = [(float(x["true_x"]), float(x["true_y"])) for x in r]
    odom = rmse([(float(x["odom_x"]), float(x["odom_y"])) for x in r], truth)
    pf = rmse([(float(x["pf_x"]), float(x["pf_y"])) for x in r], truth)
    print(f"E1 over {len(r)} checkpoint visits: RMSE odometry {100 * odom:.1f} cm, particle filter {100 * pf:.1f} cm")
    fig, ax = plt.subplots(figsize=(4, 3))
    ax.bar(["odometry", "particle filter"], [100 * odom, 100 * pf], color=[GREY, BLUE])
    ax.set_ylabel("position RMSE (cm)")
    save(fig, "e1_rmse.png")
    if (DATA / "e1_timeseries.csv").exists():
        t = rows("e1_timeseries.csv")
        time = [float(x["t"]) for x in t]
        err = {k: [100 * math.hypot(float(x[f"{k}_x"]) - float(x["true_x"]), float(x[f"{k}_y"]) - float(x["true_y"]))
                   for x in t] for k in ("odom", "pf")}
        fig, ax = plt.subplots(figsize=(6, 3))
        ax.plot(time, err["odom"], color=GREY, label="odometry")
        ax.plot(time, err["pf"], color=BLUE, label="particle filter")
        ax.set_xlabel("time (s)")
        ax.set_ylabel("position error (cm)")
        ax.legend(frameon=False)
        save(fig, "e1_error_over_time.png")


def e2():
    raw = rows("e2_trials.csv")
    trials = [dict(planner=x["planner"], valid=x["valid"] == "1", replans=int(x["replans"]), success=x["success"] == "1")
              for x in raw]
    summary = task_summary(trials)
    for planner, s in summary.items():
        print(f"E2 {planner}: SR {s['SR']:.2f}, valid plans {s['valid_fraction']:.2f}, "
              f"re-plans {s['mean_replans']:.2f} over {s['trials']} trials")
    names = list(summary)
    colors = [BLUE, ORANGE, GREY]
    fig, ax = plt.subplots(figsize=(4, 3))
    ax.bar(names, [summary[n]["SR"] for n in names], color=[colors[i % 3] for i in range(len(names))])
    ax.set_ylim(0, 1)
    ax.set_ylabel("task success rate")
    save(fig, "e2_success_rate.png")

    # Success rate by number of steps (1 to 4), one bar per planner
    steps = sorted({int(x["steps"]) for x in raw})
    width = 0.8 / len(names)
    fig, ax = plt.subplots(figsize=(5, 3))
    for i, name in enumerate(names):
        rates = [success_rate([x["success"] == "1" for x in raw if x["planner"] == name and int(x["steps"]) == n])
                 for n in steps]
        ax.bar([n + (i - (len(names) - 1) / 2) * width for n in steps], rates, width, label=name, color=colors[i % 3])
    ax.set_xticks(steps)
    ax.set_xlabel("steps in the request")
    ax.set_ylim(0, 1)
    ax.set_ylabel("task success rate")
    ax.legend()
    save(fig, "e2_success_by_steps.png")


def pickup():
    r = rows("pickup_trials.csv")
    methods = sorted({x["method"] for x in r}, key=lambda m: (m != "scripted", m))     # scripted first
    objects = sorted({x["object"] for x in r})
    colors = [GREY, BLUE, ORANGE]
    sr = {m: success_rate([x["success"] == "1" for x in r if x["method"] == m]) for m in methods}
    for m in methods:
        n = sum(x["method"] == m for x in r)
        per = ", ".join(f"{o} {success_rate([x['success'] == '1' for x in r if x['method'] == m and x['object'] == o]):.2f}"
                        for o in objects if any(x["method"] == m and x["object"] == o for x in r))
        print(f"pickup {m}: SR {sr[m]:.2f} over {n} trials ({per})")
    fig, ax = plt.subplots(figsize=(4, 3))
    ax.bar(methods, [sr[m] for m in methods], color=colors[:len(methods)])
    ax.set_ylim(0, 1)
    ax.set_ylabel("pickup success rate (held-out positions)")
    save(fig, "pickup_success_rate.png")

    # By object, one bar per method
    width = 0.8 / len(methods)
    fig, ax = plt.subplots(figsize=(5, 3))
    for i, m in enumerate(methods):
        rates = [success_rate([x["success"] == "1" for x in r if x["method"] == m and x["object"] == o] or [0])
                 for o in objects]
        ax.bar([k + (i - (len(methods) - 1) / 2) * width for k in range(len(objects))], rates, width, label=m,
               color=colors[i % 3])
    ax.set_xticks(range(len(objects)))
    ax.set_xticklabels([o.replace("_", " ") for o in objects])
    ax.set_ylim(0, 1)
    ax.set_ylabel("pickup success rate")
    ax.legend()
    save(fig, "pickup_by_object.png")


if __name__ == "__main__":
    commands = {"e1": e1, "e2": e2, "pickup": pickup}
    if len(sys.argv) != 2 or sys.argv[1] not in commands:
        sys.exit(__doc__)
    commands[sys.argv[1]]()
