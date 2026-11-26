"""Evaluate the pickup on the held-out positions: scripted (Assignment 03 IK) against learned (ACT), same trials.

    python evaluate.py sheet                    # print the trial sheet (object, mat cross, yaw for each trial)
    python evaluate.py act                      # run ACT for every trial (lerobot-rollout, conda env, no ROS)
    python evaluate.py score --method act       # then type in what you wrote on the sheet: 1 = picked, 0 = not
    python evaluate.py scripted                 # the scripted pick, trial by trial (ROS robot.launch.py arm:=real)

Success = the object is lifted off the mat and still held when the trial ends. The same success rule, the
same trials and the same order for both methods. Results go to experiments/data/pickup_trials.csv
(columns: method, object, position, trial, yaw_deg, success, seconds, note); plot them with
python3 experiments/analyze.py pickup.

act: one lerobot-rollout session (strategy episodic), one episode per trial; between trials the arm goes
back to its start pose by itself. Put the object as the sheet says during each reset. The rollouts are saved
as a dataset (~/roomrunner_data/eval_act) so the videos can be checked and shown.
scripted: try_pick at the cross's base-frame position, then you say whether it held the object.
"""
import argparse
import subprocess
import sys
import time
from pathlib import Path

from il_tools import ROOT, done_trials, evaluation_plan, load_config, log_trial
from record import camera_argument


def print_sheet(trials):
    print(" trial  object          mat  yaw      result")
    for i, (obj, pid, (x, y), t, yaw) in enumerate(trials, 1):
        print(f"  {i:3d}   {obj:15s} {pid:4s} {yaw:3d} deg   ___")


def run_act(cfg, trials):
    e, d = cfg["evaluation"], cfg["dataset"]
    if not Path(e["policy_path"]).exists():
        sys.exit(f"No policy at {e['policy_path']}: download the trained checkpoint there (README, step 5).")
    root = Path(d["root"]).parent / "eval_act"
    command = ["lerobot-rollout", "--strategy.type=episodic", f"--policy.path={e['policy_path']}",
               f"--robot.type={cfg['robot']['type']}", f"--robot.port={cfg['robot']['port']}",
               f"--robot.id={cfg['robot']['id']}", f"--robot.cameras={camera_argument(cfg['cameras'])}",
               "--dataset.repo_id=team1/eval_act", f"--dataset.root={root}", "--dataset.single_task=Pick up the object",
               f"--dataset.num_episodes={len(trials)}", f"--dataset.episode_time_s={e['rollout_duration_s']}",
               f"--dataset.reset_time_s={d['reset_time_s']}", "--dataset.push_to_hub=false", "--display_data=true"]
    if (root / "meta" / "info.json").exists():
        command.append("--resume=true")
    print_sheet(trials)
    print("\nWrite 1 or 0 next to each trial as it ends; then: python evaluate.py score --method act\n")
    input("Object at trial 1's position, arm in its start pose? Press Enter...")
    subprocess.run(command)


def score(cfg, trials, method, path):
    done = done_trials(path, method)
    todo = [t for t in trials if (t[0], t[1], t[3]) not in done]
    print(f"{len(todo)} trials to score for {method}, in sheet order.")
    answer = input("Results as one string of 1 and 0 (e.g. 1101...), or one per line: ").strip()
    while len(answer.replace(" ", "")) < len(todo):
        more = input(f"  {len(answer.replace(' ', ''))} of {len(todo)} so far, more: ").strip()
        if not more:
            break
        answer += more
    for (obj, pid, _, t, yaw), r in zip(todo, answer.replace(" ", "")):
        log_trial(path, {"method": method, "object": obj, "position": pid, "trial": t, "yaw_deg": yaw,
                         "success": int(r == "1")})
    print(f"Logged {min(len(todo), len(answer.replace(' ', '')))} trials to {path}")


def run_scripted(cfg, trials, path):
    done = done_trials(path, "scripted")
    for i, (obj, pid, (x, y), t, yaw) in enumerate(trials, 1):
        if (obj, pid, t) in done:
            continue
        print(f"\ntrial {i}/{len(trials)}: {obj} at {pid} (x {100 * x:.1f}, y {100 * y:+.1f} cm), yaw {yaw} deg")
        if input("Object placed? Enter to run, q to stop: ").strip().lower() == "q":
            return
        start = time.monotonic()
        out = subprocess.run(["ros2", "run", "roomrunner_manipulation", "try_pick", "--x", str(x), "--y", str(y),
                              "--name", obj], capture_output=True, text=True)
        seconds = time.monotonic() - start
        report = (out.stdout.strip().splitlines() or ["(no output)"])[-1]
        print(f"  {report}")
        held = input("  Lifted and still held? [y/n] ").strip().lower() == "y"
        log_trial(path, {"method": "scripted", "object": obj, "position": pid, "trial": t, "yaw_deg": yaw,
                         "success": int(held), "seconds": round(seconds, 1), "note": report[:120]})
        print("  Take the object out of the gripper.")


def main():
    cfg = load_config()
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("what", choices=["sheet", "act", "score", "scripted"])
    parser.add_argument("--method", default="act", help="score: which method's results to type in")
    args = parser.parse_args()
    trials = evaluation_plan(cfg)
    path = ROOT / cfg["evaluation"]["output"]
    if args.what == "sheet":
        print_sheet(trials)
    elif args.what == "act":
        run_act(cfg, trials)
    elif args.what == "score":
        score(cfg, trials, args.method, path)
    else:
        run_scripted(cfg, trials, path)


if __name__ == "__main__":
    main()
