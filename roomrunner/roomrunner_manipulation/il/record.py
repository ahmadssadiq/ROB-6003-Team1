"""Record the pickup demonstrations for one object with the leader arm (LeRobot lerobot-record).

    python record.py --object blue_cube              # its 20 episodes, in the planned order
    python record.py --object blue_cube --start 12   # go on from episode 12 of its plan after an interruption
    python record.py --object blue_cube --dry-run    # only print the plan and the command
    python record.py --object blue_cube --practice   # 5 warm-up episodes into a separate, throwaway dataset

Before: stop every ROS launch (LeRobot and robot_node cannot share the motor board), fix the robot in
place, tape the position mat (make_mat.py), fill in the PLACEHOLDERs of il_pick.yaml (leader port, cameras).

It prints the plan: for each episode, where to put the object (mat cross) and how to turn it. During
recording: the episode runs episode_time_s; then reset_time_s to put the object at the next position and
bring the leader arm back to its start pose. Keys: right arrow = end the episode early, left arrow =
record this episode again (the plan does not move on), Esc = stop and save.
All sessions go into the same dataset (lerobot-record --resume); a log of which episode is which position
is kept next to it (<dataset root>_episodes.csv), for the report.
"""
import argparse
import csv
import json
import shutil
import subprocess
import sys
import time
from pathlib import Path

from il_tools import episode_plan, load_config


def camera_argument(cameras):
    parts = [f"{name}: {{type: opencv, index_or_path: '{c['path']}', width: {c['width']}, height: {c['height']}, "
             f"fps: {c['fps']}}}" for name, c in cameras.items()]
    return "{" + ", ".join(parts) + "}"


def recorded_episodes(root):
    info = Path(root) / "meta" / "info.json"
    return json.loads(info.read_text())["total_episodes"] if info.exists() else 0


def setup_problems(cfg):
    problems = []
    if not cfg["leader"]["port"]:
        problems.append("leader.port is empty (lerobot-find-port)")
    for name, cam in cfg["cameras"].items():
        if not cam["path"]:
            problems.append(f"cameras.{name}.path is empty (lerobot-find-cameras opencv)")
    calibration = Path.home() / ".cache/huggingface/lerobot/calibration"
    if not (calibration / "robots" / "so_follower" / f"{cfg['robot']['id']}.json").exists():
        problems.append(f"no follower calibration {cfg['robot']['id']}.json: run make_arm_calibration.py")
    if not (calibration / "teleoperators" / "so_leader" / f"{cfg['leader']['id']}.json").exists():
        problems.append(f"no leader calibration {cfg['leader']['id']}.json: lerobot-calibrate the leader arm")
    if subprocess.run(["pgrep", "-f", "robot_node"], capture_output=True).returncode == 0:
        problems.append("robot_node (ROS) is running: stop the ROS launch first, it holds the motor board")
    return problems


def main():
    cfg = load_config()
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--object", required=True, choices=sorted(cfg["objects"]))
    parser.add_argument("--start", type=int, default=1, help="first episode of this object's plan (1-based)")
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--practice", action="store_true", help="5 episodes into ~/roomrunner_data/practice_pick")
    args = parser.parse_args()

    plan = episode_plan(cfg, args.object)[args.start - 1:]
    if args.practice:
        plan = plan[:5]
        cfg["dataset"]["root"] = str(Path(cfg["dataset"]["root"]).parent / "practice_pick")
        cfg["dataset"]["repo_id"] = "team1/practice_pick"
    if not plan:
        sys.exit("Nothing left in this object's plan.")
    d, obj = cfg["dataset"], cfg["objects"][args.object]
    print(f"\n{args.object} ({' x '.join(map(str, obj['size_cm']))} cm), task \"{obj['task']}\": "
          f"{len(plan)} episodes of {d['episode_time_s']} s, {d['reset_time_s']} s to reset\n")
    print(" episode  mat  base x, y (cm)  yaw")
    for i, (pid, (x, y), yaw) in enumerate(plan, args.start):
        print(f"   {i:3d}    {pid:4s} {100 * x:5.1f} {100 * y:+6.1f}    {yaw:3d} deg")
    print("\nEach episode: start with the leader arm in its rest pose, reach, grasp, lift about 5 cm, hold still "
          "until the episode ends. Same rest pose every time; a failed grasp: left arrow and record it again.\n")

    root = Path(d["root"])
    before = recorded_episodes(root)
    command = ["lerobot-record",
               f"--robot.type={cfg['robot']['type']}", f"--robot.port={cfg['robot']['port']}",
               f"--robot.id={cfg['robot']['id']}", f"--robot.cameras={camera_argument(cfg['cameras'])}",
               f"--teleop.type={cfg['leader']['type']}", f"--teleop.port={cfg['leader']['port']}",
               f"--teleop.id={cfg['leader']['id']}",
               f"--dataset.repo_id={d['repo_id']}", f"--dataset.root={root}", f"--dataset.fps={d['fps']}",
               f"--dataset.single_task={obj['task']}", f"--dataset.num_episodes={len(plan)}",
               f"--dataset.episode_time_s={d['episode_time_s']}", f"--dataset.reset_time_s={d['reset_time_s']}",
               "--dataset.push_to_hub=false", "--display_data=true"]
    if before:
        command.append("--resume=true")
    print(" ".join(f"'{c}'" if " " in c else c for c in command), "\n")
    if args.dry_run:
        return
    problems = setup_problems(cfg)
    if problems:
        sys.exit("Fix first:\n  " + "\n  ".join(problems))
    if shutil.which("lerobot-record") is None:
        sys.exit("lerobot-record not found: run this in the conda environment (conda activate ros_mujoco).")
    input("Robot fixed, mat taped, object at the first position? Press Enter to start recording...")
    subprocess.run(command)

    after = recorded_episodes(root)
    if args.practice:
        print(f"\nPractice done ({after} episodes in {root}); delete that folder when you are happy.")
        return
    log = root.parent / f"{root.name}_episodes.csv"
    new = not log.exists()
    with open(log, "a", newline="") as f:
        writer = csv.writer(f)
        if new:
            writer.writerow(["episode", "object", "plan_index", "position", "x", "y", "yaw_deg", "recorded"])
        for k, (pid, (x, y), yaw) in enumerate(plan[:after - before]):
            writer.writerow([before + k, args.object, args.start + k, pid, x, y, yaw, time.strftime("%Y-%m-%d %H:%M")])
    print(f"\n{after - before} episodes recorded ({after} in the dataset). Log: {log}")
    if after - before < len(plan):
        print(f"Interrupted: go on with  python record.py --object {args.object} --start {args.start + after - before}")
    print("Check them:  python check_dataset.py")


if __name__ == "__main__":
    main()
