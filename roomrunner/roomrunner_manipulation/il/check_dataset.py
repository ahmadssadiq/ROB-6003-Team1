"""Check a recorded pickup dataset after each session: episodes per object, lengths, and episodes that look
wrong (too short, arm not moving, gripper never closing, servo glitches). Saves a picture with the first and
last frame of each camera for a few episodes, to see that both cameras recorded and the object is in view.

    python check_dataset.py                      # the dataset in il_pick.yaml
    python check_dataset.py --root ~/roomrunner_data/test_pick
Run it with the conda Python (LeRobot). Delete or re-record the episodes it flags
(lerobot-edit-dataset --operation.type=delete_episodes, see README).
"""
import argparse
from collections import Counter
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from lerobot.datasets.lerobot_dataset import LeRobotDataset

from il_tools import check_episode, load_config


def main():
    cfg = load_config()
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--root", default=cfg["dataset"]["root"])
    parser.add_argument("--repo-id", default=cfg["dataset"]["repo_id"])
    parser.add_argument("--pictures", type=int, default=6, help="episodes shown in the picture")
    args = parser.parse_args()
    root = Path(args.root).expanduser()

    ds = LeRobotDataset(args.repo_id, root=root)
    data = ds.hf_dataset
    episodes = np.asarray(data["episode_index"])
    actions = np.asarray([np.asarray(a) for a in data["action"]])
    tasks = ds.meta.tasks.index.tolist() if hasattr(ds.meta.tasks, "index") else list(ds.meta.tasks)
    task_of = {}
    for e, t in zip(episodes, np.asarray(data["task_index"])):
        task_of.setdefault(int(e), tasks[int(t)])

    print(f"{root}: {ds.num_episodes} episodes, {ds.num_frames} frames at {ds.fps} fps, "
          f"cameras {ds.meta.camera_keys}")
    for task, n in sorted(Counter(task_of.values()).items()):
        print(f"  {n:3d} episodes: {task}")
    flagged = 0
    for e in sorted(set(episodes.tolist())):
        problems = check_episode(actions[episodes == e].tolist(), ds.fps)
        if problems:
            flagged += 1
            print(f"  episode {e:3d} ({task_of[e]}): " + "; ".join(problems))
    lengths = [np.sum(episodes == e) / ds.fps for e in sorted(set(episodes.tolist()))]
    print(f"episode length {min(lengths):.1f} to {max(lengths):.1f} s; {flagged} episodes flagged")

    shown = sorted(set(episodes.tolist()))[-args.pictures:]
    cams = ds.meta.camera_keys
    fig, axes = plt.subplots(len(shown), 2 * len(cams), figsize=(3 * 2 * len(cams), 2.2 * len(shown)), squeeze=False)
    for row, e in zip(axes, shown):
        index = np.nonzero(episodes == e)[0]
        for k, (i, when) in enumerate([(index[0], "first"), (index[-1], "last")]):
            item = ds[int(i)]
            for c, cam in enumerate(cams):
                image = item[cam].permute(1, 2, 0).numpy()          # C, H, W in [0, 1] -> H, W, C
                ax = row[2 * c + k]
                ax.imshow(image)
                ax.set_title(f"ep {e} {cam.split('.')[-1]} {when}", fontsize=7)
                ax.axis("off")
    fig.tight_layout()
    out = root.parent / f"{root.name}_check.png"
    fig.savefig(out, dpi=80)
    print(f"wrote {out}")


if __name__ == "__main__":
    main()
