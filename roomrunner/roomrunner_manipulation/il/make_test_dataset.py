"""A tiny made-up dataset with the same features as a real recording (so101_follower, front and wrist
cameras), to try check_dataset.py and the training command before recording anything.

    python make_test_dataset.py            # -> ~/roomrunner_data/test_pick (3 episodes, small images)
Run it with the conda Python (LeRobot). The data means nothing; only the format matters.
"""
import argparse
import shutil
from pathlib import Path

import numpy as np
from lerobot.datasets.lerobot_dataset import LeRobotDataset
from lerobot.utils.feature_utils import hw_to_dataset_features

JOINTS = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--root", default="~/roomrunner_data/test_pick")
    parser.add_argument("--episodes", type=int, default=3)
    parser.add_argument("--frames", type=int, default=90)
    args = parser.parse_args()
    root = Path(args.root).expanduser()
    if root.exists():
        shutil.rmtree(root)

    h, w = 96, 128
    motors = {f"{j}.pos": float for j in JOINTS}
    observation = {**motors, "front": (h, w, 3), "wrist": (h, w, 3)}
    features = {**hw_to_dataset_features(observation, "observation"), **hw_to_dataset_features(motors, "action")}
    dataset = LeRobotDataset.create("team1/test_pick", fps=30, features=features, root=root,
                                    robot_type="so101_follower", use_videos=True)
    rng = np.random.default_rng(0)
    for episode in range(args.episodes):
        for i in range(args.frames):
            s = i / args.frames
            joints = np.array([20 * s, -30 + 40 * s, 50 - 30 * s, 60 * s, 0.0, 40.0 if s < 0.5 else 5.0], np.float32)
            frame = {"observation.state": joints, "action": joints + rng.normal(0, 0.5, 6).astype(np.float32),
                     "observation.images.front": rng.integers(0, 255, (h, w, 3), dtype=np.uint8),
                     "observation.images.wrist": rng.integers(0, 255, (h, w, 3), dtype=np.uint8),
                     "task": ["Pick up the blue cube", "Pick up the yellow cuboid", "Pick up the violet hexagon"][episode % 3]}
            dataset.add_frame(frame)
        dataset.save_episode()
    dataset.finalize()
    print(f"wrote {root}: {args.episodes} episodes x {args.frames} frames")


if __name__ == "__main__":
    main()
