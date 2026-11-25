# Imitation-learning pickup (owner: Sarvanagupta)

Learned pickup with LeRobot ACT, compared with the scripted pick (Assignment 03 IK) on held-out block
positions. Datasets and model weights stay out of git (`~/roomrunner_data/`, see `.gitignore`).

## 1. Setup (once)

- Stop any ROS launch: LeRobot and robot_node cannot share the motor board.
- `python make_arm_calibration.py` creates the arm calibration `lekiwi_arm` from the LeKiwi calibration.
- `lerobot-find-port` (motor board and leader arm), `lerobot-find-cameras opencv` (front, wrist).
- Tape 10 training positions and 5 held-out test positions in front of the robot. Fix the robot in place.

## 2. Record (about 60 episodes: 3 objects x 10 positions x 2)

```bash
lerobot-record --robot.type=so101_follower --robot.port=<motor board> --robot.id=lekiwi_arm \
  --teleop.type=so101_leader --teleop.port=<leader arm> --teleop.id=leader \
  --robot.cameras="{front: {type: opencv, index_or_path: 0, width: 640, height: 480, fps: 30}}" \
  --display_data=true --dataset.repo_id=team1/roomrunner_pick \
  --dataset.root=$HOME/roomrunner_data/roomrunner_pick --dataset.single_task="Pick up the block" \
  --dataset.num_episodes=10 --dataset.episode_time_s=15 --dataset.reset_time_s=10 --dataset.push_to_hub=false
```

Keys: right arrow ends an episode early, left arrow re-records it, Esc stops. Add `--resume=true` to add
episodes. One object in view per episode; same start pose every time; delete failed grasps.

## 3. Train (on a GPU laptop or Colab, not the mini PC)

```bash
lerobot-train --policy.type=act --dataset.repo_id=team1/roomrunner_pick \
  --dataset.root=$HOME/roomrunner_data/roomrunner_pick --output_dir=$HOME/roomrunner_data/act_pick \
  --policy.device=cuda --policy.push_to_hub=false
```

## 4. Evaluate

20 trials each for scripted and ACT on the 5 held-out positions; log successes with
`experiments/pickup_log.csv` and plot with `experiments/analyze.py pickup`.
