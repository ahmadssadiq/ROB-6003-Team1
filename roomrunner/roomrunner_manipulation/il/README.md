# Imitation-learning pickup (owner: Sarvanagupta)

A learned pickup (LeRobot ACT, trained from leader-arm demonstrations) compared with the scripted pick
(Assignment 03 IK, `try_pick`) on positions the policy never saw. Everything is set in one file,
[`roomrunner_bringup/config/il_pick.yaml`](../../roomrunner_bringup/config/il_pick.yaml): ports, cameras,
objects, the 10 training and 5 held-out positions, episode times, evaluation trials.

Datasets and checkpoints stay out of git, in `~/roomrunner_data/`.

| Step | Where | What | Time |
| --- | --- | --- | --- |
| 1 | robot PC | setup once: calibration, ports, cameras | 20 min |
| 2 | robot PC | position mat: print, tape, check | 15 min |
| 3 | robot PC | practice: 5 throwaway episodes | 10 min |
| 4 | robot PC | record: 3 objects x 10 positions x 2 = 60 episodes | about 1 h |
| 5 | robot PC | check the dataset | 5 min |
| 6 | Colab | train ACT | about 2 h |
| 7 | robot PC | evaluate: 30 trials ACT, 30 trials scripted | about 1 h |

Run every command here in `roomrunner/roomrunner_manipulation/il`, in the conda environment
(`conda activate ros_mujoco`), with **no ROS launch running** (LeRobot and `robot_node` cannot share the
motor board), except where step 7 says otherwise.

## 1. Setup (once)

```bash
python make_arm_calibration.py          # the arm's calibration lekiwi_arm, copied from the LeKiwi calibration
lerobot-find-port                       # unplug and replug the leader arm when asked: that is its port
lerobot-find-cameras opencv             # plug in both cameras first; note which is front and which is wrist
ls -l /dev/v4l/by-id/                   # stable names for the cameras (indexes change when replugged)
```

Fill in `il_pick.yaml`: `leader.port`, `cameras.front.path`, `cameras.wrist.path` (the `/dev/v4l/by-id/...-index0`
names). Check the leader arm moves the robot arm and both cameras show up (the window shows the camera views):

```bash
lerobot-teleoperate --robot.type=so101_follower --robot.port=<motor board> --robot.id=lekiwi_arm \
  --robot.cameras="{front: {type: opencv, index_or_path: '<front path>', width: 640, height: 480, fps: 30}, wrist: {type: opencv, index_or_path: '<wrist path>', width: 640, height: 480, fps: 30}}" \
  --teleop.type=so101_leader --teleop.port=<leader port> --teleop.id=leader --display_data=true
```

Also check that the gripper opens wider than 5 cm: the violet hexagon is 5.0 cm across its flat sides.

## 2. Position mat

```bash
python make_mat.py                      # -> position_mat.pdf
```

1. Measure `front_edge_x`: from the centre of the base (between the three wheels) to the robot's front
   edge, in metres. Put it in `il_pick.yaml` and make the mat again.
2. Print at **100 % (actual size)**, check the 10 cm bar with a ruler.
3. Fix the robot so it cannot roll (it must not move between episodes). Tape the mat to the floor: top edge
   against the front edge, the dashed line on the robot's axis. Put a strip of tape on the floor along the
   wheels, so you see if the robot moved.
4. Check one cross with the arm (the ROS stack, then stop it again):
   ```bash
   ros2 launch roomrunner_bringup robot.launch.py port:=<motor board> enable_motors:=true arm:=real
   ros2 topic pub --once /ik_target geometry_msgs/msg/PoseStamped "{header: {frame_id: base}, pose: {position: {x: 0.225, y: 0.0, z: 0.0}}}"
   ```
   The fingertip should stop about 3.5 cm above cross H3. If it is off along the robot's axis, correct
   `front_edge_x` and print again.

## 3. Practice

```bash
python record.py --object blue_cube --practice
```

Five episodes into `~/roomrunner_data/practice_pick`, to get used to the leader arm and the keys. Delete that
folder afterwards. **How to demonstrate** (the policy copies what you do, mistakes included):

- Start every episode from the **same rest pose** of the leader arm (pick one and mark it with tape).
- One smooth movement: over the object, open, down, close, lift about 5 cm, **hold still** until the
  episode ends. No pauses to think in the middle, no corrections if possible.
- The same strategy every time: grasp the yellow cuboid across its narrow side (2.0 cm), the hexagon across
  two flat sides, the cube across two faces. Turn the wrist to the object's yaw.
- Keep your hands and body out of both camera views; same lighting for all sessions (no sun through a window).
- A failed or clumsy episode: **left arrow** during it records it again. Right arrow ends it early, Esc stops.

## 4. Record

```bash
python record.py --object yellow_cuboid
python record.py --object violet_hexagon
python record.py --object blue_cube
```

Each prints its 20 episodes first: the mat cross and the yaw (0 = the long side pointing away from the
robot). Keep that list next to you. During each reset (15 s) put the object on the next cross and bring the
leader arm back to the rest pose. All three go into one dataset (`~/roomrunner_data/roomrunner_pick`); which
episode is which position is logged in `~/roomrunner_data/roomrunner_pick_episodes.csv`.

Interrupted? It prints the command to go on, e.g. `python record.py --object blue_cube --start 12`.

## 5. Check

```bash
python check_dataset.py
```

Prints episodes per object, lengths and the episodes that look wrong (too short, the arm does not move, the
gripper never closes, servo glitches), and saves `~/roomrunner_data/roomrunner_pick_check.png` with the first
and last frame of both cameras (the object should be in view and held at the end). Re-record flagged episodes
for the same positions with `--start`. To drop bad ones, keep the raw dataset and write a filtered copy
before training (the episode numbers change):

```bash
lerobot-edit-dataset --repo_id team1/roomrunner_pick --root ~/roomrunner_data/roomrunner_pick \
  --new_repo_id team1/roomrunner_pick --new_root ~/roomrunner_data/roomrunner_pick_clean \
  --operation.type delete_episodes --operation.episode_indices "[3, 17]"
```

## 6. Train (Colab)

```bash
cd ~/roomrunner_data && zip -r roomrunner_pick.zip roomrunner_pick      # or the _clean copy, renamed
```

Upload the zip to Google Drive in `roomrunner/`, open [`train_act_colab.ipynb`](train_act_colab.ipynb) in
Colab with a GPU runtime and run the cells. Then download `roomrunner/act_pick_last.zip` and:

```bash
mkdir -p ~/roomrunner_data/act_pick && unzip act_pick_last.zip -d ~/roomrunner_data/act_pick
ls ~/roomrunner_data/act_pick/checkpoints/last/pretrained_model     # config.json, model.safetensors, ...
```

## 7. Evaluate

Same 30 trials for both methods: each object at each held-out cross (H1 to H5) twice, in the order of the
sheet. **Success** = the object is lifted off the mat and still held when the trial ends.

```bash
python evaluate.py sheet > sheet.txt         # print it, write 1 / 0 next to each trial
python evaluate.py act                       # ACT: one rollout per trial, the arm resets by itself
python evaluate.py score --method act        # type in the 1s and 0s from the sheet
```

Then the scripted pick, with the ROS stack (`robot.launch.py ... arm:=real`) running:

```bash
python3 evaluate.py scripted                 # runs try_pick at each cross and asks whether it held
python3 ../../../experiments/analyze.py pickup
```

Results: `experiments/data/pickup_trials.csv`; plots `experiments/plots/pickup_success_rate.png` and
`pickup_by_object.png`. The ACT rollouts are saved as videos in `~/roomrunner_data/eval_act` for the report.

## Files

| File | What |
| --- | --- |
| `il_tools.py` | plain Python: episode and evaluation plans, mat marks, episode checks, CSV log (tested in `tests/test_il_tools.py`) |
| `make_arm_calibration.py` | the arm's LeRobot calibration from the LeKiwi one |
| `make_mat.py` | the printable position mat |
| `record.py` | prints the plan, checks the setup, runs `lerobot-record` |
| `check_dataset.py` | checks a recorded dataset |
| `make_test_dataset.py` | a tiny made-up dataset in the same format, to try `check_dataset.py` and training |
| `train_act_colab.ipynb` | training on Colab |
| `evaluate.py` | the evaluation: trial sheet, ACT rollouts, scoring, scripted trials |
