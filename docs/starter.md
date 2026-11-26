# Where each owner starts

Every module has its logic in plain Python (tested with `python3 -m pytest`, no ROS needed) and a thin ROS
node around it. `ros2 launch roomrunner_bringup stubs.launch.py` runs a whole mission with fake data.

| Owner | Plain-Python logic (tests) | ROS nodes | Next steps |
| --- | --- | --- | --- |
| Ahmad (planning) | `astar.py`, `fsm.py`, `mission.py`, `validator.py`, `llm_planner.py`, `pddl.py`, `mock_parser.py` | `path_planner`, `task_parser`, `mission_fsm` (on `fsm.py` + `mission.py`: RETRY, localization watchdog, `/refresh_map`) | try `planner:=direct` and `llmp` with a key; run E2 with a key (see below); real-robot run |
| Mourya (perception) | `overhead_geometry.py` (homography, occupancy), `overhead_camera.py` (camera pose, tags at their height), `overhead_pipeline.py` (one frame to objects, robot, grid), `colour_objects.py` (untagged cuboids by colour), `synthetic.py` (test pictures) | `overhead_node`, `calibrate_overhead`, `overhead_debug`, `overhead_colour_picker`, `fake_overhead` | mount the camera and check its view covers the room (below); intrinsics; anchors 30 to 33; measure tag heights in `objects.yaml`; empty-floor image; tune the obstacle threshold with `overhead_debug` |
| Mourya (localization) | `transform2d.py` (incl. `fit_rigid`), `pose_check.py` (fit `T_map_aruco`, PF vs overhead monitor) | `map_aruco_tf`, `aruco_map_publisher`, `pose_monitor`, `measure_t_map_aruco`, `fake_robot` | measure `T_map_aruco` with `measure_t_map_aruco` (4 to 6 spots); watch `/localization_check` during runs; then the EKF |
| Mourya (control) | `controller.py` (incl. min speed, acceleration limit, overhead correction), `sim_base.py` (simulated base) | `waypoint_follower` (with overhead alignment) | tune offline with `python3 -m roomrunner_control.sim_base`, then on the real base: measure the motor deadband and set `min_speed`; check the stops with the overhead camera |
| Sarvanagupta (manipulation) | `grasp.py` (map to base, reach, phases, IK check, holding check, tray slots, grasp offset) | `pick_place_server` (stub / real via the Assignment 03 IK node), `try_pick` | tune `grasp_z`, `grasp_offset`, gripper values and `holding_min_gripper` with `try_pick`; measure grasp heights and gripper values (`manipulation.yaml`); scripted pick with the base stopped; demonstrations and ACT (`roomrunner_manipulation/il/README.md`) |
| Everyone (experiments) | `experiments/metrics.py` (RMSE, SR) | `experiments/log_poses.py` | fill the CSVs, then `python3 experiments/analyze.py e1 / e2 / pickup` |

Placeholders to measure (all marked in their config files): `T_map_aruco.yaml`, `overhead_camera.yaml`,
`objects.yaml` (tag heights, robot clearing radius), `places.yaml`, `manipulation.yaml`.

## Overhead camera: mounting and calibration

1. **Check the view first.** The camera sees a floor patch of about (image width / focal length) x height
   by (image height / focal length) x height. A normal 1080p webcam (focal length about 1400 px) at 2.25 m
   sees only about 3.1 m x 1.7 m, so the room needs a wide-angle camera (about 1000 px or less), a higher
   mount, or a smaller arena. Cells the camera cannot see are -1 (blocked) in the grid.
2. **Intrinsics** with the Assignment 04 checkerboard script; set `intrinsics_file`. Without them the tag
   heights are ignored and the robot-top tag lands up to about 0.2 m off near the edges.
3. **Anchors** 30 to 33 flat on the floor, centres tape-measured from below wall marker 2, then
   `ros2 run roomrunner_perception calibrate_overhead --config <overhead_camera.yaml>`. It prints the camera
   height (compare with a tape) and the reprojection error (under about 3 px; more means a bad measurement).
4. **Colour cuboids** (the objects have no tags; `colour_objects.py`): put every cuboid in view, then
   `ros2 run roomrunner_perception overhead_colour_picker --camera --write` and drag a box inside each top
   face. It warns when two colours overlap in hue (pick colours far apart: red, yellow, green, blue, violet)
   and saves `overhead_colours.png` with what it finds. Measure each cuboid's `size` and `height` in
   `objects.yaml`. The same room light as on the day of the runs.
5. **Check** with `ros2 run roomrunner_perception overhead_debug --camera`: tape-measure one block and compare,
   obstacles red, the robot not red. `overhead_debug --synthetic` shows what a good result looks like.

## Localization: T_map_aruco and the overhead check

1. With the robot stack and the calibrated overhead camera running:
   `ros2 run roomrunner_localization measure_t_map_aruco --output roomrunner/roomrunner_bringup/config/T_map_aruco.yaml`.
   Drive to 4 to 6 spots spread over the room, press Enter at each. A few cm per spot is good; since the
   overhead map is built on the wall-marker frame (decisions.md, 10) the result should be close to identity.
2. During runs `pose_monitor` compares the particle filter with the robot-top tag whenever the robot has
   been still for 2 s, and re-seeds the filter at the overhead pose after 2 disagreements over 0.40 m or
   25 deg (30 s apart at most once). `ros2 topic echo /localization_check` shows each check;
   `pose_reset:=false` on robot.launch.py only reports.

## Control: tuning and the final alignment

1. **Offline:** `python3 -m roomrunner_control.sim_base` drives a test route on a simulated base (lag, motor
   deadband, pose delay) for a few gain settings and prints time, final error and distance driven. Change
   `FOLLOWER` / `BASE` in `sim_base.py` to try others.
2. **Deadband on the real base:** send small commands (`ros2 topic pub /base_commands ...`) and find the
   slowest speed that still moves the robot; set `min_speed` a little above it.
3. **Alignment:** after each stop the follower asks the overhead camera for a picture and re-drives by the
   difference between the filter and the robot-top tag (at most 2 times). The action result then reports the
   camera's error. Without the overhead camera it is skipped after 3 s (`align_with_overhead:=false` to turn it off).
4. `/robot_pose` now comes at 20 Hz from TF (filter correction times live odometry), so the follower never steers
   on a pose that is seconds old; if it gets older than 0.5 s anyway, the follower stops and waits.

## Manipulation: tuning the scripted pick

1. **Build in the conda env, without `--symlink-install`** (as for the course packages): then the nodes start
   with the conda `python3`, and `pick_place_server` can import the Assignment 03 IK (it needs MuJoCo through
   `lekiwi_sim`) to check every phase before moving. Otherwise it warns and only the IK node's refusal is caught.
2. Start the robot with `arm:=real`, put a block at a ruler-measured spot in front of the robot, then
   `ros2 run roomrunner_manipulation try_pick --x 0.25 --y 0.0 --place`. The server reads `manipulation.yaml`
   again for every goal: edit `grasp_z`, `grasp_offset` (the block must end up between the jaws, not under the
   fingertip), `gripper_open` / `gripper_closed`, and try again.
3. **Holding check:** note the gripper reading (`ros2 topic echo /arm_state`, last value) with a block held and
   with nothing; set `holding_min_gripper` between the two. A miss then fails the pick and the mission retries.
4. **Reach:** with the Assignment 03 IK, finger-down targets reach about 0.30 m in front of the base at grasp
   height, less for the hover phase. Keep `standoff` (robot.yaml) so the block lands near 0.22 to 0.26 m.
5. **Tray:** `tray_slots` are drop spots in the tray tag's frame; a slot with another object within
   `slot_clearance` is skipped. `stub_fail_rate` (stub mode) makes picks miss to try the mission's RETRY.

## Running E2 (language tasks)

The 20 requests are in `experiments/e2_requests.yaml` (1 to 4 steps, five of each, direct and indirect wording),
each with its goal as PDDL facts. A trial succeeds when the plan is valid and ends in exactly that goal
(`roomrunner_planning/goal_check.py`). The scene there must match the room during the run.

```bash
python3 experiments/run_e2.py --planners mock direct llmp          # offline: planning only, key from .env
python3 experiments/run_e2.py --ros --planners direct llmp --confirm   # on the launched system
python3 experiments/analyze.py e2                                   # SR overall and by step count
```

The mock parser is the baseline: it solves the 14 direct requests and none of the 6 indirect ones.

## Simulation (no robot)

```bash
conda activate ros_mujoco
ros2 launch roomrunner_bringup sim.launch.py viewer:=true
```

The course simulator with our room (`roomrunner_sim/config/sim_room.yaml`): three cuboids, the tray, two low
obstacles, the robot-top tag and an overhead camera 2.5 m above a 4 m x 3 m arena. The real nodes run on it: the
Assignment 04 particle filter on the simulated front camera, `map_aruco_tf`, `pose_monitor`, `overhead_node`
(pictures from `overhead/image`), the planner, the mission FSM and the follower; picks are stubs. 20 s after
the start it sends "bring the red cuboid to the tray then go home" (`request:=none` to send your own).
`/true_pose` is the simulator's ground truth, for checking the filter (E1 in simulation).

Things the simulator taught us, which hold for the real robot too:
- Wall markers must stay visible over the obstacles (in the sim they are lower than the markers); behind a tall
  obstacle the filter drifts on odometry alone.
- `robot_radius` (path_planner) must cover the base (0.15 m) plus the filter's error: 0.30 m in the sim.
- The arm, stretched out, reaches 0.41 m in front of the base: drive with it folded on the real robot.
