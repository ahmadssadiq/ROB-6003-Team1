# Where each owner starts

Every module has its logic in plain Python (tested with `python3 -m pytest`, no ROS needed) and a thin ROS
node around it. `ros2 launch roomrunner_bringup stubs.launch.py` runs a whole mission with fake data.

| Owner | Plain-Python logic (tests) | ROS nodes | Next steps |
| --- | --- | --- | --- |
| Ahmad (planning) | `astar.py`, `fsm.py`, `validator.py`, `llm_planner.py`, `pddl.py`, `mock_parser.py` | `path_planner`, `task_parser`, `mission_fsm` | move `mission_fsm` onto `fsm.MissionStateMachine` (RETRY handling, call `/refresh_map`); try `planner:=direct` and `llmp` with a key; E2 request set |
| Mourya (perception) | `overhead_geometry.py` (homography, occupancy), `overhead_camera.py` (camera pose, tags at their height), `overhead_pipeline.py` (one frame to objects, robot, grid), `synthetic.py` (test pictures) | `overhead_node`, `calibrate_overhead`, `overhead_debug`, `fake_overhead` | mount the camera and check its view covers the room (below); intrinsics; anchors 30 to 33; measure tag heights in `objects.yaml`; empty-floor image; tune the obstacle threshold with `overhead_debug` |
| Mourya (localization) | `transform2d.py` (incl. `fit_rigid` for `T_map_aruco`) | `map_aruco_tf`, `aruco_map_publisher`, `fake_robot` | measure `T_map_aruco` with the robot-top tag; then the EKF for E1 |
| Mourya (control) | `controller.py` | `waypoint_follower` | tune gains on the real base; final alignment with the overhead pose |
| Sarvanagupta (manipulation) | `grasp.py` (map to base, reach, pick/place phases) | `pick_place_server` (stub / real via the Assignment 03 IK node) | measure grasp heights and gripper values (`manipulation.yaml`); scripted pick with the base stopped; demonstrations and ACT (`roomrunner_manipulation/il/README.md`) |
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
4. **Check** with `ros2 run roomrunner_perception overhead_debug --camera`: tape-measure one block and compare,
   obstacles red, the robot not red. `overhead_debug --synthetic` shows what a good result looks like.
