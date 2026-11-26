# Where each owner starts

Every module has its logic in plain Python (tested with `python3 -m pytest`, no ROS needed) and a thin ROS
node around it. `ros2 launch roomrunner_bringup stubs.launch.py` runs a whole mission with fake data.

| Owner | Plain-Python logic (tests) | ROS nodes | Next steps |
| --- | --- | --- | --- |
| Ahmad (planning) | `astar.py`, `fsm.py`, `validator.py`, `llm_planner.py`, `pddl.py`, `mock_parser.py` | `path_planner`, `task_parser`, `mission_fsm` | move `mission_fsm` onto `fsm.MissionStateMachine` (RETRY handling, call `/refresh_map`); try `planner:=direct` and `llmp` with a key; E2 request set |
| Mourya (perception) | `overhead_geometry.py` (homography, tag poses, occupancy) | `overhead_node`, `calibrate_overhead`, `fake_overhead` | mount and calibrate the camera; anchors 30 to 33; empty-floor image; tune obstacle threshold |
| Mourya (localization) | `transform2d.py` (incl. `fit_rigid`), `pose_check.py` (fit `T_map_aruco`, PF vs overhead monitor) | `map_aruco_tf`, `aruco_map_publisher`, `pose_monitor`, `measure_t_map_aruco`, `fake_robot` | measure `T_map_aruco` with `measure_t_map_aruco` (4 to 6 spots); watch `/localization_check` during runs; then the EKF |
| Mourya (control) | `controller.py` | `waypoint_follower` | tune gains on the real base; final alignment with the overhead pose |
| Sarvanagupta (manipulation) | `grasp.py` (map to base, reach, pick/place phases) | `pick_place_server` (stub / real via the Assignment 03 IK node) | measure grasp heights and gripper values (`manipulation.yaml`); scripted pick with the base stopped; demonstrations and ACT (`roomrunner_manipulation/il/README.md`) |
| Everyone (experiments) | `experiments/metrics.py` (RMSE, SR) | `experiments/log_poses.py` | fill the CSVs, then `python3 experiments/analyze.py e1 / e2 / pickup` |

Placeholders to measure (all marked in their config files): `T_map_aruco.yaml`, `overhead_camera.yaml`,
`places.yaml`, `manipulation.yaml`.

## Localization: T_map_aruco and the overhead check

1. With the robot stack and the calibrated overhead camera running:
   `ros2 run roomrunner_localization measure_t_map_aruco --output roomrunner/roomrunner_bringup/config/T_map_aruco.yaml`.
   Drive to 4 to 6 spots spread over the room, press Enter at each. A few cm per spot is good; since the
   overhead map is built on the wall-marker frame (decisions.md, 10) the result should be close to identity.
2. During runs `pose_monitor` compares the particle filter with the robot-top tag whenever the robot has
   been still for 2 s, and re-seeds the filter at the overhead pose after 2 disagreements over 0.40 m or
   25 deg (30 s apart at most once). `ros2 topic echo /localization_check` shows each check;
   `pose_reset:=false` on robot.launch.py only reports.
