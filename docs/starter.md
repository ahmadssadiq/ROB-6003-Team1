# Where each owner starts

Every module has its logic in plain Python (tested with `python3 -m pytest`, no ROS needed) and a thin ROS
node around it. `ros2 launch roomrunner_bringup stubs.launch.py` runs a whole mission with fake data.

| Owner | Plain-Python logic (tests) | ROS nodes | Next steps |
| --- | --- | --- | --- |
| Ahmad (planning) | `astar.py`, `fsm.py`, `mission.py`, `validator.py`, `llm_planner.py`, `pddl.py`, `mock_parser.py` | `path_planner`, `task_parser`, `mission_fsm` (on `fsm.py` + `mission.py`: RETRY, localization watchdog, `/refresh_map`) | try `planner:=direct` and `llmp` with a key; E2 request set; real-robot run |
| Mourya (perception) | `overhead_geometry.py` (homography, tag poses, occupancy) | `overhead_node`, `calibrate_overhead`, `fake_overhead` | mount and calibrate the camera; anchors 30 to 33; empty-floor image; tune obstacle threshold |
| Mourya (localization) | `transform2d.py` (incl. `fit_rigid` for `T_map_aruco`) | `map_aruco_tf`, `aruco_map_publisher`, `fake_robot` | measure `T_map_aruco` with the robot-top tag; then the EKF for E1 |
| Mourya (control) | `controller.py` | `waypoint_follower` | tune gains on the real base; final alignment with the overhead pose |
| Sarvanagupta (manipulation) | `grasp.py` (map to base, reach, pick/place phases) | `pick_place_server` (stub / real via the Assignment 03 IK node) | measure grasp heights and gripper values (`manipulation.yaml`); scripted pick with the base stopped; demonstrations and ACT (`roomrunner_manipulation/il/README.md`) |
| Everyone (experiments) | `experiments/metrics.py` (RMSE, SR) | `experiments/log_poses.py` | fill the CSVs, then `python3 experiments/analyze.py e1 / e2 / pickup` |

Placeholders to measure (all marked in their config files): `T_map_aruco.yaml`, `overhead_camera.yaml`,
`places.yaml`, `manipulation.yaml`.
