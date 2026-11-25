# Where each owner starts

Every module has its logic in plain Python (tested with `python3 -m pytest`, no ROS needed) and a thin ROS
node around it. `ros2 launch roomrunner_bringup stubs.launch.py` runs a whole mission with fake data.

| Owner | Plain-Python logic (tests) | ROS nodes | Next steps |
| --- | --- | --- | --- |
| Ahmad (planning) | `astar.py`, `fsm.py`, `mission.py`, `validator.py`, `llm_planner.py`, `pddl.py`, `mock_parser.py` | `path_planner`, `task_parser`, `mission_fsm` (on `fsm.py` + `mission.py`: RETRY, localization watchdog, `/refresh_map`) | try `planner:=direct` and `llmp` with a key; run E2 with a key (see below); real-robot run |
| Mourya (perception) | `overhead_geometry.py` (homography, tag poses, occupancy) | `overhead_node`, `calibrate_overhead`, `fake_overhead` | mount and calibrate the camera; anchors 30 to 33; empty-floor image; tune obstacle threshold |
| Mourya (localization) | `transform2d.py` (incl. `fit_rigid` for `T_map_aruco`) | `map_aruco_tf`, `aruco_map_publisher`, `fake_robot` | measure `T_map_aruco` with the robot-top tag; then the EKF for E1 |
| Mourya (control) | `controller.py` | `waypoint_follower` | tune gains on the real base; final alignment with the overhead pose |
| Sarvanagupta (manipulation) | `grasp.py` (map to base, reach, pick/place phases) | `pick_place_server` (stub / real via the Assignment 03 IK node) | measure grasp heights and gripper values (`manipulation.yaml`); scripted pick with the base stopped; demonstrations and ACT (`roomrunner_manipulation/il/README.md`) |
| Everyone (experiments) | `experiments/metrics.py` (RMSE, SR) | `experiments/log_poses.py` | fill the CSVs, then `python3 experiments/analyze.py e1 / e2 / pickup` |

Placeholders to measure (all marked in their config files): `T_map_aruco.yaml`, `overhead_camera.yaml`,
`places.yaml`, `manipulation.yaml`.

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
