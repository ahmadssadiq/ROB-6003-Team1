# Frames and transforms

All frames are right-handed with z up, except the camera optical frames (z out of the lens, x right in the
image, y down). Units: metres and radians.

| Frame | Origin and axes | Parent | Transform published by | Where the numbers come from |
| --- | --- | --- | --- | --- |
| `map` | planning frame: a fixed point on the floor seen by the overhead camera; x, y on the floor, z up. The occupancy grid and object poses are in this frame | (root) | | chosen when the overhead camera is calibrated |
| `overhead_camera` | optical frame of the overhead camera (OpenCV: x right, y down, z along the view) | `map` | static, `roomrunner_bringup` launch | `camera_pose` in `config/overhead_camera.yaml`, from `calibrate_overhead.py`: X_camera = R X_map + t, R = Rodrigues(rvec); see `overhead_camera.py` |
| `aruco_map` | Assignment 04 room map: origin on the floor below wall marker 2, x out of its wall into the room | `map` | static, `map_aruco_tf` | `config/T_map_aruco.yaml` (x, y, yaw), measured once |
| `marker_<id>` | a wall marker where the map says it is | `aruco_map` | static, `aruco_map_publisher` | `classroom_map.yaml` (Assignment 04) |
| `odom` | where odometry started | `aruco_map` | `localization_node` (A04), the particle filter's correction | particle filter |
| `base` | robot base centre (middle of the three wheels), on the base plate; x forward, y left | `odom` | `odometry_node` (A04) | wheel odometry |
| `camera_link` | front camera lens; x forward, z up | `base` | static, launch | measured: (0.132, 0, 0.043) m, pitch up 9.6 deg (Assignment 04) |
| `camera` | front camera optical frame | `camera_link` | static, launch | fixed axis swap |
| `detected_<id>` | a marker as the front camera sees it | `camera` | `aruco_detector` (A04) | detections |
| `arm_base` | SO-101 shoulder pan axis at the base plate | `base` | static, launch | `config/robot.yaml`; the Assignment 03 IK works in `base`, so this frame is for display and for the manipulation code's own checks |

## Chains used by the planner

- Robot pose for planning: `map → aruco_map → odom → base`, published as `/robot_pose` (frame `map`).
- Object for grasping: object in `map` (overhead camera) → `base` (robot pose) → sent as `/ik_target` in
  `base` to the Assignment 03 IK node.

## Fixed transforms to measure

| Transform | File | Status |
| --- | --- | --- |
| `map → aruco_map` | `roomrunner_bringup/config/T_map_aruco.yaml` | placeholder (identity) until measured with `measure_t_map_aruco` |
| `map → overhead_camera` | `roomrunner_bringup/config/overhead_camera.yaml` | placeholder until calibrated |
| `base → arm_base` | `roomrunner_bringup/config/robot.yaml` | from the Assignment 03 model: shoulder pan at (0.067, 0, 0.084) m |
| `base → camera_link` | `roomrunner_bringup/config/robot.yaml` | measured in Assignment 04 |
