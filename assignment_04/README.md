# Assignment 4: particle filter localization with ArUco markers

A particle filter that localizes the LeKiwi robot in a map of ArUco markers, using the camera and the
wheel odometry, in simulation and in the classroom. Built on the course repository
[FoR26, branch assignment4](https://github.com/lair-research-nyu/FoR26/tree/assignment4).

## Code deliverables

| Deliverable | File |
| --- | --- |
| `aruco_detector.py` | [aruco_localization/aruco_localization/aruco_detector.py](aruco_localization/aruco_localization/aruco_detector.py): `marker_pose_from_corners()` |
| `particle_filter.py` | [aruco_localization/aruco_localization/particle_filter.py](aruco_localization/aruco_localization/particle_filter.py): `motion_update()`, `expected_measurement()`, `measurement_log_likelihood()`, `low_variance_resample()` |
| `localization_node.py` | [aruco_localization/aruco_localization/localization_node.py](aruco_localization/aruco_localization/localization_node.py): `measurement_from_detection()` |
| `camera_intrinsics.yaml` | [aruco_localization/config/camera_intrinsics.yaml](aruco_localization/config/camera_intrinsics.yaml) |
| `config/localization.yaml` | [aruco_localization/config/localization.yaml](aruco_localization/config/localization.yaml), unchanged from the course defaults |
| Launch arguments | below |

### Launch arguments used (real robot)

Camera mount measured on our robot: 0.132 m forward of the base centre, on the centre line, 0.043 m
above the base frame origin. The tilt was first measured as 12 deg and corrected to 9.6 deg from the
marker distance experiment (the detected marker height is only constant with 9.6 deg).

```bash
ros2 launch aruco_localization localization.launch.py robot:=real \
    port:=/dev/serial/by-id/usb-1a86_USB_Single_Serial_5B79017378-if00 enable_motors:=true \
    camera_device:=0 camera_x:=0.132 camera_y:=0.0 camera_z:=0.043 camera_pitch:=9.6
```

`camera_device` is the number of `/dev/video*` that the camera gets; it changed to 1 once after the
cables were replugged.

Simulation: `ros2 launch aruco_localization localization.launch.py` (all defaults).

## Contents

| Path | What it is |
| --- | --- |
| `aruco_localization/` | the ROS 2 package with our code (copy it to `~/FOR26/ros_ws/src/`) |
| `tools/marker_distance.py` | D2: detected against tape-measured marker distance |
| `tools/spot_accuracy.py` | L3: estimated against tape-measured floor positions |
| `tools/plot_a4.py` | L1 and L2 plots |
| `tools/run_l1_sim.sh`, `tools/run_l2_sim.sh` | the simulation runs for L1 and L2 |
| `data/a4_calibration/` | C1: calibration result, picture with corners, the 33 calibration pictures |
| `data/d2_distance/` | D2: `distance.csv` (tilt 9.6 deg), `distance_raw_pitch12.csv` (as recorded at 12 deg) |
| `data/a4_l1_sim/` | L1: unknown and known start runs |
| `data/a4_l2_sim/`, `data/l2_real/` | L2: marker consistency, simulation and real robot |
| `data/l3_real/` | L3: `spots.csv` (used in the report), `spots_as_typed.csv` (as typed during the run) |
| `videos/` | D1 (40 s), L-sim (70 s, trim to 60 s before submitting), L-real RViz screen recording (75 s) |
| `report/` | the IEEE report (LaTeX; upload the folder to Overleaf) |

The phone video of the robot for L-real is not in this repository.

## Results

| Experiment | Result |
| --- | --- |
| C1 calibration | RMS reprojection error 0.41 px, fx 337.9, fy 337.5, cx 344.1, cy 264.1, FOV 86.9 x 70.8 deg |
| D2 marker distance | error -11 mm + 3.7 % of the distance, 0.5 to 3 m |
| L1 simulation | unknown start below 20 cm after 9.2 s, mean 8.9 cm; odometry heading drift 5.8 deg in 115 s |
| L2 marker consistency | median 1.3 cm within 1.5 m (real), growing with distance |
| L3 real robot | 5.7 to 14.6 cm at four floor spots, mean 9.1 cm |

## Reproducing the plots

```bash
python tools/marker_distance.py d2_distance --plot
python tools/plot_a4.py l1 a4_l1_sim
python tools/plot_a4.py l2 a4_l2_sim
python tools/plot_a4.py l2 l2_real
python tools/spot_accuracy.py l3_real --plot
```

The tools expect the data under `~/FOR26/ros_ws/data/`, as on the robot's computer.
