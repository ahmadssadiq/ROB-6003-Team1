# Design decisions

Agreed decisions, so the whole team builds against the same assumptions. Change them here first.

| # | Decision | Why |
| --- | --- | --- |
| 1 | Localization: reuse the Assignment 04 particle filter for the first end-to-end run; add a 2D EKF afterwards and compare odometry, PF and EKF in E1 | the PF already works on the real robot; the proposal describes an EKF |
| 2 | Manipulation uses the Assignment 03 EEControl node (`inverse_kinematics`) as it is | already integrated with ROS and the robot driver |
| 3 | Packages live in this repository under `roomrunner/`, symlinked into `~/FOR26/ros_ws/src` | one `colcon build` covers course and project packages; nothing existing moves |
| 4 | The robot body frame keeps the name `base` (not `base_link`) | the Assignment 03/04 nodes and the camera mount use `base` |
| 5 | The mini PC rides on the robot. The overhead camera plugs into Mourya's laptop, which stays still | a USB cable cannot follow the robot |
| 6 | Mourya's laptop runs the overhead pipeline (camera, occupancy grid, object poses) and sends only the results (`/occupancy_grid`, `/objects`) to the robot, not images | a few kB instead of video, so network delay does not matter |
| 7 | The overhead camera takes one frame every 10 s, and immediately when the mission FSM asks (before each plan, after each pick and place) | the camera and room are static; the map only changes when an object moves |
| 8 | Mourya's laptop runs Ubuntu with ROS 2 Jazzy, so the overhead nodes run there as normal ROS nodes. Laptop and mini PC are both on Tailscale and find each other through a Fast DDS discovery server on the mini PC (Tailscale carries no multicast). Setup: [networking.md](networking.md) | works on any network, including from home |
| 9 | Overhead camera: wide-angle 1080p USB webcam, 2 to 2.5 m above the floor, looking straight down | a 50 mm tag is still about 20 to 25 px wide from 2.5 m |
| 10 | Overhead calibration: intrinsics with the Assignment 04 checkerboard script; extrinsics from 4 ArUco anchor tags on the floor whose positions are tape-measured from wall marker 2. The overhead map frame is then the wall-marker frame, so `T_map_aruco` is the identity, measured | one frame for robot, objects and obstacles, no separate registration |
| 11 | A tag on top of the robot checks the registration (overhead pose against PF pose) and gives E1 its overhead-camera ground truth | |
| 12 | Tags: dictionary `DICT_4X4_50` everywhere. Ids: walls 1, 2, 4, 7, 8 (206.5 mm); blocks 10 to 19 (50 mm, not used: see 15); tray 20 to 29 (100 mm); floor anchors 30 to 33 (150 mm); robot top 40 (100 mm) | separate id ranges; the PF ignores ids that are not in its map |
| 13 | LLM: OpenAI and Anthropic both supported behind one interface, chosen by `ROOMRUNNER_LLM_PROVIDER` in `.env`; mock mode is the default | use whichever the team has credits for |
| 14 | Simulation: a `roomrunner_sim` package that reuses the course simulator (wall markers, tagged cubes, tray, overhead camera) without editing course code | course code stays untouched |
| 15 | The objects are 3 to 5 cuboids of different colours, without tags. The overhead camera finds them by colour (HSV range, size check, top-face plane, visible-side correction; `colour_objects.py`) to within about 1 cm, enough to drive there; ACT does the grasp from the robot's cameras | a tag small enough for a 2 cm wide cuboid is about 8 px from 2.25 m, too small to read; the colours are distinct |

## Open

- The camera model actually used, and its mounting height.
