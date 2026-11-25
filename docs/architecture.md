# RoomRunner architecture

Every node, topic, message type and frame of the project. The overhead nodes run on Mourya's laptop (decisions.md); `occupancy_grid_node` and `object_pose_node` in the diagram are
both implemented by `overhead_node`. Nodes marked *(A03)* or *(A04)* are our
assignment nodes, launched unchanged; *(course)* nodes come from the course repository. Units are metres
and radians unless a topic says otherwise. Frames are described in [frames.md](frames.md).

## Data flow

```mermaid
flowchart LR
  subgraph robot["Robot or simulator (course)"]
    RN["robot_node / sim_node"]
  end

  subgraph perception["roomrunner_perception (Mourya)"]
    OC["overhead camera_node (A04)"]
    OD["overhead aruco_detector (A04)"]
    OG["occupancy_grid_node"]
    OP["object_pose_node"]
  end

  subgraph localization["roomrunner_localization (Mourya)"]
    FC["front camera_node (A04)"]
    FD["front aruco_detector (A04)"]
    ODO["odometry_node (A04)"]
    PF["localization_node (A04)<br/>map_frame = aruco_map"]
    AM["aruco_map_publisher"]
    TF1["map_aruco_tf"]
  end

  subgraph control["roomrunner_control (Mourya)"]
    WF["waypoint_follower"]
  end

  subgraph planning["roomrunner_planning (Ahmad)"]
    TP["task_parser"]
    PP["path_planner (A*)"]
    FSM["mission_fsm"]
  end

  subgraph manipulation["roomrunner_manipulation (Sarvanagupta)"]
    PK["pick_place_server"]
    IK["inverse_kinematics (A03 EEControl)"]
  end

  USER(["user"]) -- "/task_request<br/>String" --> TP
  TP -- "/task_spec<br/>TaskSpec" --> FSM
  FSM -- "/mission_status<br/>MissionStatus" --> USER

  OC -- "/overhead/image<br/>Image" --> OG
  OC -- "/overhead/image, /overhead/camera_info" --> OD
  OD -- "/overhead/detected_tags<br/>ArucoMarkerArray" --> OP
  OG -- "/occupancy_grid<br/>OccupancyGrid" --> PP
  OP -- "/objects<br/>ObjectPoseArray" --> TP
  OP -- "/objects" --> FSM

  RN -- "/base_velocity<br/>TwistStamped" --> ODO
  FC -- "/camera/image, /camera/camera_info" --> FD
  FD -- "/detected_markers<br/>ArucoMarkerArray" --> PF
  ODO -- "/odom<br/>Odometry" --> PF
  AM -- "/map_markers<br/>ArucoMarkerArray" --> PF
  PF -- "/pose_estimate<br/>PoseWithCovarianceStamped" --> TF1
  TF1 -- "/robot_pose<br/>PoseStamped" --> FSM
  TF1 -- "/robot_pose" --> WF

  FSM -- "/plan_path<br/>PlanPath (srv)" --> PP
  FSM -- "/navigate_to<br/>NavigateTo (action)" --> WF
  WF -- "/base_commands<br/>Float64MultiArray" --> RN
  FSM -- "/pick_place<br/>PickPlace (action)" --> PK
  PK -- "/ik_target<br/>PoseStamped" --> IK
  IK -- "/arm_commands<br/>Float64MultiArray" --> RN
  PK -- "/arm_commands (gripper)" --> RN
  RN -- "/arm_state<br/>Float64MultiArray" --> PK
```

## Topics

| Topic | Type | Frame / units | From | To |
| --- | --- | --- | --- | --- |
| `/task_request` | `std_msgs/String` | free text | user, experiment scripts | `task_parser` |
| `/task_spec` | `roomrunner_interfaces/TaskSpec` | object and place names | `task_parser` | `mission_fsm` |
| `/mission_status` | `roomrunner_interfaces/MissionStatus` | | `mission_fsm` | user, loggers |
| `/planned_path` | `nav_msgs/Path` | `map` | `mission_fsm` | RViz, loggers |
| `/overhead/image`, `/overhead/camera_info` | `sensor_msgs/Image`, `CameraInfo` | `overhead_camera` | overhead `camera_node` | perception |
| `/overhead/detected_tags` | `aruco_interfaces/ArucoMarkerArray` | `overhead_camera` | overhead `aruco_detector` | `object_pose_node` |
| `/occupancy_grid` | `nav_msgs/OccupancyGrid` | `map`, 0 free, 100 occupied, -1 unknown | `overhead_node` (laptop) | `path_planner` |
| `/objects` | `roomrunner_interfaces/ObjectPoseArray` | `map` | `overhead_node` (laptop) | `task_parser`, `mission_fsm` |
| `/overhead_robot` | `geometry_msgs/PoseStamped` | `map` | `overhead_node` (robot-top tag 40) | E1 logger, `T_map_aruco` check |
| `/camera/image`, `/camera/camera_info` | `sensor_msgs/Image`, `CameraInfo` | `camera` | front `camera_node` / `sim_node` | front `aruco_detector` |
| `/detected_markers` | `aruco_interfaces/ArucoMarkerArray` | `camera` | front `aruco_detector` | `localization_node` |
| `/map_markers` | `aruco_interfaces/ArucoMarkerArray` | `aruco_map` | `aruco_map_publisher` | `localization_node` |
| `/base_velocity` | `geometry_msgs/TwistStamped` | `base`, m/s and rad/s | `robot_node` / `sim_node` | `odometry_node` |
| `/odom` | `nav_msgs/Odometry` | `odom` | `odometry_node` | `localization_node` |
| `/pose_estimate` | `geometry_msgs/PoseWithCovarianceStamped` | `aruco_map` | `localization_node` | `map_aruco_tf` |
| `/robot_pose` | `geometry_msgs/PoseStamped` | `map` | `map_aruco_tf` (`fake_robot` in stub mode) | `mission_fsm`, `waypoint_follower` |
| `/base_commands` | `std_msgs/Float64MultiArray` | [forward m/s, left m/s, turn **deg/s**] | `waypoint_follower` | `robot_node` / `sim_node` |
| `/ik_target` | `geometry_msgs/PoseStamped` | `base` | `pick_place_server` | `inverse_kinematics` (A03) |
| `/arm_commands`, `/arm_state` | `std_msgs/Float64MultiArray` | 5 joints in **degrees**, gripper % | IK node, `pick_place_server` / robot | robot / `pick_place_server` |

## Services and actions

| Name | Type | Server | Client | Meaning |
| --- | --- | --- | --- | --- |
| `/plan_path` | `roomrunner_interfaces/PlanPath` (srv) | `path_planner` | `mission_fsm`, task validator | A* path in `map` from start to goal |
| `/navigate_to` | `roomrunner_interfaces/NavigateTo` (action) | `waypoint_follower` | `mission_fsm` | follow a path; succeed when aligned and stopped |
| `/pick_place` | `roomrunner_interfaces/PickPlace` (action) | `pick_place_server` | `mission_fsm` | pick an object or place it |
| `/refresh_map` | `std_srvs/Trigger` (srv) | `overhead_node` (laptop) | `mission_fsm` | take an overhead frame now (before planning, after pick/place) |

## Mission state machine

```mermaid
stateDiagram-v2
  [*] --> IDLE
  IDLE --> PARSE_TASK: /task_request
  PARSE_TASK --> PLAN_TO_PICKUP: valid TaskSpec
  PARSE_TASK --> FAILED: object not found
  PLAN_TO_PICKUP --> NAVIGATE: path found
  PLAN_TO_PICKUP --> FAILED: no valid path
  NAVIGATE --> ALIGN: arrived
  NAVIGATE --> RETRY: localization lost
  ALIGN --> PICK: aligned and stopped
  PICK --> PLAN_TO_DROPOFF: holding object
  PICK --> RETRY: pickup failed
  PLAN_TO_DROPOFF --> NAVIGATE: path found
  ALIGN --> PLACE: at drop-off
  PLACE --> DONE: released
  RETRY --> PLAN_TO_PICKUP: attempts left
  RETRY --> FAILED: no attempts left
  DONE --> IDLE
  FAILED --> IDLE
```

A request with several steps (E2: 1 to 4 steps) repeats the plan / navigate / align and pick or place
states once per step.

## TF tree

```mermaid
flowchart TD
  map["map<br/>(overhead planning frame)"] -->|"static: T_map_aruco<br/>config/T_map_aruco.yaml"| aruco_map["aruco_map<br/>(Assignment 04 room map)"]
  map -->|"static: overhead calibration<br/>config/overhead_camera.yaml"| overhead_camera["overhead_camera"]
  aruco_map -->|"localization_node (PF)"| odom["odom"]
  aruco_map -->|"static: aruco_map_publisher"| marker["marker_&lt;id&gt;"]
  odom -->|"odometry_node"| base["base"]
  base -->|"static: camera mount<br/>(0.132, 0, 0.043) m, pitch 9.6 deg"| camera_link["camera_link"]
  camera_link -->|"static: optical axes"| camera["camera"]
  camera -->|"front aruco_detector"| detected["detected_&lt;id&gt;"]
  base -->|"static: arm mount"| arm_base["arm_base"]
```

In stub mode there is no particle filter: `fake_robot` publishes `map → base` directly.
